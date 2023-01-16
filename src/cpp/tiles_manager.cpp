/*
 *	Smala Map component
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#include <string>
#include <iostream>
#include <stdio.h>
#include <curl/curl.h>
#include <stdlib.h>
#include <unistd.h>
#include <cassert>
#include <thread>
// #include <semaphore> C++20

namespace curl {
#include <curl/curl.h>
#include <curl/easy.h>
}

#include <semaphore.hpp> // without C++20
#include "gui/gui.h"
#include "tiles_manager.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include "exec_env/global_mutex.h"
#include "base/native_async_action.h"
#include "cpp/coords-utils.h"
#include "core/utils/error.h"
#include "core/utils/filesystem.h"
using namespace std;

#define OSM_BASE_URI  "http://a.tile.openstreetmap.fr/osmfr/"
#define GEOPORTAIL_BASE_URI "https://wxs.ign.fr/essentiels/geoportail/wmts?layer=ORTHOIMAGERY.ORTHOPHOTOS&style=normal&tilematrixset=PM&Service=WMTS&Request=GetTile&Version=1.0.0&Format=image%2Fjpeg&TileMatrix="

#define MAX_POOL 8
struct __request { 
  djnn::Process* tile;
  int Z;
  int X;
  int Y;
  string uri;
  string filepath;
  string proxy;
} ;

semaphore sem_pool(MAX_POOL); 
semaphore sem_wr(1); 
static std::list <__request> request_queue ;
static bool __init_tiles_manager = false;
static std::vector <std::thread> thread_pool(MAX_POOL);

static size_t write_data(void* ptr, size_t size, size_t nmemb, void* stream)
{
  size_t written = fwrite(ptr, size, nmemb, (FILE*)stream);
  return written;
}

void check_and_build_dir(const std::string& path)
{
  if (filesystem::exists(path))
    return;
  filesystem::create_directories(path);
}

static int
download_tile(const std::string& uri, const std::string& filepath, const std::string& proxy)
{
  using namespace curl;
  CURL* curl = curl_easy_init();
  if (!curl) {
    djnn::lock_ios_mutex();
    std::cerr << "Error setting curl" << std::endl;
    djnn::release_ios_mutex();
    return 1;
  }

#if 0 //DEBUG
  djnn::lock_ios_mutex();
  std::cerr << "----->" << uri << " - " << filepath << std::endl;
  djnn::release_ios_mutex();
#endif

  FILE* fp = nullptr;
  errno = 0;
  fp = fopen(filepath.c_str(), "wb");
  if (!fp) {
    djnn::lock_ios_mutex();
    std::cerr << "ERROR - tile_manager - Fail to create file " << filepath << std::endl;
    perror("");
    if (errno == EMFILE) {
      std::cerr << "ERROR - tile_manager - too many open files, retrying later..." << __FL__ ;
    }
    djnn::release_ios_mutex();
    curl_easy_cleanup(curl);
    return 2;
  }

  //std::cout << "download: " << uri << std::endl;
  curl_easy_setopt(curl, CURLOPT_URL, uri.c_str());
  if (!proxy.empty())
    curl_easy_setopt(curl, CURLOPT_PROXY, proxy.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  // it seems that we need to tell to geoportail that we are a browser
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/62.0.3202.94 Safari/537.36");
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  /* complete within 3 seconds */
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);
  
  auto res = curl_easy_perform(curl);

  if (res != CURLE_OK) {
    djnn::lock_ios_mutex();
    std::cerr << "ERROR -- performing curl: " << res << " " /*<< uri << " error: "*/ << curl_easy_strerror(res) << std::endl;
    fclose(fp);
    int err = remove(filepath.c_str());
    if (err) perror("");
    djnn::release_ios_mutex();
    assert(!filesystem::exists(filepath));
    curl_easy_cleanup(curl);
    return 3;
  }

  curl_easy_cleanup(curl);
  fclose(fp);

#if 0 //DEBUG
  djnn::lock_ios_mutex();
  std::cerr << "<-----" << uri << " - " << filepath << std::endl;
  djnn::release_ios_mutex();
#endif

  return 0;
}

static void
each_slot_does () {

  int error = 0;
  while (1){
    sem_pool.acquire (); // wait for a slot
    sem_wr.acquire (); // wait for request_queue

    if (!request_queue.empty()) {

      // copy data from queue
      __request t = request_queue.front ();
      request_queue.pop_front ();
      sem_wr.release (); // release request_queue after poping

      //work
      error = download_tile (t.uri, t.filepath, t.proxy); // bloquing until curl did not finish
      if (!error) { 
        djnn::get_exclusive_access(DBG_GET); // wait for djnn
        int X = getInt(t.tile->find_child("X"));
        int Y = getInt(t.tile->find_child("Y"));
        int Z = getInt(t.tile->find_child("Z"));
        if (Z == t.Z && Y == t.Y && t.X == X) {
          ((djnn::AbstractProperty*)(t.tile->find_child("img/path")))->set_value(t.filepath, true);
        }
        // else {
        //   djnn::lock_ios_mutex();
        //   std::cerr << " -- ERROR !: " << Z << " - " << X << " - " << Y << " NOT Matched !!" << std::endl;
        //   djnn::release_ios_mutex();
        // }
        djnn::release_exclusive_access(DBG_GET); // realse djnn
      }
    }
    else {
      sem_wr.release (); // release request_queue if empty
    }
  } 
}

static void 
wakeup_tiles_manager () {
  if (!__init_tiles_manager) {
    __init_tiles_manager = true;

    for (int i = 0; i < MAX_POOL; ++i) {
    thread_pool.push_back(std::thread(each_slot_does));
    }
  }
}

static void 
add_to_request_queue (__request& p) {
  
  sem_wr.acquire (); // wait for request_queue
  request_queue.push_back (p);
  sem_wr.release (); // release reuest_queue

  wakeup_tiles_manager (); //if necessary

  sem_pool.release (); // release a slot
}

static void
build_request (djnn::Process* tile, int z, int row, int col, const std::string& filepath, const std::string& layer_name)
{ 
  // proxy
  std::string proxy = ((djnn::AbstractProperty*)(tile->find_child("proxy")))->get_string_value();

  // uri from layer_name
  std::string uri = "UNKNOWN_LAYER";
  if (layer_name.compare ("osm") == 0 )
      uri = OSM_BASE_URI + std::to_string(z) + "/" + std::to_string(col) + "/" + std::to_string(row) + ".png";
  else if (layer_name.compare ("geoportail") == 0 )
      uri = GEOPORTAIL_BASE_URI + std::to_string(z) + "&TileRow=" + std::to_string(row) + "&TileCol=" + std::to_string(col);
  else {
    std::cerr << "ERROR - UNKNOWN layer \"" << layer_name << "\" -- should be: osm/geoportail" << std::endl;
    return;
  }

  __request t;
  t.tile = tile;
  t.Z = z;
  t.X = col;
  t.Y = row;
  t.uri =  uri;
  t.filepath =  filepath;
  t.proxy =  proxy;

  add_to_request_queue (t);
}

void
load_tiles_from(djnn::Process* src) {

  assert(src);
  djnn::Process* tile = (djnn::Process*)get_native_user_data(src);

  int X = getInt(tile->find_child("X"));
  int Y = getInt(tile->find_child("Y"));
  int Z = getInt(tile->find_child("Z"));
  const std::string& layer_name = ((djnn::AbstractProperty*)(tile->find_child("layer_name")))->get_string_value();

  std::string path = "src/img/default.png";

  int max = pow(2, Z);
  if (X < max && Y < max) {
    std::string new_path = "cache/" + layer_name + "/" + std::to_string(Z) + "_" + std::to_string(X) + "_" + std::to_string(Y) + ".png";

    // if file do not exist or empty
    if (!filesystem::exists(new_path) || std::filesystem::file_size(new_path) == 0) {
      build_request (tile, Z, Y, X, new_path, layer_name);
      // djnn::lock_ios_mutex();
      // std::cerr << new_path << " doesn't exist or is empty" << std::endl;
      // djnn::release_ios_mutex();
    }
    else {
      // djnn::lock_ios_mutex();
      // std::cerr << new_path << " LOADING cache" << std::endl;
      // djnn::release_ios_mutex();
      path = new_path;
    }
  }
  else {
    djnn::lock_ios_mutex();
    std::cerr << "Warning - load_osm_tile - out of bounds x (" << X << ") or y (" << X << "), max is " << max << " (Z = " << Z << ")" << std::endl;
    djnn::release_ios_mutex();
  }
  ((djnn::AbstractProperty*)(tile->find_child("img/path")))->set_value(path, true);
}