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
#include <semaphore>

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

#define MAX_POOL 8

using namespace std;


#include "core/utils/filesystem.h"

namespace curl { // fix 'Rectangle' clash name for windowss
#include <curl/curl.h>
#include <curl/easy.h>
}

enum source_t { OSM, GEOPORTAIL };

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


static std::counting_semaphore<MAX_POOL> sem {MAX_POOL};
static std::binary_semaphore sem_wr{1};
static std::list <std::tuple<djnn::Process*, std::string, std::string>> request_queue ;
static bool __init_tiles_manager = false;
static int request_queue_count = 0;
static std::vector <std::thread> thread_pool(MAX_POOL);

int nb_download = 0 ;

void
each_slot_does () {

  // debug
  // djnn::lock_ios_mutex();
  // std::cerr << "\n" << " -- Thread " << std::this_thread::get_id() << " STARTED " << std::endl;
  // djnn::release_ios_mutex();

  bool done = false;
  int error = 0;
  while (!done){
    sem.acquire (); // wait for a slot
    sem_wr.acquire (); // wait for request_queue
    
    if (!request_queue.empty()) {
      // copy data from queue
      std::tuple<djnn::Process*, std::string, std::string> t = request_queue.front ();
      request_queue.pop_front ();
      sem_wr.release (); // release request_queue after poping
      // djnn::lock_ios_mutex();
      // std::cerr << "\n" << nb_download++ << " -- DOWNLOADING: " << std::get<1>(t) << " - file: " << std::get<2>(t) << std::endl;
      // djnn::release_ios_mutex();
      
      //work
      error = download_tile (std::get<1>(t), std::get<2>(t), ""); // bloquing until curl did not finish
      if (!error) { 
        djnn::get_exclusive_access(DBG_GET); // wait for djnn
        ((djnn::AbstractProperty*)(std::get<0>(t)->find_child("img/path")))->set_value(std::get<2>(t), true);
        djnn::release_exclusive_access(DBG_GET); // realse djnn
      }
    }
    else {
      sem_wr.release (); // release request_queue if empty
      done = true; 
    }
    sem.release (); // release slot
  } 
  // debug
  // djnn::lock_ios_mutex();
  // std::cerr << "\n" << " -- Thread " << std::this_thread::get_id() << " DONE ! " << std::endl;
  // djnn::release_ios_mutex();
}

void
run_manager () {

  // debug
  // djnn::lock_ios_mutex();
  // std::cerr << " -- Thread  RUN " << std::this_thread::get_id() << " STARTED " << std::endl;
  // djnn::release_ios_mutex();

  for (int i = 0; i < MAX_POOL; ++i) {
    thread_pool.push_back(std::thread(each_slot_does));
  }
  for (std::thread &t: thread_pool)
    if(t.joinable()){t.join();}
  // uninit tiles_manager
  __init_tiles_manager = false ;

  //debug
  // djnn::lock_ios_mutex();
  // std::cerr << "\n" << " -- Thread  RUN " << std::this_thread::get_id() << " DONE " << std::endl;
  // djnn::release_ios_mutex();
}

void init_tiles_manager () {

  if (!__init_tiles_manager) {
    __init_tiles_manager = true;
    std::thread t (run_manager);
    t.detach ();
  }

}

void 
add_to_queue (std::tuple<djnn::Process*, std::string, std::string> p) {
  // debug
  // djnn::lock_ios_mutex();
  // std::cerr << __FUNCTION__  << std::endl;
  // djnn::release_ios_mutex();
  sem_wr.acquire (); // wait for request_queue
  request_queue.push_back (p);
  sem_wr.release (); // release reuest_queue
}

int download_tile(const std::string& uri, const std::string& filepath, const std::string& proxy)
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

int load_image_from_geoportail(int z, int row, int col, const std::string& name, const std::string& proxy)
{
  std::string uri = "https://wxs.ign.fr/essentiels/geoportail/wmts?layer=ORTHOIMAGERY.ORTHOPHOTOS&style=normal&tilematrixset=PM&Service=WMTS&Request=GetTile&Version=1.0.0&Format=image%2Fjpeg&TileMatrix="
    + std::to_string(z)
    + "&TileRow="
    + std::to_string(row)
    + "&TileCol="
    + std::to_string(col);

  std::string filepath = "cache/" + name + "/" + std::to_string(z) + "_" + std::to_string(col) + "_" + std::to_string(row) + ".jpg";

  auto res = 0;
  while (!filesystem::exists(filepath)) {
    // debug
    // djnn::lock_ios_mutex();
    // std::cerr << "----->" << filepath << " NOT exist" << std::endl;
    // djnn::release_ios_mutex();
    res = download_tile(uri, filepath, proxy);
  }

  // file exist but size is null ? => try again
  while (filesystem::exists(filepath) && std::filesystem::file_size(filepath) == 0) {
    // debug
    // djnn::lock_ios_mutex();
    // std::cerr << "----->" << filepath << " EMPTY " << std::endl;
    // djnn::release_ios_mutex();
    res = download_tile(uri, filepath, proxy);
  }

  assert(filesystem::exists(filepath));
  assert(std::filesystem::file_size(filepath) != 0);

  return res;
}

void
load_image_from_osm (djnn::Process* tile, int z, int row, int col, const std::string& name)
{
  std::string uri = "http://a.tile.openstreetmap.fr/osmfr/"
    + std::to_string(z)
    + "/"
    + std::to_string(col)
    + "/"
    + std::to_string(row) + ".png";
  std::string filepath = "cache/" + name + "/" + std::to_string(z) + "_" + std::to_string(col) + "_" + std::to_string(row) + ".png";

  add_to_queue (std::make_tuple(tile, uri, filepath));

  //there is already something in the queue when the threads start
  init_tiles_manager ();
}

void
load_osm_tile(djnn::Process* src) {

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
      load_image_from_osm(tile, Z, Y, X, layer_name);
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

void
load_geoportail_tile(djnn::Process* src) {
  djnn::get_exclusive_access(DBG_GET);
  assert(src);
  auto* native = dynamic_cast<djnn::NativeAsyncAction*> (src);
  djnn::Process* data = (djnn::Process*)get_native_user_data(src);
  int x = getInt(data->find_child("X"));
  int y = getInt(data->find_child("Y"));
  int z = getInt(data->find_child("Z"));
  std::string name = ((djnn::AbstractProperty*)(data->find_child("layer_name")))->get_string_value();
  std::string proxy = ((djnn::AbstractProperty*)(data->find_child("proxy")))->get_string_value();
  std::string new_path = "src/img/default.png";

  int max = pow(2, z);
  if (x < max && y < max) {
    ((djnn::AbstractProperty*)(data->find_child("img/path")))->set_value(new_path, true);
    
    djnn::release_exclusive_access(DBG_REL);
    int failure;
    failure = load_image_from_geoportail(z, y, x, name, proxy);
    djnn::get_exclusive_access(DBG_GET);
    
    if (native->should_i_stop()) {
      //DBG;
      djnn::release_exclusive_access(DBG_REL);
      return;
    }
    else {
      //DBG;
    }
    if (!failure) {
      new_path = "cache/" + name + "/" + std::to_string(z) + "_" + std::to_string(x) + "_" + std::to_string(y) + ".jpg";
      assert(filesystem::exists(new_path));
      assert(std::filesystem::file_size(new_path) != 0);
      ((djnn::AbstractProperty*)(data->find_child("img/path")))->set_value(new_path, true);
    }
    djnn::release_exclusive_access(DBG_REL);
  }
  else {
    std::cout << "Warning: out of bounds x (" << x << ") or y (" << y << "), max is " << max << "\n";
    ((djnn::AbstractProperty*)(data->find_child("img/path")))->set_value(new_path, true);
    djnn::release_exclusive_access(DBG_REL);
  }
}
