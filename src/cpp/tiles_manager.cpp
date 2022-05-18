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

#include "gui/gui.h"

#include "tiles_manager.h"
#include <chrono>
#include <thread>

#include <cmath>
#include <iostream>
#include "exec_env/global_mutex.h"

//#include "core/utils/getset.h"

#include "base/native_async_action.h"

#include "cpp/coords-utils.h"

#include "core/utils/error.h"

using namespace std;


#include "core/utils/filesystem.h"

namespace curl { // fix 'Rectangle' clash name for windowss
#include <curl/curl.h>
#include <curl/easy.h>
}

enum source_t {OSM, GEOPORTAIL};

size_t
mycurl_write_callback(char *ptr, size_t size, size_t nmemb, FILE* stream)
{
    size_t written;
    assert (size);
    written = fwrite (ptr, size, nmemb, stream);
    return written;
}

void check_and_build_dir (const std::string &path)
{
  if(filesystem::exists(path))
    return;
  filesystem::create_directories(path);
}

int download_tile (int z, int row, int col, const std::string &uri, const std::string &filepath)
{
  using namespace curl;
  CURL *curl = curl_easy_init ();
  if (!curl) {
    djnn::lock_ios_mutex ();
    std::cerr << "Error setting curl" << std::endl;
    djnn::release_ios_mutex ();
    return 1;
  }

  auto tries = 3;
  std::cerr << uri << std::endl;
  FILE* fp = nullptr;
  while (tries && fp == nullptr) {
    errno = 0;
    fp = fopen(filepath.c_str(), "wb");
    if (!fp) {
      if(errno == EMFILE) {
        djnn::lock_ios_mutex ();
        std::cerr << "too many open files, retrying later..." << __FL__;
        djnn::release_ios_mutex ();
        usleep(500'000+random()*300'000);
      } else {
        break;
      }
    }
    --tries;
  }

  //fp = fopen(filepath.c_str(), "wb");
  if (fp == nullptr) {
    djnn::lock_ios_mutex ();
    std::cerr << "Fail to create file " << filepath << std::endl;
    perror("");
    djnn::release_ios_mutex ();
    curl_easy_cleanup (curl);
    return 2;
  }

  //std::cout << "download: " << uri << std::endl;
  curl_easy_setopt(curl, CURLOPT_URL, uri.c_str ());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mycurl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  // it seems that we need to tell to geoportail that we are a browser
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/62.0.3202.94 Safari/537.36");
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  // FIXME: should be cancellable with https://curl.se/libcurl/c/CURLOPT_XFERINFOFUNCTION.html
  auto res = curl_easy_perform (curl);

  // circumvent errors possibly due to batch HTTP GET // not very fair :-/....
  tries = 3;
  while (tries && res != CURLE_OK) {
    if (res == CURLE_COULDNT_CONNECT || CURLE_SSL_CACERT_BADFILE) {
      usleep(500'000);
      res = curl_easy_perform (curl);
    }
    --tries;
  }
  // FIXME: when definitely not ok, the code should behave in a better way... like it's not updating later...

  if (res != CURLE_OK) {
    djnn::lock_ios_mutex ();
    std::cerr << "Error performing curl: " << res << " " /*<< uri << " error: "*/ << curl_easy_strerror (res) << std::endl;
    fclose(fp);
    int err = remove(filepath.c_str());
    if(err) perror("");
    djnn::release_ios_mutex ();
    assert (!filesystem::exists(filepath));
    curl_easy_cleanup (curl);
    return 3;
  }

  curl_easy_cleanup (curl);
  fclose(fp);
  assert (filesystem::exists(filepath));
  assert (std::filesystem::file_size(filepath));
  return 0;
}

int load_image_from_geoportail (int z, int row, int col, const std::string& name)
{
  std::string uri = "https://wxs.ign.fr/essentiels/geoportail/wmts?LAYER=ORTHOIMAGERY.ORTHOPHOTOS&EXCEPTIONS=text/xml&FORMAT=image/jpeg&SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile&STYLE=normal&TILEMATRIXSET=PM&TILEMATRIX=" 
                    + std::to_string(z)
                    + "&TILEROW=" 
                    + std::to_string(row) 
                    + "&TILECOL="
                    + std::to_string(col) + "&";
  std::string filepath =  "cache/" + name + "/" + std::to_string(z) + "_" + std::to_string(row) + "_" + std::to_string(col) + ".jpg";
  if (!filesystem::exists(filepath)) {
    auto res = download_tile (z, row, col, uri, filepath);
    return res;
  } else {
    if (std::filesystem::file_size(filepath)==0) { // file is still in creation FIXME
      return 1;
    }
  }
  return 0;
}

int load_image_from_osm (int z, int row, int col, const std::string& name)
{
  std::string uri = "http://a.tile.openstreetmap.fr/osmfr/"
                    + std::to_string(z)
                    + "/" 
                    + std::to_string(col) 
                    + "/"
                    + std::to_string(row) + ".png";
  std::string filepath =  "cache/" + name + "/" + std::to_string(z) + "_" + std::to_string(col) + "_" + std::to_string(row) + ".png";
  if (!filesystem::exists(filepath)) {
    auto res = download_tile (z, row, col, uri, filepath);
    return res;
  } else {
    if (std::filesystem::file_size(filepath)==0) { // file is still in creation FIXME
      return 1;
    }
  }
  return 0;
}

void
load_osm_tile (djnn::Process* src) {
  djnn::get_exclusive_access(DBG_GET);
  auto * native = dynamic_cast<djnn::NativeAsyncAction*> (src);
  assert(src);
  djnn::Process* data = (djnn::Process*) get_native_user_data (src);
  //GET_CHILD_VALUE (x, djnn::Int, data, "X");
  int x = getInt (data->find_child("X"));
  int y = getInt (data->find_child("Y"));
  int z = getInt (data->find_child("Z"));
  //GET_CHILD_VALUE (name, djnn::Text, data, "layer_name");
  std::string name = ((djnn::AbstractProperty*) (data->find_child("layer_name")))->get_string_value ();
  
  //release_exclusive_access(DBG_REL);
  std::string new_path = "src/img/default.png";
  
  int max = pow (2, z);
  if (x < max && y < max) {
    //SET_CHILD_VALUE(Text, data, "img/path", new_path, true);
    ((djnn::AbstractProperty*) (data->find_child("img/path")))->set_value(new_path, true);

    djnn::release_exclusive_access(DBG_REL);
    int failure = load_image_from_osm (z, y, x, name);
    djnn::get_exclusive_access(DBG_GET);

    if (! native->should_i_stop()) {
      if (!failure) {
        new_path = "cache/" + name + "/" + std::to_string (z) + "_" + std::to_string (x) + "_" + std::to_string (y) + ".png";
        //std::cerr << new_path << std::endl;
        assert (filesystem::exists(new_path));
        assert (std::filesystem::file_size(new_path));
      }
    }
  }
  ((djnn::AbstractProperty*) (data->find_child("img/path")))->set_value(new_path, true);
  djnn::release_exclusive_access(DBG_REL);
}

void
load_geoportail_tile (djnn::Process* src) {
  djnn::get_exclusive_access(DBG_GET);
  auto * native = dynamic_cast<djnn::NativeAsyncAction*> (src);
  assert(src);
  djnn::Process* data = (djnn::Process*) get_native_user_data (src);
  int x = getInt (data->find_child("X"));
  int y = getInt (data->find_child("Y"));
  int z = getInt (data->find_child("Z"));
  std::string name = ((djnn::AbstractProperty*)(data->find_child("layer_name")))->get_string_value ();
  //release_exclusive_access(DBG_REL);
  std::string new_path = "src/img/default.png";
  
  int max = pow (2, z);
  if (x < max && y < max) {
    ((djnn::AbstractProperty*) (data->find_child("img/path")))->set_value(new_path, true);
    djnn::release_exclusive_access(DBG_REL);
    int failure;
    failure = load_image_from_geoportail (z, y, x, name);
    djnn::get_exclusive_access(DBG_GET);
    if (native->should_i_stop()) {
      //DBG;
      djnn::release_exclusive_access(DBG_REL);
      return;
    } else {
      //DBG;
    }
    if (!failure) {
      new_path = "cache/" + name + "/" + std::to_string (z) + "_" + std::to_string (y) + "_" + std::to_string (x) + ".jpg";

      //get_exclusive_access(DBG_GET);
      //std::cerr << new_path << std::endl;
      assert (filesystem::exists(new_path));
      assert (std::filesystem::file_size(new_path));
      ((djnn::AbstractProperty*) (data->find_child("img/path")))->set_value(new_path, true);
    }
    djnn::release_exclusive_access(DBG_REL);
  } else {
    std::cout << "Warning: out of bounds x (" << x << ") or y (" << y << "), max is " << max << "\n";
    ((djnn::AbstractProperty*) (data->find_child("img/path")))->set_value(new_path, true);
    djnn::release_exclusive_access(DBG_REL);
  }
}
