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
#include "core/core-dev.h"
#include <thread>

void check_and_build_dir (const std::string &path);
int download_tile (const std::string& uri, const std::string& filepath, const std::string& proxy);
int load_image_from_geoportail (int z, int row, int col, const std::string& name);
void load_image_from_osm (djnn::Process* tile, int z, int row, int col, const std::string& name);
void load_osm_tile (djnn::Process* src);
void load_geoportail_tile (djnn::Process* src);


class tiles_manager {       
    public:
        tiles_manager (int maxpool=8) : 
            _maxpool(maxpool) {};
        void run ();
        void add_to_queue (std::tuple<djnn::Process*, std::string, std::string> p) ;
        void download_tile ();
    private:
        int _maxpool;
        // std::list <std::tuple<djnn::Process*, std::string, std::string>> request_queue ;
        std::vector <std::thread> thread_pool;
};