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

void check_and_build_dir (const std::string &path);
int download_tile (int z, int row, int col);
int load_image_from_geoportail (int z, int row, int col, const std::string& name);
int load_image_from_osm (int z, int row, int col, const std::string& name);
void load_osm_tile (djnn::Process* src);
void load_geoportail_tile (djnn::Process* src);
