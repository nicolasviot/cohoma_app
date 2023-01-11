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
void load_tiles_from (djnn::Process* src);