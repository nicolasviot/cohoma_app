use core
use gui
use base
use display

import TaskAreaSummit

_native_code_
%{
	#include "cpp/coords-utils.h"
	#include <iostream>
%}

_define_
EnvMapLayer (Process map){

	Double visibility_map_resolution (1)

	TaskAreaSummit georef_visibility_map (map, 0, 0)
	Translation t_to_georef (0, 0)
	Translation t_to_center (0, 0)
	Scaling  scale (1, 1, 0, 0)
	DataImage visibility_map (0,0,0,0)

	georef_visibility_map.x => t_to_georef.tx
	georef_visibility_map.y => t_to_georef.ty
	// beynes : 1.52, esperces 1.36, caylus 1.40
	visibility_map_resolution * map.scaling_factor_correction / get_resolution ($map.zoomLevel) =:> scale.sx
	visibility_map_resolution * map.scaling_factor_correction / get_resolution ($map.zoomLevel) =:> scale.sy

	- (visibility_map.width * visibility_map_resolution * map.scaling_factor_correction/ get_resolution ($map.zoomLevel)) / 2 =:> t_to_center.tx
	- (visibility_map.height * visibility_map_resolution * map.scaling_factor_correction/ get_resolution ($map.zoomLevel)) / 2 =:> t_to_center.ty
}