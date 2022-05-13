use core
use gui
use base
use display

import TaskAreaSummit

_native_code_
%{
#include "cpp/coords-utils.h"
#include <iostream>
/*unsigned long RGBToHex(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}*/
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

	visibility_map_resolution * 1 / get_resolution ($map.zoomLevel) =:> scale.sx
	visibility_map_resolution * 1 / get_resolution ($map.zoomLevel) =:> scale.sy

	- (visibility_map.width * visibility_map_resolution / get_resolution ($map.zoomLevel)) / 2 =:> t_to_center.tx
	- (visibility_map.height * visibility_map_resolution / get_resolution ($map.zoomLevel)) / 2 =:> t_to_center.ty
}