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

	Double resolution (5)

	TaskAreaSummit georef_visibility_map (map, 0, 0)
	Translation t_to_georef (0, 0)
	Translation t_to_center (0, 0)
	Scaling  scale (1, 1, 0, 0)
	DataImage visibility_map (0,0,0,0)

	georef_visibility_map.x => t_to_georef.tx
	georef_visibility_map.y => t_to_georef.ty

	resolution * 1 / get_resolution ($map.zoomLevel) =:> scale.sx
	resolution * 1 / get_resolution ($map.zoomLevel) =:> scale.sy

	- (visibility_map.width * resolution / get_resolution ($map.zoomLevel)) / 2 =:> t_to_center.tx
	- (visibility_map.height * resolution / get_resolution ($map.zoomLevel)) / 2 =:> t_to_center.ty
}