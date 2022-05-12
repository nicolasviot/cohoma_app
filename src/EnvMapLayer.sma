use core
use gui
use base
use display

import TaskAreaSummit


_define_
EnvMapLayer (Process map){
	TaskAreaSummit georef_visibility_map (map, 0, 0)
	DataImage visibility_map (0,0,0,0)

	georef_visibility_map.x => visibility_map.x 
	georef_visibility_map.y => visibility_map.y
}