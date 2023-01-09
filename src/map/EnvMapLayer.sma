use core
use gui
use base
use display

import behavior.NotDraggableItem

_native_code_
%{
	#include "cpp/coords-utils.h"
	#include <iostream>
%}

// Visibility Map Layer ?
_define_
EnvMapLayer (Process _map, Process _context)
{
	Double visibility_map_resolution (1)
	Double visibility_map_lat (0)
	Double visibility_map_lon (0)

	Scaling sc (1, 1, 0, 0)
	_context.map_scale =:> sc.sx, sc.sy

	Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	Translation t_to_georef (0, 0)
	Translation t_to_center (0, 0)
	Scaling scale (1, 1, 0, 0)

	DataImage visibility_map (0, 0, 0, 0)

	// Update the position via "t_to_georef" in function of lat/lon and current zoom level
	NotDraggableItem not_draggable_item (_map, visibility_map_lat, visibility_map_lon, t_to_georef.tx, t_to_georef.ty)
        
	// scaling_factor_correction: beynes : 1.52, esperces 1.36, caylus 1.40
	visibility_map_resolution * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =:> scale.sx
	visibility_map_resolution * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =:> scale.sy

	- (visibility_map.width * visibility_map_resolution * _map.scaling_factor_correction/ get_resolution ($_map.zoomLevel)) / 2 =:> t_to_center.tx
	- (visibility_map.height * visibility_map_resolution * _map.scaling_factor_correction/ get_resolution ($_map.zoomLevel)) / 2 =:> t_to_center.ty
}