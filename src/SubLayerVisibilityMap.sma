/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2023)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *	Contributors:
 *    Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use gui
use base
use display

import SubLayer
import behavior.NotDraggableItem

_native_code_
%{
	#include "cpp/coords-utils.h"
	#include <iostream>
%}


_define_
SubLayerVisibilityMap (Process _layer_model, Process _map, Process _context) inherits SubLayer (_layer_model)
{
	Double visibility_map_resolution (1)
	Double visibility_map_lat (0)
	Double visibility_map_lon (0)

	addChildrenTo this.switch.true {

		Component veil {
			FillOpacity opacity (0.5)
			_context.veil_opacity =:> opacity.a
			FillColor black (#000000)
			//Rectangle r (0, 0, $_map.width, $_map.height, 0, 0)
			Rectangle r (-500, -500, $_map.width + 1000, $_map.height + 1000, 0, 0)
			//_map.width =:> r.width
			//_map.height =:> r.height
		}

		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		Translation t_to_georef (0, 0)
		Translation t_to_center (0, 0)
		Scaling scale (1, 1, 0, 0)

		DataImage image (0, 0, 0, 0)

		// Update the position via "t_to_georef" in function of lat/lon and current zoom level
		NotDraggableItem not_draggable_item (_map, visibility_map_lat, visibility_map_lon, t_to_georef.tx, t_to_georef.ty)
			
		// scaling_factor_correction: beynes : 1.52, esperces 1.36, caylus 1.40
		visibility_map_resolution * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =:> scale.sx
		visibility_map_resolution * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =:> scale.sy

		- (image.width * visibility_map_resolution * _map.scaling_factor_correction/ get_resolution ($_map.zoomLevel)) / 2 =:> t_to_center.tx
		- (image.height * visibility_map_resolution * _map.scaling_factor_correction/ get_resolution ($_map.zoomLevel)) / 2 =:> t_to_center.ty
	}

	ui aka this.switch.true
}