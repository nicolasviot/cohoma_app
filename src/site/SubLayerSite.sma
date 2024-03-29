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
import ExclusionArea
import Lima

_native_code_
%{
    #include <iostream>
	using namespace std;
%}


_action_
action_limits_added (Process src, Process self)
{
	print ("New model of point added to the limits: " + self.model_manager.limits.size + "\n")

	for (int i = 1; i <= self.model_manager.limits.size; i++) {
		//print ("View of point: lat = " + self.model_manager.limits.[i].lat + " -- lon = " + self.model_manager.limits.[i].lon + "\n")
		
		addChildrenTo self.ui.limits.poly_line_out.points {
			PolyPoint _ (0, 0)
		}
		addChildrenTo self.ui.limits.poly_line_in.points {
			PolyPoint _ (0, 0)
		}

		addChildrenTo self.ui.limits.behaviors {
			NotDraggableItem _ (self.map, self.model_manager.limits.[i].lat, self.model_manager.limits.[i].lon, self.ui.limits.poly_line_out.points.[i].x, self.ui.limits.poly_line_out.points.[i].y)
			NotDraggableItem _ (self.map, self.model_manager.limits.[i].lat, self.model_manager.limits.[i].lon, self.ui.limits.poly_line_in.points.[i].x, self.ui.limits.poly_line_in.points.[i].y)
		}
	}
}


_action_
action_zones_added (Process src, Process self)
{
	print ("New model of zone added to list " + self.model_manager.zones.size + "\n")

	for model : self.model_manager.zones {
		addChildrenTo self.ui.zones {
			ExclusionArea zone (self.map, self.context, model)
		}
	}
}


_action_
action_limas_added (Process src, Process self)
{
	print ("New model of Lima added to list " + self.model_manager.limas.size + "\n")
	
	for model : self.model_manager.limas {
		addChildrenTo self.ui.limas {
			Lima lima (self.map, self.context, model)
		}
	}
}


// SITE = Limits + Exclusion zones + Limas
_define_
SubLayerSite (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	NativeAction na_limits_added (action_limits_added, this, 1)
	_model_manager.limits.$added -> na_limits_added
	
	NativeAction na_zones_added (action_zones_added, this, 1)
	_model_manager.zones.$added -> na_zones_added

	NativeAction na_limas_added (action_limas_added, this, 1)
	_model_manager.limas.$added -> na_limas_added


	addChildrenTo this.switch.true {
		Component layer {

			Translation pos (0, 0)
			_context.map_translation_x =:> pos.tx
			_context.map_translation_y =:> pos.ty

			OutlineCapStyle _ (1)

			// LIMITS
			Component limits {
				NoFill _
				
				OutlineWidth outline_width (6)
				OutlineColor outline_color (#000000)
				
				Polyline poly_line_out

				OutlineWidth outline_width_in (2)
				OutlineColor outline_color_in (#BBBBBB)

				Polyline poly_line_in

				Component behaviors
			}

			// EXCLUSION ZONES
			List zones

			// LIMAS
			List limas
		}
	}

	ui aka this.switch.true.layer


	//____________________
	//
	//  DEBUG
	//
	if (_model_manager.IS_DEBUG)
	{
		// LIMITS
		for (int i = 1; i <= _model_manager.limits.size; i++) {
			addChildrenTo ui.limits.poly_line_out.points {
				PolyPoint _ (0, 0)
			}
			addChildrenTo ui.limits.poly_line_in.points {
				PolyPoint _ (0, 0)
			}

			addChildrenTo ui.limits.behaviors {
				NotDraggableItem _ (_map, _model_manager.limits.[i].lat, _model_manager.limits.[i].lon, ui.limits.poly_line_out.points.[i].x, ui.limits.poly_line_out.points.[i].y)
				NotDraggableItem _ (_map, _model_manager.limits.[i].lat, _model_manager.limits.[i].lon, ui.limits.poly_line_in.points.[i].x, ui.limits.poly_line_in.points.[i].y)
			}
		}

		// EXCLUSION ZONES
		for model : _model_manager.zones {
			addChildrenTo ui.zones {
				ExclusionArea zone (_map, _context, model)
			}
		}

		// LIMAS
		for model : _model_manager.limas {
			addChildrenTo ui.limas {
				Lima lima (_map, _context, model)
			}
		}
	}
}