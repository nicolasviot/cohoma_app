use core
use gui
use base
use display

import behavior.NotDraggableItem
import ExclusionArea
import Lima

_native_code_
%{
    #include <iostream>
	using namespace std;
%}


_define_
SiteLayer (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	Layer layer {

		Scaling sc (1, 1, 0, 0)
		_context.map_scale =:> sc.sx, sc.sy

		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		OutlineCapStyle _ (1)

		// LIMITS
		Component limits {
			NoFill _
			OutlineWidth outline_width (6)
			OutlineColor outline_color (#14BE14)
			
			Polyline poly_line

			Component behaviors

			//print ("Limits: " + _model_manager.limits.size + " points\n")

			_model_manager.limits.$added -> na_limits_added:(this) {
				print ("New model of point added to the limits: " + this.model_manager.limits.size + "\n")
			
				for (int i = 1; i <= this.model_manager.limits.size; i++) {
					//print ("View of point: lat = " + this.model_manager.limits.[i].lat + " -- lon = " + this.model_manager.limits.[i].lon + "\n")
					
					addChildrenTo this.limits.poly_line.points {
						PolyPoint _ (0, 0)
					}

					addChildrenTo this.limits.behaviors {
						NotDraggableItem _ (this.map, this.model_manager.limits.[i].lat, this.model_manager.limits.[i].lon, this.limits.poly_line.points.[i].x, this.limits.poly_line.points.[i].y)
					}
				}
			}
		}


		// EXCLUSION ZONES
		List zones

		_model_manager.zones.$added -> na_zone_added:(this) {
			print ("New model of zone added to list " + this.model_manager.zones.size + "\n")

			for model : this.model_manager.zones {
				addChildrenTo this.zones {
					ExclusionArea zone (this.map, this.context, model)
				}
			}
		}


		// LIMAS
		List limas

		_model_manager.limas.$added -> na_lima_added:(this) {
			print ("New model of Lima added to list " + this.model_manager.limas.size + "\n")
			
			for model : this.model_manager.limas {
				addChildrenTo this.limas {
					Lima lima (this.map, this.context, model)
				}
			}
		}
	}

	// DEBUG
	if (_model_manager.IS_DEBUG)
	{
		// LIMITS
		for (int i = 1; i <= _model_manager.limits.size; i++) {
			addChildrenTo layer.limits.poly_line.points {
				PolyPoint _ (0, 0)
			}

			addChildrenTo layer.limits.behaviors {
				NotDraggableItem _ (_map, _model_manager.limits.[i].lat, _model_manager.limits.[i].lon, layer.limits.poly_line.points.[i].x, layer.limits.poly_line.points.[i].y)
			}
		}

		// EXCLUSION ZONES
		for model : _model_manager.zones {
			addChildrenTo layer.zones {
				ExclusionArea zone (_map, _context, model)
			}
		}

		// LIMAS
		for model : _model_manager.limas {
			addChildrenTo layer.limas {
				Lima lima (_map, _context, model) //, null)
			}
		}
	}
}