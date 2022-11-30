use core
use gui
use base

import ItineraryOnMap

_native_code_
%{
	#include <iostream>
%}


_define_
Itineraries (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	//model_manager aka _model_manager

	Scaling sc (1, 1, 0, 0)
	_context.map_scale =:> sc.sx, sc.sy

	Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	// Itineraries on map
	Component itineraries_on_map


	// Create a view for each model of itinerary
	for model : _model_manager.itineraries {
		addChildrenTo itineraries_on_map {
			ItineraryOnMap itinerary (_context, _model_manager, model)

			model.is_selected.true -> (itinerary) {
				moveChild itinerary >>
			}
		}
	}

}
