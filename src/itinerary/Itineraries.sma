use core
use gui
use base

import ItineraryOnMap

_native_code_
%{
	#include <iostream>
	#include "core/tree/list.h"
%}

 
/*_action_
id_changed_action (Process src, Process self)
{
	//color
	// unselected = 0x232323;
	// selected = 0x1E90FF;

	itinerary_to_reset = getRef (self.ref_current_itinerary)
	if (&itinerary_to_reset != null) {
	  	for (int i = 1; i <= $itinerary_to_reset.edges.size; i++) {
			color_value = find (itinerary_to_reset, "edges/" + to_string(i) + "/outline_color/value")
			color_value = 0x232323
      	}
   	}
   
	itinerary_to_move = find ( &self, "itineraries_list/" + toString(self.context.selected_itinerary_id))
	if (&itinerary_to_move != null) {
		for (int i = 1; i <= $itinerary_to_move.edges.size; i++) {
			color_value = find (itinerary_to_move, "edges/" + to_string(i) + "/outline_color/value")
			color_value = 0x1E90FF
      	}
		self.ref_current_itinerary = &itinerary_to_move

		// move selected itinerary above
		moveChild itinerary_to_move >>
	} 
}*/

/*_action_
edge_released_action (Process src, Process self)
{   
	print ("edge_released_action\n")
	
	// find the id of the itinerary that has been clicked.
	// release(src)->line(edge)->component(edge)->list(edges)->component(itinerary)
	itinerary_id = find (&src, "../../../../id")

	// and assign it to "selected_itinerary_id" in the context
	self.context.selected_itinerary_id = toString (itinerary_id)
}*/


_define_
Itineraries (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	//model_manager aka _model_manager

	Ref ref_current_itinerary (nullptr)

	//Spike clear

	Scaling sc (1, 1, 0, 0)
	_context.map_scale =:> sc.sx, sc.sy

	Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	// Parent for itineraries
	//Component itineraries_list

	List itineraries_on_map

	// Create a view for each model of itinerary
	for model : _model_manager.itineraries {
		addChildrenTo itineraries_on_map {
			ItineraryOnMap itinerary (_context, _model_manager, model)
		}
	}

	//NativeAction edge_released_na (edge_released_action, this, 1)

	//NativeAction id_changed_na (id_changed_action, this, 1)
	//_context.selected_itinerary_id -> id_changed_na

}
