use core
use gui
use base

import Node
import Edge
import ManagerId

_native_code_
%{
#include <iostream>
#include "core/tree/list.h"
%}

 
_action_
id_changed_action (Process src, Process itineraries)
{
	//color
	// unselected = 0x232323;
	// selected = 0x1E90FF;

	itinerary_to_reset = getRef (itineraries.ref_current_itinerary)
	if (&itinerary_to_reset != null) {
	  	for (int i = 1; i <= $itinerary_to_reset.edges.size; i++) {
			color_value = find (itinerary_to_reset, "edges/" + to_string(i) + "/color/value")
			color_value = 0x232323
      	}
   	}
   
	itinerary_to_move = find ( &itineraries, "itineraries_list/" + toString(itineraries.id))
	if (&itinerary_to_move != null) {
		for (int i = 1; i <= $itinerary_to_move.edges.size; i++) {
			color_value = find (itinerary_to_move, "edges/" + to_string(i) + "/color/value")
			color_value = 0x1E90FF
      	}
		itineraries.ref_current_itinerary = &itinerary_to_move
		// move selected itinerary above
		moveChild itinerary_to_move >>
	} 
}

_action_
edge_released_action (Process src, Process data)
{   

	//note:
	// find the id of the itinerary that has been clicked.
	// release(src)->line(edge)->component(edge)->list(edges)->component(itinerary)
	itinerary_id = find (&src, "../../../../id")
	// and assign it to current itineraries.id(data)
	data.id = itinerary_id
}

_define_
Itineraries (Process _map, Process f){
	map aka _map
	Spike create_bindings
	Spike clear
	Int id (0)
	Ref ref_current_itinerary (&id)
	ManagerId manager(0)
	Component itineraries_list {}

	NativeAction edge_released_na (edge_released_action, this, 1)
	NativeAction id_changed_na (id_changed_action, this, 1)
	id -> id_changed_na

	//debug
	// LogPrinter debug_itineraries("debug itineraries : ")
	// id =:> debug_itineraries.input
}
