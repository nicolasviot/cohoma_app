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
id_changed_action (Process c)
%{
	int unselected = 0x232323;
  	int selected = 0x1E90FF;

	std::cout << "id_changed_action" << std::endl;

 	Process *_itineraries = (Process*) get_native_user_data (c);
 	int  _id = dynamic_cast<IntProperty*>(_itineraries->find_child ("id"))->get_value ();
	Component *_itineraries_list  = dynamic_cast<Component*>(_itineraries->find_child ("itineraries_list"));
	RefProperty* _ref_itinerary = dynamic_cast<RefProperty*> (_itineraries->find_child ("ref_current_itinerary"));
	Component *_itinerary_to_reset = dynamic_cast<Component*> (_ref_itinerary->get_value ());
	int _itineraries_size = _itineraries_list->children ().size ();
	
	std::cout << "_size " << _itineraries_size << " - " << _itinerary_to_reset << std::endl;

	if ( _itineraries_size ) {

		//reset previous ref to black
  		if (_itinerary_to_reset) {
	    	List* edges = dynamic_cast<List*> (_itinerary_to_reset->find_child ("edges"));
    		int edges_size = dynamic_cast<IntProperty*> (edges->find_child("size"))->get_value ();
    		for (int i = 1; i <= edges_size; i++) {
      			((AbstractProperty*) edges->find_child( to_string(i)+"/color/value"))->set_value (unselected, true);
    		}
  		}

		// find the new one, color in blue and push LAST
    	Component* itinerary_to_move = dynamic_cast<Component*> (_itineraries_list->find_child (to_string(_id)));
  		if (itinerary_to_move) {
    		List* edges = dynamic_cast<List*> (itinerary_to_move->find_child ("edges"));
    		int edges_size = dynamic_cast<IntProperty*> (edges->find_child("size"))->get_value ();
    		for (int i = 1; i <= edges_size; i++) {
      			((AbstractProperty*) edges->find_child( to_string(i)+"/color/value"))->set_value (selected, true);
    		}
    		_itineraries_list->move_child (itinerary_to_move, LAST);
			//save the ref
			_ref_itinerary->set_value (itinerary_to_move, true);
  		}
		
	}
%}

_define_
Itineraries (Process _map, Process f, Process navgraph){
	map aka _map
	Spike create_bindings
	Spike clear
	Int id (0)
	Ref ref_current_itinerary (&id)
	ManagerId manager(0)
	Component itineraries_list {}

	NativeAction id_changed_na (id_changed_action, this, 1)
	id -> id_changed_na

	//debug
	LogPrinter debug_itineraries("debug itineraries : ")
	id =:> debug_itineraries.input
}
