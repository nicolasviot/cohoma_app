use core 
use gui
use base
use display 

import ItineraryDescriptor
_define_
ItineraryPannel(double _dx, double _dy){
	Translation t (_dx, _dy)

	Int idSelected(1)

	ItineraryDescriptor first (10, 10, "selected", 1)
	ItineraryDescriptor second (10, 120, "unselected", 2)
	ItineraryDescriptor third (10, 230, "unselected", 3)

	FSM itinerary_select{
		State first_selected{

			"selected" =: first.fsm.state
			"unselected" =: second.fsm.state
			"unselected" =: third.fsm.state
			first.itinerary_id =: idSelected
			
		}
		State second_selected{
			"unselected" =: first.fsm.state
			"selected" =: second.fsm.state
			"unselected" =: third.fsm.state
			second.itinerary_id =: idSelected
			
		}
		State third_selected{
			
			"unselected" =: first.fsm.state
			"unselected" =: second.fsm.state
			"selected" =: third.fsm.state
			third.itinerary_id =:idSelected
		}
		first_selected -> second_selected (second.select, first.unselect)
		first_selected -> third_selected (third.select, first.unselect)
		second_selected -> first_selected (first.select, second.unselect)
		second_selected -> third_selected (third.select, second.unselect)
		third_selected -> first_selected (first.select, third.unselect)
		third_selected -> second_selected (second.select, third.unselect)
	}


	
}