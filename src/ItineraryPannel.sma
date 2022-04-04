use core 
use gui
use base
use display 

import ItineraryDescriptor
_define_
ItineraryPannel(double _dx, double _dy){
	Translation t (_dx, _dy)

	Int idSelected(1)


	FSM itinerary_select{
		State first_selected{
			ItineraryDescriptor first (10, 10, "selected")
			ItineraryDescriptor second (10, 120, "unselected")
			ItineraryDescriptor third (10, 230, "unselected")
			1 =: idSelected	
			
		}
		State second_selected{
			ItineraryDescriptor first (10, 10, "unselected")
			ItineraryDescriptor second (10, 120, "selected")
			ItineraryDescriptor third (10, 230, "unselected")
			2 =: idSelected
			
		}
		State third_selected{
			ItineraryDescriptor first (10, 10, "unselected")
			ItineraryDescriptor second (10, 120, "unselected")
			ItineraryDescriptor third (10, 230, "selected")	
			3 =: idSelected
			
		}
		first_selected -> second_selected (first_selected.second.select, first_selected.first.unselect)
		first_selected -> third_selected (first_selected.third.select, first_selected.first.unselect)
		second_selected -> first_selected (second_selected.first.select, second_selected.second.unselect)
		second_selected -> third_selected (second_selected.third.select, second_selected.second.unselect)
		third_selected -> first_selected (third_selected.first.select, third_selected.third.unselect)
		third_selected -> second_selected (third_selected.second.select, third_selected.third.unselect)
	}


	
}