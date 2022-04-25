use core 
use gui
use base
use display 




import ItineraryDescriptor
_define_
ItineraryPannel(double _dx, double _dy, Process _id_selected){
	Translation t (_dx, _dy)

	id_selected aka _id_selected

	Spike plan_set
	ItineraryDescriptor first (10, 10, "selected", "toto")
	ItineraryDescriptor second (10, 120, "unselected", "titi")
	ItineraryDescriptor third (10, 230, "unselected", "tata")
	
	first.plan_set -> plan_set
	second.plan_set -> plan_set
	third.plan_set -> plan_set

	(toString(id_selected) == toString(first.itinerary_id)) ? "selected" : "unselected" =:> first.sw.state
	(toString(id_selected) == toString(second.itinerary_id)) ? "selected" : "unselected" =:> second.sw.state
	(toString(id_selected) == toString(third.itinerary_id)) ? "selected" : "unselected" =:> third.sw.state

	FSM fsm_select{
		State idle {}
		State first_selected{
			first.itinerary_id =?: id_selected
		}
		State second_selected{
			second.itinerary_id =?: id_selected
		}
		State third_selected{
			third.itinerary_id =?: id_selected
		}
		idle -> first_selected (first.select)
		idle -> second_selected (second.select)
		idle -> third_selected (third.select)
		first_selected -> second_selected (second.select)
		first_selected -> third_selected (third.select)
		second_selected -> first_selected (first.select)
		second_selected -> third_selected (third.select)
		third_selected -> first_selected (first.select)
		third_selected -> second_selected (second.select)
	}

	//debug
	// LogPrinter lp ("state: ")
	// fsm_select.state =:> lp.input	
}