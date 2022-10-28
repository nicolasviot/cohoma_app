use core 
use gui
use base
use display 
use animation

import gui.animation.Animator
import ItineraryDescriptor

_define_
ItineraryPannel(Process _context, Process _model_manager, Process _id_selected)
{
	//context aka _context
	//model_manager aka _model_manager

	id_selected aka _id_selected

	svg_itineraries = loadFromXML ("res/svg/Itinerary_Panel.svg")
	gfx_itineraries << svg_itineraries.itinerary_panel

	Spike plan_set
	ItineraryDescriptor first (_context, _model_manager.itineraries.shortest, "Shorter", 44)
	ItineraryDescriptor second (_context, _model_manager.itineraries.safest, "Safer", 96)
	ItineraryDescriptor third (_context, _model_manager.itineraries.tradeoff, "Mix", 148)
	
	first.plan_set -> plan_set
	second.plan_set -> plan_set
	third.plan_set -> plan_set

	(toString(id_selected) == toString(first.itinerary_id)) ? "selected" : "unselected" =:> first.sw.state
	(toString(id_selected) == toString(second.itinerary_id)) ? "selected" : "unselected" =:> second.sw.state
	(toString(id_selected) == toString(third.itinerary_id)) ? "selected" : "unselected" =:> third.sw.state

	FSM fsm_select{
		State idle
		
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


	///// WAITING FEEDBACK /////
    Spike start_waiting_anim
    Spike stop_waiting_anim

    id_selected -> stop_waiting_anim
    Animator anim_rotation (800, 0, 360, DJN_IN_OUT_SINE, 1, 0)
    20 =: anim_rotation.fps

    start_waiting_anim -> anim_rotation.start
    stop_waiting_anim -> anim_rotation.reset
    stop_waiting_anim -> anim_rotation.abort
	
    FSM fsm_waiting {
        State idle

        State animate {
			Translation _ (100, 15)
			FillColor _ (200, 200, 200)
			Text _ (30, 12, "computing itinerary")
			
			Rotation rot (0, 10, 5)
			FillColor _ (100,250,100)
			Rectangle _ (0, 0, 20, 10, 5,5)
			anim_rotation.output =:> rot.a
        }
       
        idle -> animate (start_waiting_anim)
        animate -> idle (stop_waiting_anim)
    }

}