use core 
use gui
use base
use display 
use animation

import gui.animation.Animator



import ItineraryDescriptor
_define_
ItineraryPannel(double _dx, double _dy, Process _id_selected){
	Translation t (_dx, _dy)

	id_selected aka _id_selected

	ite_svg = loadFromXML ("res/svg/Itinerary_Panel.svg")
	ite_gfx << ite_svg.itinerary_panel

	Spike plan_set
	ItineraryDescriptor first (0, 44, "selected", "Shorter")
	ItineraryDescriptor second (0, 96, "unselected", "Safer")
	ItineraryDescriptor third (0, 148, "unselected", "Mix")
	
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


	//waiting feedback

	//HIGHLIGHT ANIMATION ON REQUEST /////
    Spike startWaitingAnim
    Spike stopWaitingAnim

    Animator radius_anim (800, 0, 360, DJN_IN_OUT_SINE, 1, 0)

    20 =: radius_anim.fps
    startWaitingAnim -> radius_anim.start
    stopWaitingAnim -> radius_anim.reset
    stopWaitingAnim -> radius_anim.abort

	
    FSM locate_FSM{
        State idle{

        }
        State animate{
			Translation _ (100,15)
			FillColor _ (200,200,200)
			Text _ (30, 12, "computing itinerary")
			Rotation rot (0, 10, 5)
			FillColor _ (100,250,100)
			Rectangle _ (0, 0, 20, 10, 5,5)
			radius_anim.output =:> rot.a
        }
       
        idle -> animate (startWaitingAnim)
        animate -> idle (stopWaitingAnim)
    }

	//debug
	// LogPrinter lp ("state: ")
	// fsm_select.state =:> lp.input	
}