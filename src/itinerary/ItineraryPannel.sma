use core 
use gui
use base
use display 
use animation

import gui.animation.Animator
import ItineraryStrip

_native_code_
%{
    #include <iostream>
%}


/*_action_
action_select_strip (Process src, Process self)
{
	strip = find(&src, "..")

	print ("action_select_strip " + strip.model.type + " (" + strip.model.uid + ")\n")
	
	self.context.selected_itinerary_id = toString(strip.model.uid)
}*/


_define_
ItineraryPannel (Process _context, Process _model_manager)
{
	context aka _context
	//model_manager aka _model_manager

	svg_itineraries = loadFromXML ("res/svg/Itinerary_Panel.svg")
	gfx_itineraries << svg_itineraries.itinerary_panel

	Spike plan_set
	Spike start_waiting_anim
    Spike stop_waiting_anim

	List strips

	int dy = 42

	// Create a strip for each model of itinerary
	for model : _model_manager.itineraries {
		addChildrenTo strips {
			ItineraryStrip strip (_context, model, dy)
		}

		dy += 52
	}

	//NativeAction na_select_strip (action_select_strip, this, 1)
	//na_select_strip -> stop_waiting_anim

	for strip : strips {
		//strip.select -> na_select_strip
		strip.select -> {
			toString(strip.model.uid) =: _context.selected_itinerary_id
		}
		strip.select -> stop_waiting_anim

		strip.plan_set -> plan_set
	}

	
	/*ItineraryStrip first (_context, _model_manager.itineraries.shortest, 44)
	ItineraryStrip second (_context, _model_manager.itineraries.safest, 96)
	ItineraryStrip third (_context, _model_manager.itineraries.tradeoff, 148)
	
	first.plan_set -> plan_set
	second.plan_set -> plan_set
	third.plan_set -> plan_set

	(toString(id_selected) == toString(first.model.uid)) ? "selected" : "unselected" =:> first.sw.state
	(toString(id_selected) == toString(second.model.uid)) ? "selected" : "unselected" =:> second.sw.state
	(toString(id_selected) == toString(third.model.uid)) ? "selected" : "unselected" =:> third.sw.state

	FSM fsm_select{
		State idle
		
		State first_selected {
			first.model.uid =?: id_selected
		}
		State second_selected {
			second.model.uid =?: id_selected
		}
		State third_selected {
			third.model.uid =?: id_selected
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
	}*/


	///// WAITING FEEDBACK /////

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