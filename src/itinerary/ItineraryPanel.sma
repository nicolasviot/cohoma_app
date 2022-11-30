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


_define_
ItineraryPanel (Process _context, Process _model_manager)
{
	context aka _context
	//model_manager aka _model_manager

	svg_itineraries = loadFromXML ("res/svg/Itinerary_Panel.svg")
	gfx_itineraries << svg_itineraries.itinerary_panel

	Spike plan_set
	Spike start_waiting_anim
    Spike stop_waiting_anim

	// List of strips
	List strips

	int dy = 42

	// Create a strip for each model of itinerary
	for model : _model_manager.itineraries {
		addChildrenTo strips {
			ItineraryStrip strip (_context, model, dy)
		}

		dy += 52
	}

	for strip : strips {
		strip.plan_set -> plan_set
	}


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