use core
use gui
use display
use base

import widgets.Button

_native_code_
%{
#include "string.h"
#include <iostream>
%}

_define_
ItineraryDescriptor(double dx, double dy, string start_state, string id){

	Translation t (dx, dy)

	// ex msg : "b39409be39-39090 Planning shortest|safest|tradeoff OK a path including PPO ... with cost 286.458"
	String description_input ("")
	Regex regex (".*Planning (\\S*) .* cost (\\S*)")
	String cost ("..")
	String legend ("Itinerary: ")
	String itinerary_id(id)
	description_input =:> regex.input
	LogPrinter lp ("debug regexp")
	description_input =:> lp.input
	regex.[1] =:> legend
  	regex.[2] =:> cost

	FontFamily _ ("Helvetica")
    //FontWeight _ (50)
    Spike select
    Spike unselect
    Spike plan_set

	Switch sw (unselected){
		Component unselected{
			Translation _ (30, 0)
			FillColor grey(128, 128, 128)
			Rectangle bg(0, 0, 390, 50, 10, 10)

			FontSize _ (5, 12)
			FillColor _ (255, 255, 255)
			Text legend_label (10, 28, "...")
			legend =:> legend_label.text
			Text cost_label (150, 28, "...") 
			cost =:> cost_label.text
		}
		Component selected {
			Translation _ (8, 0)
			FillColor blue(53, 178, 255)
			Rectangle bg(0, 0, 390, 50, 10, 10 )

    		FontSize _ (5, 12)
			FillColor _ (255, 255, 255)
			Text legend_label (10, 28, "...")
			legend =:> legend_label.text
			Text cost_label (150, 28, "...") 
			cost =:> cost_label.text
			
			Button set_plan (bg, "set itinerary", 299, 3)

			44 =: set_plan.r.height
			88 =: set_plan.r.width
			24 =: set_plan.thisLabel.y
			// set_plan.r.height = 44
			// set_plan.thisLabel.y = 28
			set_plan.click -> plan_set
		}
	}

	sw.unselected -> unselect
	sw.selected -> select

	sw.unselected.bg.press -> select
	sw.selected.bg.press -> unselect

	
}