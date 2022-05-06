use core
use gui
use display
use base
import Button
_native_code_
%{
#include "string.h"
#include <iostream>
%}
_define_
ItineraryDescriptor(double dx, double dy, string start_state, string id){

	Translation t (dx, dy)

	String description("..")
	String legend ("Itinerary: ")
	String itinerary_id(id)

	FontFamily _ ("B612")
    FontWeight _ (75)
    Spike select
    Spike unselect
    Spike plan_set

	Switch sw (unselected){
		Component unselected{
			Translation _ (20, 0)
			FillColor grey(128, 128, 128)
			Rectangle bg(0, 0, 300, 50, 10, 10)

    		FontSize _ (5, 12)
    		FillColor _ (255, 255, 255)
			Text legend_label(10, 20, "...")
			legend + toString (itinerary_id) =:> legend_label.text
			FontSize _ (5, 8)
			Text description_label (10, 50, "...") 
			description =:> description_label.text
		}
		Component selected {
			FillColor blue(0, 191, 255)
			Rectangle bg(0, 0, 300, 50, 10, 10 )

    		FontSize _ (5, 12)
    		FillColor _ (255, 255, 255)
			Text legend_label(10, 20, "...")
			legend + toString (itinerary_id) =:> legend_label.text
			FontSize _ (5, 8)
			Text description_label (10, 50, "...") 
			description =:> description_label.text
			
			Button set_plan (bg, "set itinerary", 200, 25)
			set_plan.click -> plan_set
		}
	}

	sw.unselected -> unselect
	sw.selected -> select

	sw.unselected.bg.press -> select
	sw.selected.bg.press -> unselect

	
}