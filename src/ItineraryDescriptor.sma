use core
use gui
use display
use base
import Button
_native_code_
%{
#include "string.h"
%}
_define_
ItineraryDescriptor(double dx, double dy){

	Translation t (dx, dy)

	String description("This is the most amazing itinerary ever")
	String legend ("Itinerary n°")
	Int itinerary_id(0)


	FontFamily _ ("B612")
    FontWeight _ (75)
    Spike select
    Spike unselect

	FSM fsm{
			State selected{
			FillColor blue(0, 191, 255)
			Rectangle bg(0, 0, 500, 100, 10, 10 )

    		FontSize _ (5, 12)
    		FillColor _ (255, 255, 255)
			Text legend_label(10, 20, "toto")
			legend + toString (itinerary_id) =:> legend_label.text
			FontSize _ (5, 8)
			Text description_label (10, 50, "toto") 
			description =:> description_label.text
			
			Button set_plan (bg, "set itinerary", 300, 25)

		}
		State unselected{
			Translation _ (40, 0)
			FillColor grey(128, 128, 128)
			Rectangle bg(0, 0, 500, 100, 10, 10 )

    		FontSize _ (5, 12)
    		FillColor _ (255, 255, 255)
			Text legend_label(10, 20, "toto")
			legend + toString (itinerary_id) =:> legend_label.text
			FontSize _ (5, 8)
			Text description_label (10, 50, "toto") 
			description =:> description_label.text
			Button set_plan (bg, "set itinerary", 300, 25)
		}

	
		unselected -> selected (select)
		selected -> unselected (unselect)
	}

	fsm.unselected.bg.press -> select
	fsm.selected.bg.press -> unselect


}