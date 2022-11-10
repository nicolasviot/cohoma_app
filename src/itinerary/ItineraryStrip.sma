use core
use gui
use display
use base

import widgets.Button

_native_code_
%{
	//#include "string.h"
	#include <iostream>
%}

_define_
ItineraryStrip(Process _context, Process _model, double dy)
{
	//context aka _context
	model aka _model

	Translation t (0, dy)

	FontFamily _ ("Helvetica")
    //FontWeight _ (50)
	FontSize _ (5, 12)

    Spike select
    Spike unselect
    Spike plan_set

	Switch sw (unselected){
		Component unselected{
			Translation _ (30, 0)

			FillColor grey(128, 128, 128)
			Rectangle bg (0, 0, 390, 50, 10, 10)
			bg.press -> select

			FillColor _ (#FFFFFF)
			Text legend_label (10, 28, "...")
			_model.legend =:> legend_label.text

			Text cost_label (150, 28, "...") 
			_model.cost =:> cost_label.text
		}
		Component selected {
			Translation _ (8, 0)

			FillColor blue(53, 178, 255)
			Rectangle bg (0, 0, 390, 50, 10, 10)
			bg.press -> unselect

			FillColor _ (#FFFFFF)
			Text legend_label (10, 28, "...")
			_model.legend =:> legend_label.text

			Text cost_label (150, 28, "...") 
			_model.cost =:> cost_label.text
			
			Button set_plan (bg, "set itinerary", 299, 3)

			44 =: set_plan.r.height
			88 =: set_plan.r.width
			24 =: set_plan.thisLabel.y

			set_plan.click -> plan_set
		}
	}

	sw.unselected -> unselect
	sw.selected -> select

}