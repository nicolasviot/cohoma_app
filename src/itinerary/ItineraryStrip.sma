use core
use gui
use display
use base

import widgets.Button

_native_code_
%{
	#include <iostream>
%}


_define_
ItineraryStrip (Process _context, Process _model, double dy)
{
	//context aka _context
	model aka _model

	Translation tr (0, dy)

	FontFamily _ ("Helvetica")
    //FontWeight _ (DJN_NORMAL)
	FontSize _ (5, 12)

    Spike set_plan


	Switch switch (false) {
		Component false{
			Translation _ (30, 0)

			FillColor grey(128, 128, 128)
			Rectangle bg (0, 0, 390, 50, 10, 10)

			bg.press -> _model.select

			FillColor _ (#FFFFFF)
			Text legend_label (10, 28, "...")
			_model.legend =:> legend_label.text

			Text cost_label (150, 28, "...") 
			_model.cost =:> cost_label.text
		}
		Component true {
			Translation _ (8, 0)

			FillColor blue (53, 178, 255)
			Rectangle bg (0, 0, 390, 50, 10, 10)

			FillColor _ (#FFFFFF)
			Text legend_label (10, 28, "...")
			_model.legend =:> legend_label.text

			Text cost_label (150, 28, "...") 
			_model.cost =:> cost_label.text
			
			Button btn_set_plan (bg, "set plan", 299, 3)

			44 =: btn_set_plan.r.height
			88 =: btn_set_plan.r.width
			24 =: btn_set_plan.thisLabel.y

			btn_set_plan.click -> set_plan
		}
	}
	_model.is_selected =:> switch.state

}