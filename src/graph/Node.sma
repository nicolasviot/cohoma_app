use core
use gui
use base

import behavior.DraggableItem

_native_code_
%{
	#include <iostream>
    //#include "cpp/coords-utils.h"
%}


_define_
Node (Process _map, Process _context, Process _model)
{
	 //context aka _context
	model aka _model
    
    //Int id_in_tooltip (_model.id - 1)
    //_model.id - 1 => id_in_tooltip

    Int default_radius (10)
	Int inside_radius (20)
	Int mask_radius (30)

    Translation screen_translation (0, 0)
	_model.dx_in_map =:> screen_translation.tx
	_model.dy_in_map =:> screen_translation.ty
    
  
	Component bg {
		//FillOpacity _ (0)
		//OutlineOpacity _(0)
		NoFill _
		PickFill _
    	Circle mask (0, 0, $mask_radius)

		mask.right.press -> {
    	    this =: _context.ref_current_node
    	}

    	mask.left.press -> {
        	_model.id =: _context.selected_node_id
    	}
	}

	Component ui {
		// Graphical variables to be updated in different status
		FillColor fill_color (0)
		FillOpacity fill_opacity (0.6)
		OutlineColor outline_color (0)
		OutlineWidth outline_width (1)
		OutlineOpacity outline_opacity(0.5)

		Circle c (0, 0, $default_radius)

		Switch status_switch (default) {
			Component default {
				_context.NODE_COL =: fill_color.value
				_context.WHITE_COL =: outline_color.value

				3 =: outline_width.width
			}

			Component start {
				_context.ACTIVE_COL =: fill_color.value
				_context.START_COL =: outline_color.value
				
				2 =: outline_width.width
			}

			Component end {
				_context.ACTIVE_COL =: fill_color.value
				_context.MANDATORY_COL =: outline_color.value

				2 =: outline_width.width
			}

			Component forced { // equal to locked ?
				_context.ACTIVE_COL =: fill_color.value
				_context.BLACK_COL =: outline_color.value

				2 =: outline_width.width
			}

			Component mandatory {
				_context.ACTIVE_COL =: fill_color.value
				_context.MANDATORY_COL =: outline_color.value

				1 =: outline_width.width

				NoFill _ 
				OutlineWidth _ (2)
				OutlineOpacity _ (1.5)
				Circle outer_circle (0, 0, 15)
			}
		}
		_model.status =:> status_switch.state
	}
    
    //FillOpacity _ (1.2)
    FillColor _ (0, 0, 0)
    FontWeight _ (75)
    FontSize _ (5, 20)
    Text txt_label (-$ui.c.r / 2, -20, "")
    _model.label =:> txt_label.text
    

    FSM fsm {
        State idle {
			default_radius =: ui.c.r
		}
        
        State entered {
            Timer t (500)
			inside_radius =: ui.c.r
        }

        State display_tooltip {
			//inside_radius =: c.r

			OutlineWidth _ (1)
			OutlineColor _ (#777777)
            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            Rectangle bg (20, -10, 50, 20, 5, 5)

            FillColor black (#000000)
			FontSize _ (5, 12)
            FontWeight _ (DJN_NORMAL)
			Text legend (23, 4, "Node 0")
            legend.width + 6 =:> bg.width
            //legend.height + 2 =:> bg.height
			
            //"Node " + toString(id_in_tooltip) + " (" + _model.label + ") " + _model.status =:> legend.text
			"Node " + toString(_model.id) + " (" + _model.label + ") " + _model.status =:> legend.text
        }

        idle -> entered (bg.mask.enter)
        entered -> display_tooltip (entered.t.end)
        entered -> idle (bg.mask.leave)
        display_tooltip -> idle (bg.mask.leave)
    }
    

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItem draggable_item (_map, _context, _model.lat, _model.lon, _model.dx_in_map, _model.dy_in_map, bg.mask)

}