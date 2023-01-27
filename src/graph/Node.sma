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
    
    Int default_radius (5)
	Int inside_radius (12)
	Int mask_radius (16)

    Translation screen_translation (0, 0)
	_model.dx_in_map =:> screen_translation.tx
	_model.dy_in_map =:> screen_translation.ty
    
  
	Component bg {
		//FillOpacity _ (0)
		//OutlineOpacity _(0)
		NoFill _
		PickFill _
    	Circle mask (0, 0, $mask_radius)

    	mask.left.press -> {
			_model =: _context.ref_node_graph_edition
    	}
		mask.right.press -> {
    	    _model =: _context.ref_node_status_edition
    	}
	}


	FSM fsm {
        State idle

        State entered {
            Timer t (500)
        }

        State display_details

        idle -> entered (bg.mask.enter)
        entered -> display_details (entered.t.end)
        entered -> idle (bg.mask.leave)
        display_details -> idle (bg.mask.leave)
    }


	Component label {
		Int default_label_height (20)
		FillOpacity global_op (0.7)

		FillColor light_grey (#D3D3D3)
		Rectangle r_link ($default_radius, -2, 15 - $default_radius, 4, 0, 0)

		OutlineOpacity outline_op (0.7)
		OutlineWidth _ (1)
		OutlineColor _ (#555555)

		Rectangle r_bg (15, -10, 20, $default_label_height, 5, 5)
		
		FillColor texts_color (#000000)

		Component details {
			FillOpacity details_op (0.0)

			FontWeight _ (DJN_NORMAL)
			FontSize _ (5, 14)
			Text txt_details (18, 5, "")
			"(" + _model.id + ") " + _model.status =:> txt_details.text
		}

		Switch switch_is_empty_name (true) {
			// NOT empty name
			Component false {
				FontWeight _ (DJN_BOLD)
				FontSize _ (5, 14)
				Text txt_name (18, 5, toString(_model.label))
				//_model.label =:> txt_name.text

				24 =: details.txt_details.y

				Switch switch_not_empty_label (idle) {
					Component idle {
						txt_name.width + 6 =: r_bg.width
						0.7 =: global_op.a, outline_op.a
						0.0 =: details.details_op.a
						default_label_height =: r_bg.height
					}
					Component entered {
						
					}
					Component display_details {
						txt_name.width > details.txt_details.width ? txt_name.width + 6 : details.txt_details.width + 6 =: r_bg.width
						0.9 =: global_op.a, outline_op.a, details.details_op.a
						40 =: r_bg.height
					}
				}
				fsm.state =:> switch_not_empty_label.state
			}
			// Empty name
			Component true {
				details.txt_details.width + 6 =:> r_bg.width

				Switch switch_empty_name (idle) {
					Component idle {
						0.0 =: global_op.a, outline_op.a, details.details_op.a
					}
					Component entered {
						
					}
					Component display_details {
						0.9 =: global_op.a, outline_op.a, details.details_op.a
					}
				}
				fsm.state =:> switch_empty_name.state
			}
		}
		_model.is_empty_label =: switch_is_empty_name.state
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

				2 =: outline_width.width
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

			Component forced {
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
				Circle outer_circle (0, 0, $inside_radius)
			}
		}
		_model.status =:> status_switch.state
	}
    
	Switch switch (idle) {
		Component idle {
			default_radius =: ui.c.r
		}
		Component entered {
			inside_radius =: ui.c.r
		}
		Component display_details {
			//inside_radius =: c.r
		}
	}
	fsm.state =:> switch.state
    

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItem draggable_item (_map, _context, _model.lat, _model.lon, _model.dx_in_map, _model.dy_in_map, bg.mask, _context.frame_released)

}