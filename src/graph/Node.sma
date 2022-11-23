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

    Spike disable_drag
    Spike renable_drag
    Spike leave
    Spike enter

    Int default_radius (10)
	Int inside_radius (20)
	Int mask_radius (30)

    Translation screen_translation (0, 0)
	screen_translation.tx =:> _model.dx_map
	screen_translation.ty =:> _model.dy_map
    
  
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
    
    FillOpacity _ (1.2)
    FillColor _ (0, 0, 0)
    FontWeight _ (75)
    FontSize _ (5, 20)
    Text txt_label (-$c.r / 2, -20, "")
    _model.label =:> txt_label.text
    

    //LogPrinter lp ("debug enter/leave (Graph Node) ")

    FSM tooltip {
        State idle
        
        State entered {
            Timer t (500)
        }

        State display_tooltip {
			OutlineWidth _ (1)
            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            FontSize _ (5, 12)
            FontWeight _ (50)
            Rectangle bg (20, -10, 50, 20, 5, 5)

            FillColor black (#000000)
			Text legend (23, 4, "Node 0")
            legend.width + 6 =:> bg.width
            //legend.height + 2 =:> bg.height
			
            //"Node " + toString(id_in_tooltip) + " (" + _model.label + ") " + _model.status =:> legend.text
			"Node " + toString(_model.id) + " (" + _model.label + ") " + _model.status =:> legend.text
        }

        idle -> entered (enter)
        entered -> display_tooltip (entered.t.end)
        entered -> idle (leave)
        display_tooltip -> idle (leave)
    }

    FillOpacity _ (0)
    OutlineOpacity _(0)
    Circle interact_mask (0, 0, $mask_radius)

    interact_mask.enter -> enter
    interact_mask.leave -> leave

    interact_mask.right.press -> {
        this =: _context.ref_current_node
    }

    interact_mask.left.press -> {
        _model.id =: _context.selected_node_id
    }
    

    FSM fsm_mask {
        State idle {
            default_radius =: c.r
        }
        State inside {
            inside_radius =: c.r
        }
        idle -> inside (interact_mask.enter)
        inside -> idle (interact_mask.leave)
    }

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItem draggable_item (_map, _context, _model.lat, _model.lon, screen_translation.tx, screen_translation.ty, interact_mask)

}