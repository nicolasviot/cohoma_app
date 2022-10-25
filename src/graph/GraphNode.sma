use core
use gui
use base
use animation

import gui.animation.Animator
import behavior.DraggableItem

_native_code_
%{
    #include "cpp/coords-utils.h"

%}

_define_
GraphNode (Process map, Process _context, int _id, double _lat, double _lon)
{
    //context aka _context

    Int id (_id)
    Double lat (_lat)
    Double lon (_lon)

    Bool islocked(0)
    Bool isMandatory(0)

    String label("")
    Double altitude_msl(0)

    String usage_status ("default")
    
    Int id_in_tooltip (_id - 1)
    id - 1 => id_in_tooltip

    Spike disable_drag
    Spike renable_drag
    Spike leave
    //Spike left_press
    Spike right_press
    Spike enter

    Int default_radius (10)
    Int other_radius (10)

    Translation screen_translation(0, 0)
    
  
    // Graphical variables to be updated in different status
    FillOpacity fill_opacity (0.6)
    opacity aka fill_opacity.a


    FillColor fill_color (0)
    OutlineColor outline_color (0)
    OutlineWidth outline_width (1)
    OutlineOpacity outline_opacity(0.5)
    Circle c (0, 0, 8)

    Switch status_switch (default) {
        Component default {
            _context.NODE_COL =: fill_color.value
            _context.WHITE_COL =: outline_color.value

            3 =: outline_width.width
            default_radius =: c.r
            0 =: isMandatory
        }
        Component start {
            _context.ACTIVE_COL =: fill_color.value
            _context.START_COL =: outline_color.value
            
            2 =: outline_width.width
            other_radius =: c.r
        }
        Component end {
            _context.ACTIVE_COL =: fill_color.value
            _context.MANDATORY_COL =: outline_color.value

            other_radius =: c.r
            2 =: outline_width.width
        }
        Component forced {
            _context.ACTIVE_COL =: fill_color.value
            _context.BLACK_COL =: outline_color.value

            other_radius =: c.r
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
            1 =: isMandatory
        }
    }
    usage_status => status_switch.state
    
    FillOpacity _ (1.2)
    FillColor _ (0, 0, 0)
    FontWeight _ (75)
    FontSize _ (5, 20)
    Text label_text(-$c.r/2, -20, "")
    label =:> label_text.text
    

    //LogPrinter lp ("debug enter/leave (Graph Node) ")

    FSM tooltip{
        State idle
        
        State entered{
            Timer t (500)
            //"enter in Graph Node" =: lp.input
        }

        State display_tooltip{
            Translation t (20, 0)

            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            FontSize _ (5, 12)
            FontWeight _ (50)
            Rectangle bg (0, 0, 50, 20, 5, 5)
            FillColor _ (0, 0, 0)
            Text legend (0, 0, "Node 0")
            legend.x =:> bg.x
            legend.y - 12 =:> bg.y
            legend.width =:> bg.width
            legend.height =:> bg.height
            "Node " + toString(id_in_tooltip) + " (" + label + ") " + usage_status =:> legend.text
        }

        idle -> entered (enter)
        entered -> display_tooltip (entered.t.end)
        entered -> idle (leave)
        display_tooltip -> idle (leave)
    }

    FillOpacity _ (0)
    OutlineOpacity _(0)
    Circle interact_mask (0, 0, 30)

    interact_mask.enter -> enter
    interact_mask.leave -> leave
    //interact_mask.left.press -> left_press
    interact_mask.right.press -> right_press

    interact_mask.enter -> {
		this =: _context.entered_wpt
	}
    interact_mask.press -> {
        id =: _context.selected_node_id
    }
    

    FSM fsm_mask {
        State idle {
            10 =: c.r
        }
        State inside {
            20 =: c.r
        }
        idle -> inside (interact_mask.enter)
        inside -> idle (interact_mask.leave)
    }

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItem draggable_item (map, _context, lat, lon, screen_translation.tx, screen_translation.ty, interact_mask)

}