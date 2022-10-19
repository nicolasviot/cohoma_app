use core
use gui
use base
use animation

import gui.animation.Animator

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
    Int default_radius (10)
    Int other_radius (10)
    String label("")
    Double altitude_msl(0)
    Double battery_voltage(0)
    Double heading_rot(0)
    String usage_status ("default")
    
    Int id_in_tooltip (_id - 1)
    id - 1 => id_in_tooltip

    Spike disable_drag
    Spike renable_drag
    Spike leave
    //Spike left_press
    Spike right_press
    Spike enter


    Translation screen_translation(0, 0)

    Rotation rot (0, 0, 0)
    screen_translation.tx =:> rot.cx
    screen_translation.ty =:> rot.cy
    heading_rot =:> rot.a


    
  
    //graphical variables to be updated in different status
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
    label =:>label_text.text
    

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
    

    c.cx =:> interact_mask.cx
    c.cy =:> interact_mask.cy

    FSM enterLeave {
        State idle {
            10 =: c.r
        }
        State inside {
            20 =: c.r
        }
        idle -> inside (interact_mask.enter)
        inside -> idle (interact_mask.leave)
    }

      FSM drag_fsm {
            State no_drag {
                map.t0_y - lat2py ($lat, $map.zoomLevel) =:> screen_translation.ty
                (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> screen_translation.tx
            }
            State no_drag_while_drawing_edge{
                map.t0_y - lat2py ($lat, $map.zoomLevel) =:> screen_translation.ty
                (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> screen_translation.tx
            }
            State drag {
                Double init_cx (0)
                Double init_cy (0)
                Double offset_x (0)
                Double offset_y (0)
                screen_translation.tx =: init_cx
                screen_translation.ty =: init_cy
                interact_mask.press.x - screen_translation.tx =: offset_x
                interact_mask.press.y - screen_translation.ty =: offset_y
                interact_mask.move.x - offset_x => screen_translation.tx
                interact_mask.move.y - offset_y => screen_translation.ty
                px2lon ($screen_translation.tx + map.t0_x, $map.zoomLevel) => lon, map.reticule.pointer_lon2
                py2lat (map.t0_y - $screen_translation.ty, $map.zoomLevel) => lat, map.reticule.pointer_lat2
            }
            no_drag -> drag (interact_mask.left.press, map.reticule.show_reticule2)
            no_drag -> no_drag_while_drawing_edge (_context.shift)
            no_drag_while_drawing_edge -> no_drag (_context.shift_r)
            drag -> no_drag (interact_mask.left.release, map.reticule.hide_reticule2)
        }




    FSM fsm {
        State idle {
            //map.t0_y - lat2py ($lat, $map.zoomLevel) =:> screen_translation.ty
            //(lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> screen_translation.tx
            

        }
        State zoom_in {
            Double new_cx (0)
            Double new_cy (0)
            map.new_t0_y - lat2py ($lat, $map.zoomLevel + 1) =: new_cy
            (lon2px ($lon, $map.zoomLevel + 1) - map.new_t0_x) =: new_cx
            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input
            Double dx (0)
            Double init_cx (0)
            screen_translation.tx =: init_cx
            new_cx - init_cx =: dx
            Double dy (0)
            Double init_cy(0)
            screen_translation.ty =: init_cy
            new_cy - init_cy =: dy
            anim.output * (dx + map.new_dx) + init_cx =:> screen_translation.tx
            anim.output * (dy + map.new_dy) + init_cy =:> screen_translation.ty
        }
        State zoom_out {
            Double new_cx (0)
            Double new_cy (0)
            map.new_t0_y - lat2py ($lat, $map.zoomLevel - 1) =: new_cy
            (lon2px ($lon, $map.zoomLevel - 1) - map.new_t0_x) =: new_cx
            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input
            Double dx (0)
            Double init_cx (0)
            screen_translation.tx =: init_cx
            new_cx - screen_translation.tx =: dx
            Double dy (0)
            Double init_cy(0)
            new_cy - screen_translation.ty =: dy
            screen_translation.ty =: init_cy
            anim.output * (dx + map.new_dx) + init_cx =:> screen_translation.tx
            anim.output * (dy + map.new_dy) + init_cy =:> screen_translation.ty
        }
        idle->zoom_in (map.prepare_zoom_in)
        zoom_in->idle (zoom_in.anim.end)
        idle->zoom_out (map.prepare_zoom_out)
        zoom_out -> idle (zoom_out.anim.end)
    }

}