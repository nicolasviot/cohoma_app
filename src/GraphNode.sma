use core
use gui
use base
use animation

import gui.animation.Animator

_native_code_
%{
#include "cpp/coords-utils.h"
unsigned long RGBToHex(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}
%}

_define_
GraphNode(Process map, double _lat, double _lon, int r, int g, int b)
{


//APP-6A
    String usage_status ("default")
    Int node_col (#CCCCCC)
    Int white_col (#FFFFFF)
    Int black_col (#000000)
    Int active_col (#29ABE2)
    Int start_col (#70EE49)
    Int mandatory_col (#FF30FF)

    Int default_radius (5)
    Int other_radius (8)

    Double lat($_lat)
    Double lon($_lon)
    Double altitude_msl(0)
    Double battery_voltage(0)
    Double heading_rot(0)

    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty

    Translation screen_translation(0, 0)
    Rotation rot (0, 0, 0)
    screen_translation.tx =:> rot.cx
    screen_translation.ty =:> rot.cy
    heading_rot =:> rot.a

    //graphical variables to be updated in different status
    FillOpacity fill_opacity (0.6)
    opacity aka fill_opacity.a
    FillColor fill_color (r, g, b)

    OutlineColor outline_color (r, g, b)
    OutlineWidth outline_width (1)
    OutlineOpacity outline_opacity(0.5)

    Circle c (0, 0, 4)

    // API (TODO)
    leave aka c.leave
    right_press aka c.right.press


    Switch status_switch (default) {
        Component default {
             node_col =: fill_color.value
            white_col =: outline_color.value

                1 =: outline_width.width
            default_radius =: c.r
        }
        Component start {
            active_col =: fill_color.value
             start_col =: outline_color.value
            
                     2 =: outline_width.width
          other_radius =: c.r
        }
        Component end {
              active_col =: fill_color.value
           mandatory_col =: outline_color.value
            other_radius =: c.r
                       2 =: outline_width.width
        }
        Component forced {
            active_col =: fill_color.value
             black_col =: outline_color.value

             other_radius =: c.r
                        2 =: outline_width.width
        }
        Component mandatory {
               active_col =: fill_color.value
            mandatory_col =: outline_color.value

                        1 =: outline_width.width

            NoFill _ 
            OutlineWidth _ (2)
            Circle outer_circler (0,0, 12)
        }
    }
    usage_status => status_switch.state

   
  FSM drag_fsm {
        State no_drag {
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
            c.press.x - screen_translation.tx =: offset_x
            c.press.y - screen_translation.ty =: offset_y
            c.move.x - offset_x => screen_translation.tx
            c.move.y - offset_y => screen_translation.ty
            px2lon ($screen_translation.tx + map.t0_x, $map.zoomLevel) => lon
            py2lat (map.t0_y - $screen_translation.ty, $map.zoomLevel) => lat 
        }
        no_drag->drag (c.left.press, map.reticule.show_reticule)
        drag->no_drag (c.left.release, map.reticule.hide_reticule)
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
            Animator anim (1000, 0, 1, DJN_IN_SINE, 0, 1)
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
            Animator anim (1000, 0, 1, DJN_IN_SINE, 0, 1)
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