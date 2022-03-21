use core
use gui
use base
use animation

import gui.animation.Animator

_native_code_
%{
#include "cpp/coords-utils.h"
unsigned long createRGB(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}
%}

_define_
Waypoints (Process map, double _lat, double _lon, int r, int g, int b)
{


//APP-6A
    String usage_status ("usable")
    Int default_col (createRGB (r, g, b))
    Int usable_col (Blue)
    Int forbidden_col (Red)
    Int mandatory_col (#0CF266)
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
    FillOpacity fo (1)
    opacity aka fo.a
    
    FillColor my_fc (r, g, b)
    NoOutline _

    Circle c (0, 0, 8)
    OutlineColor oc (r, g, b)
    OutlineWidth _ (5)
    OutlineOpacity outline_opacity(1)
    Switch ctrl_color (default) {
        Component default {
            default_col =: my_fc.value, oc.value
        }
        Component usable {
            usable_col =: my_fc.value, oc.value
        }
        Component forbidden {
            forbidden_col =: my_fc.value, oc.value
        }
        Component mandatory {
            mandatory_col =: my_fc.value, oc.value
        }
    }
    usage_status => ctrl_color.state
    Rotation rot (0, 0, 0)
    c.cx =:> rot.cx
    c.cy =:> rot.cy
    heading_rot =:> rot.a
    Line compass_heading (0, 0, 0, -20)
    c.cx =:> compass_heading.x1
    c.cy =:> compass_heading.y1
    c.cx =:> compass_heading.x2
    c.cy - 20 =:> compass_heading.y2    
  
    FSM fsm {
        State idle {
            map.t0_y - lat2py ($lat, $map.zoomLevel) =:> c.cy
            (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> c.cx
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
            c.cx =: init_cx
            new_cx - init_cx =: dx
            Double dy (0)
            Double init_cy(0)
            c.cy =: init_cy
            new_cy - init_cy =: dy
            anim.output * (dx + map.new_dx) + init_cx =:> c.cx
            anim.output * (dy + map.new_dy) + init_cy =:> c.cy
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
            c.cx =: init_cx
            new_cx - c.cx =: dx
            Double dy (0)
            Double init_cy(0)
            new_cy - c.cy =: dy
            c.cy =: init_cy
            anim.output * (dx + map.new_dx) + init_cx =:> c.cx
            anim.output * (dy + map.new_dy) + init_cy =:> c.cy
        }
        idle->zoom_in (map.prepare_zoom_in)
        zoom_in->idle (zoom_in.anim.end)
        idle->zoom_out (map.prepare_zoom_out)
        zoom_out -> idle (zoom_out.anim.end)
    }

}