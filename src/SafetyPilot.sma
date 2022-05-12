use core
use gui
use base
use animation

import gui.animation.Animator

_native_code_
%{
#include "cpp/coords-utils.h"
/*unsigned long RGBToHex(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}*/
%}

_define_
SafetyPilot (Process map, double _lat, double _lon, int _id)
{


//APP-6A

    Double lat($_lat)
    Double lon($_lon)
    Double altitude_msl(0)
    Double radius(100)
    Int id($_id)
    
    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty
    Translation screen_translation (0, 0)
    

    


    NoOutline _
    FillColor circleFill (#A056F6)
    FillOpacity _(0.3)
    Circle c (0, 0, 50)
    radius/get_resolution ($map.zoomLevel) =:> c.r

    FillColor _ (0,0,0)
    FillOpacity _ (3.3) //ahah sinon ça se multiplie
    Translation icon_translation(0,0)
    svg = loadFromXML("res/svg/safety_pilot.svg")
    icon << svg.icon

    c.cx =:> icon_translation.tx
    c.cy =:> icon_translation.ty

    

  FSM drag_fsm {
        State no_drag {
            map.t0_y - lat2py ($lat, $map.zoomLevel) =:> c.cy
            (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> c.cx
            radius/get_resolution ($map.zoomLevel) =:> c.r
        }
        State drag {
            Double init_cx (0)
            Double init_cy (0)
            Double offset_x (0)
            Double offset_y (0)
            c.cx =: init_cx
            c.cy =: init_cy
            c.press.x - c.cx =: offset_x
            c.press.y - c.cy =: offset_y
            c.move.x - offset_x => c.cx
            c.move.y - offset_y => c.cy
            px2lon ($c.cx + map.t0_x, $map.zoomLevel) => lon
            py2lat (map.t0_y - $c.cy, $map.zoomLevel) => lat 
        }
        no_drag->drag (c.left.press, map.reticule.show_reticule)
        drag->no_drag (c.left.release, map.reticule.hide_reticule)
    }
    

/*
    FSM fsm {
        State idle {
            //map.t0_y - lat2py ($lat, $map.zoomLevel) =:> c.cy
            //(lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> c.cx
            

        }
        State zoom_in {
            Double new_cx (0)
            Double new_cy (0)
            Double new_cr (0)
            map.new_t0_y - lat2py ($lat, $map.zoomLevel + 1) =: new_cy
            (lon2px ($lon, $map.zoomLevel + 1) - map.new_t0_x) =: new_cx
            radius/get_resolution ($map.zoomLevel + 1) =: new_cr
            Animator anim (1000, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input
            Double dx (0)
            Double dr (0)
            Double init_cx (0)
            Double init_cr (0)
            c.cx =: init_cx
            c.r =: init_cr
            new_cx - init_cx =: dx
            Double dy (0)
            Double init_cy(0)
            c.cy =: init_cy
            new_cy - init_cy =: dy
            new_cr - init_cr =: dr
            anim.output * (dx + map.new_dx) + init_cx =:> c.cx
            anim.output * (dy + map.new_dy) + init_cy =:> c.cy
            anim.output * dr + init_cr =:> c.r
        }
        State zoom_out {
            Double new_cx (0)
            Double new_cy (0)
            Double new_cr (0)
            radius/get_resolution ($map.zoomLevel - 1) =: new_cr
            map.new_t0_y - lat2py ($lat, $map.zoomLevel - 1) =: new_cy
            (lon2px ($lon, $map.zoomLevel - 1) - map.new_t0_x) =: new_cx
            Animator anim (1000, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input
            Double dx (0)
            Double dr (0)
            Double init_cx (0)
            Double init_cr (0)
            c.cx =: init_cx
            c.r =: init_cr
            new_cx - c.cx =: dx
            Double dy (0)
            Double init_cy(0)
            new_cy - c.cy =: dy
            c.cy =: init_cy
            new_cr - init_cr =: dr
            anim.output * (dx + map.new_dx) + init_cx =:> c.cx
            anim.output * (dy + map.new_dy) + init_cy =:> c.cy
            anim.output * dr + init_cr =:> c.r
        }
        idle->zoom_in (map.prepare_zoom_in)
        zoom_in->idle (zoom_in.anim.end)
        idle->zoom_out (map.prepare_zoom_out)
        zoom_out -> idle (zoom_out.anim.end)
    }
*/
}