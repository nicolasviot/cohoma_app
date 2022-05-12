use core
use base
use display
use gui
use animation



import gui.animation.Animator

_native_code_
%{
#include "cpp/coords-utils.h"

%}


_define_
TaskAreaSummit(Process map, double _lat, double _lon){



	Double x(0)
	Double y(0)
    Double lat($_lat)
    Double lon($_lon)
    Double alt(0)

    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty

    Translation screen_translation(0, 0)

    screen_translation.tx + pos.tx=:> x
    screen_translation.ty + pos.ty=:> y

    map.t0_y - lat2py ($lat, $map.zoomLevel) =:> screen_translation.ty
    (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> screen_translation.tx

/*
    FSM fsm {
        State idle {
            map.t0_y - lat2py ($lat, $map.zoomLevel) =:> screen_translation.ty
            (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> screen_translation.tx
            

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

*/

}