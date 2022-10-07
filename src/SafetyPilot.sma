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
SafetyPilot (Process map, Process _context, Process _model, Process _svg)
{
    //APP-6A: Symbologie militaire interarmées de l'OTAN

    //context aka _context
    model aka _model
    
    Translation screen_translation (0, 0)
    
    NoOutline _
    FillColor _ ($model.color)
    FillOpacity _ (0.3)
    Circle c (0, 0, 50)
    model.radius * map.scaling_factor_correction / get_resolution ($map.zoomLevel) =:> c.r

    FillColor _ (0, 0, 0)
    FillOpacity _ (3.3) // 0.3 * 3.3 = 0.99 (opacity = 100%)

    Translation icon_translation(0, 0)

    icon << clone (_svg.icon)

    c.cx =:> icon_translation.tx
    c.cy =:> icon_translation.ty

    picking aka icon.picking

    /*Switch switch_type (uav){
        Component uav {
        }
        Component ugv {
        }
    }
    model.type =:> switch_type.state*/


    FSM drag_fsm {
        State no_drag {
            map.t0_y - lat2py ($model.lat, $map.zoomLevel) =:> c.cy
            (lon2px ($model.lon, $map.zoomLevel) - map.t0_x) =:> c.cx
        }
        State drag {
            Double init_cx (0)
            Double init_cy (0)
            Double offset_x (0)
            Double offset_y (0)
            c.cx =: init_cx
            c.cy =: init_cy
            picking.press.x - c.cx =: offset_x
            picking.press.y - c.cy =: offset_y
            picking.move.x - offset_x => c.cx
            picking.move.y - offset_y => c.cy
            px2lon ($c.cx + map.t0_x, $map.zoomLevel) => model.lon, map.reticule.pointer_lon2
            py2lat (map.t0_y - $c.cy, $map.zoomLevel) => model.lat, map.reticule.pointer_lat2
        }
        no_drag -> drag (picking.left.press, map.reticule.show_reticule2)
        drag -> no_drag (picking.left.release, map.reticule.hide_reticule2)
    }
    FSM fsm {
        State idle {
            //map.t0_y - lat2py ($model.lat, $map.zoomLevel) =:> c.cy
            //(lon2px ($model.lon, $map.zoomLevel) - map.t0_x) =:> c.cx
        }
        State zoom_in {
            Double new_cx (0)
            Double new_cy (0)
            Double new_cr (0)
            map.new_t0_y - lat2py ($model.lat, $map.zoomLevel + 1) =: new_cy
            (lon2px ($model.lon, $map.zoomLevel + 1) - map.new_t0_x) =: new_cx
            model.radius / get_resolution ($map.zoomLevel + 1) =: new_cr
            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
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
            model.radius / get_resolution ($map.zoomLevel - 1) =: new_cr
            map.new_t0_y - lat2py ($model.lat, $map.zoomLevel - 1) =: new_cy
            (lon2px ($model.lon, $map.zoomLevel - 1) - map.new_t0_x) =: new_cx
            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
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

}