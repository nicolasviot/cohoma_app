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
Vehicule (Process map, double _lat, double _lon, string init_state, int _color, Process _svg)
{
    Double lat (_lat)
    Double lon (_lon)
    String state(init_state)
    Int color (_color)

    Double battery_voltage(24)
    Int battery_percentage(75)
    Double altitude_msl(500)
    Double heading_rot(180)
    Bool emergency_stop(0)
    Bool failsafe(0)
    Int operation_mode(0)

    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty
 
    Translation screen_translation (0, 0)
    Rotation rot (0, 0, 0)
    heading_rot =:> rot.a

    // [insert beautiful graphics here]
    icon << clone (_svg.icon)
    color =: icon.shape.fill.value

    /*Switch graphics (vab) {
        Component vab {
        }
        Component agilex1 {
        }
        Component agilex2 {
        }
        Component lynx {
        }
        Component spot {
        }
        Component drone {
        }
    }
    state =:> graphics.state*/


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


    //HIGHLIGHT ANIMATION ON REQUEST /////
    Spike startAnim
    Spike stopAnim


    //TODO compute radius according to the map center
    Double radius(60)
    Double distance_to_center_pix (100)

    ClampMin clamp_radius (60, 60)
    distance_to_center_pix =:> clamp_radius.input
    clamp_radius.result =:> radius

    Animator radius_anim (1000, 60, 5, DJN_IN_OUT_SINE, 1, 0)
    distance_to_center_pix =:> radius_anim.min

    20 =: radius_anim.fps
    radius_anim.output =:> radius
    startAnim -> radius_anim.start
    stopAnim -> radius_anim.reset
    stopAnim -> radius_anim.abort

    FSM locate_FSM{
        State idle{

        }
        State animate{

            OutlineWidth _ (4)
            OutlineColor _ ($color)
            Circle c (0, 0, $radius)
            radius =:> c.r
        }
       
        idle -> animate (startAnim)
        animate -> idle (stopAnim)
    }

}