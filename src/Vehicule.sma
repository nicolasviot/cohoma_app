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
Vehicule (Process map, double _lat, double _lon, int r, int g, int b, string init_state)
{

    
    Double lat($_lat)
    Double lon($_lon)
    Double battery_voltage(0)
    Int battery_percentage(0)
    Double altitude_msl(0)
    Double heading_rot(0)
    Bool emergency_stop(0)
    Bool failsafe(0)
    Int operation_mode(0)



    String state(init_state)
    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty
 
    Translation screen_translation (0, 0)
    Rotation rot (0, 0, 0)
    screen_translation.tx =:> rot.cx
    screen_translation.ty =:> rot.cy
    heading_rot =:> rot.a
    // [insert beautiful graphics here]

    Switch graphics(vab) {
        Component vab{
            svg = loadFromXML ("res/svg/vab.svg")
            icon << svg.icon
        }
        Component agilex1{
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            FillColor agiCOL (200, 40, 40)
            agiCOL.value =: icon.shape.fill.value
        }
        Component agilex2{
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            FillColor agiCOL2 (160, 40, 40)
            agiCOL2.value =: icon.shape.fill.value
        }
        Component lynx {
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            FillColor lynxCOL (140, 40, 40)
            lynxCOL.value =: icon.shape.fill.value
        }
        Component spot {
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            FillColor spotCOL (100, 40, 40)
            spotCOL.value =: icon.shape.fill.value
        }
        Component drone {
            svg = loadFromXML ("res/svg/drone.svg")
            icon << svg.icon
          
        }
    } 

    state =:> graphics.state
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

}