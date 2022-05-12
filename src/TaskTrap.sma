use core 
use gui
use display
use base
use animation

import gui.animation.Animator
_native_code_
%{
#include "cpp/coords-utils.h"
%}



_define_
TaskTrap (Process map, int _trap_id, double _lat, double _lon){

	Int trap_id(_trap_id)

    Double lat($_lat)
    Double lon($_lon)
    Bool selected (0)

    Bool identified(0)
    Bool active(0)

    //identification 

    /*builtin_interfaces/Time stamp       # identification stamp
    uint8 robot_id                      # Robot ID, see RobotState.msg
    geographic_msgs/GeoPoint location   # location
    string id                            # 4 digits
    string description                  # text describing the kind of trap
    float32 radius                      # action radius [m]
    bool remotely_deactivate            # whether the trap can be deactivated remotely
    bool contact_deactivate             # whether the trap can be deactivated through contact
    int8 contact_mode                   # which type of satellite can deactivate; see enum
    string code                        # code to deactivate the trap
    string hazard                       # description of an hazardous situation to take into account

    */
    String trap_id_str("")
    String description("")

    Double radius(50)
    Bool remotely_deactivate(0)
    Bool contact_deactivate(0)
    Int contact_mode(0)
    /*int8 CONTACT_UNKONWN = 0
      int8 CONTACT_AERIAL = 1
      int8 CONTACT_GROUND = 2
      int8 CONTACT_GROUND_MULTIPLE = 3
      int8 CONTACT_AERIAL_AND_GROUND = 4
      int8 CONTACT_AERIAL_OR_GROUND = 5*/
    String code("")
    String hazard("")
    
    String trap_status("radius_unknown")
    
    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty
    Translation screen_translation (0, 0)
    
    NoOutline _
    FillColor red(240, 0, 0)
    Rotation rot (45, 0, 0)
    Rectangle rect (0, 0, 20, 20)
    Rotation un_rot (-45, 0, 0)

    OutlineWidth circle_perimeter_width(0)
    OutlineColor yellow(255, 255, 0)
    Spike toogleselect
    Switch ctrl_trap_selected(not_select){ 
    
    Component select { 
        5 =: circle_perimeter_width.width
        255 =: yellow.r
        255 =: yellow.g
        0 =: yellow.b
    }
    Component not_select{
        0 =: circle_perimeter_width.width
        0 =: yellow.r
        0 =: yellow.g
        0 =: yellow.b

    }

   }
   selected?"select":"not_select" => ctrl_trap_selected.state


    FillOpacity _(0.3)
    Circle c (0, 0, 50)
    c.cx - rect.width/2 =:> rect.x 
    c.cy - rect.height/2 =:> rect.y
    radius/get_resolution ($map.zoomLevel) =:> c.r

    //rotation of the rectangle to be a losange
    c.cx =:> rot.cx
    c.cy =:> rot.cy
    rot.cx =:> un_rot.cx
    rot.cy =:> un_rot.cy

      rect.press -> toogleselect
   c.press -> toogleselect
   toogleselect ->{
    selected?0:1 =: selected 
   }



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
        //no_drag->drag (c.left.press, map.reticule.show_reticule)
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