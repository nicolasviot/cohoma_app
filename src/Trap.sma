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
Trap (Process map, double _lat, double _lon, int _id)
{

    Double lat($_lat)
    Double lon($_lon)
    Double altitude_msl(0)
    Int id($_id)
    Bool identified(1)
    Bool active(1)
    String state ("unknown") //can be unkown, identified, deactivated
    String trap_id("?")

    
    active ? (identified ? "identified" : "unkown") : "deactivated" =:> state


    /*
    string description                  # text describing the kind of trap
    float32 radius                      # action radius [m]
    bool remotely_deactivate            # whether the trap can be deactivated remotely
    bool contact_deactivate             # whether the trap can be deactivated through contact
    int8 contact_mode                   # which type of satellite can deactivate; see enum
    string code                        # code to deactivate the trap
    string hazard                       # description of an hazardous situation to take into account

    */
    String description("this is a trap")
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
    String code("?")
    String hazard("?")

    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty
    Translation screen_translation (0, 0)
    

    //Rectangle
    OutlineOpacity trap_out_op (0)
    OutlineColor _ (0,0,0)
    OutlineWidth _ (2)
    FillOpacity golbal_opacity (1)
    FillColor red(240, 0, 0)
    Rotation rot (45, 0, 0)
    Rectangle rect (0, 0, 30, 30)
    Rotation un_rot (-45, 0, 0)

    NoOutline _

    //Circle (linked with radius)
    FillOpacity circle_opacity(0.3)
    Circle c (0, 0, 50)
    c.cx - rect.width/2 =:> rect.x 
    c.cy - rect.height/2 =:> rect.y
    radius/get_resolution ($map.zoomLevel) =:> c.r //attention peut etre pas tout le temps

    //rotation of the rectangle to be a losange
    c.cx =:> rot.cx
    c.cy =:> rot.cy
    rot.cx =:> un_rot.cx
    rot.cy =:> un_rot.cy

    //for drag interaction
    picking aka rect


    //text for identification and information
    FillColor _ (0,0,0)
    FillOpacity text_opacity (3)
    1 / circle_opacity.a =:> text_opacity.a
    FontSize _ (0, 18)
    TextAnchor _ (1)
    Text trap_id_text (0,0, "?")
    trap_id =:> trap_id_text.text
    c.cx =:> trap_id_text.x
    c.cy + 5 =:> trap_id_text.y

    FontSize _ (0, 14)
    Text trap_description_text (0,0, "...")
    c.cx =:> trap_description_text.x
    c.cy + 30 =:> trap_description_text.y

    Text trap_description_text2 (0,0, "...")
    c.cx =:> trap_description_text2.x
    c.cy + 50 =:> trap_description_text2.y

    Text trap_description_text3 (0,0, "...")
    c.cx =:> trap_description_text3.x
    c.cy + 70 =:> trap_description_text3.y

    Text trap_description_text4 (0,0, "...")
    c.cx =:> trap_description_text4.x
    c.cy + 90 =:> trap_description_text4.y
    
    description =:> trap_description_text.text

    // state switch
    Switch trap_state_switch(unknown){
        Component unknown
        {
              50 =: radius //set radius to maximum possible radius
        "unkown" =: trap_description_text.text
             " " =: trap_description_text2.text
             " " =: trap_description_text3.text
             " " =: trap_description_text4.text
            
        }
        Component identified
        {
        
            radius/get_resolution ($map.zoomLevel) =:> c.r
            "Hazard:" + hazard + " Code:"+code =:> trap_description_text2.text
            "Remotely:"+ toString(remotely_deactivate) + "  " + "Contact:" + toString(contact_deactivate) + " " =:> trap_description_text3.text
            "contact mode:"+ toString(contact_mode) =:> trap_description_text4.text
            1 =: trap_out_op.a

        }
        Component deactivated
       {    
            0 =: radius
            0.1 =: trap_out_op.a
            0.3 =: golbal_opacity.a
            //fill in grey
            100 =: red.r
            " " =: trap_description_text.text
            " " =: trap_description_text2.text
            " " =: trap_description_text3.text
            " " =: trap_description_text4.text

       }
    }
    state =:> trap_state_switch.state

    FSM set_State_Menu{
        State hidden{

        }
        State visible{
            Rectangle unknown_rect ( 0,0, 20, 20, 0,0)

        }
    }

    TextPrinter tp
    state =:> tp.input

  

  FSM drag_fsm {
        State no_drag {
            map.t0_y - lat2py ($lat, $map.zoomLevel) =:> c.cy
            (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> c.cx
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
            px2lon ($c.cx + map.t0_x, $map.zoomLevel) => lon
            py2lat (map.t0_y - $c.cy, $map.zoomLevel) => lat 
        }
        no_drag->drag (picking.left.press, map.reticule.show_reticule)
        drag->no_drag (picking.left.release, map.reticule.hide_reticule)
    }
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
            radius/get_resolution ($map.zoomLevel - 1) =: new_cr
            map.new_t0_y - lat2py ($lat, $map.zoomLevel - 1) =: new_cy
            (lon2px ($lon, $map.zoomLevel - 1) - map.new_t0_x) =: new_cx
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


 //HIGHLIGHT ANIMATION ON REQUEST /////
    Spike start_highlight_Anim
    Spike stop_highlight_Anim

    FSM locate_FSM{
        State idle{

        }
        State animate{
            Double radius(60)
            OutlineWidth _ (4)
            OutlineColor _ ($red.value)
            Circle c (0, 0, $radius)
            radius =:> c.r

            Clock timer (30)
            Incr ellapsedIncr (0)

            AssignmentSequence reset_radius (1){
                0 =: ellapsedIncr.state
            }

            |-> reset_radius
            
            timer.tick -> ellapsedIncr

            60 - ellapsedIncr.state * 3 =:> radius
            (radius <= 5) -> reset_radius
        }
       
        idle -> animate (start_highlight_Anim)
        animate -> idle (stop_highlight_Anim)
    }    

}