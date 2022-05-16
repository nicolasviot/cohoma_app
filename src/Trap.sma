use core
use gui
use base
use animation

import gui.animation.Animator
import TrapStatusSelector
import ros_node
_native_code_
%{
#include "cpp/coords-utils.h"
/*unsigned long RGBToHex(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}*/
%}


_action_
change_activation_action (Process c)
%{

    Process *data = (Process*) get_native_user_data(c);
    RosNode *node = dynamic_cast<RosNode*>(data->find_child("node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    BoolProperty *new_activation = dynamic_cast<BoolProperty*>(data->find_child("active")); 
    #ifndef NO_ROS
    node ->send_msg_trap_activation(id->get_value(), new_activation->get_value()); 
    #endif
    
%}



_define_
Trap (Process map, double _lat, double _lon, int _id, Process _node)
{

    Double lat($_lat)
    Double lon($_lon)
    Double altitude_msl(0)
    Int id($_id)
    Bool identified(0)
    Bool active(1)
    String state ("unknown") //can be unkown, identified, deactivated
    String trap_id("?")

    node aka _node
    active ? (identified ? "identified" : "unknown") : "deactivated" =:> state


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
    Double radius(30)
    Bool remotely_deactivate(1)
    Bool contact_deactivate(1)
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


  
    //encapsulating content to prevent opacities interferences with menu and localization
    Component content{

    
        //Rectangle
        OutlineOpacity trap_out_op (0)
        OutlineColor _ (0,0,0)
        OutlineWidth _ (2)
        FillOpacity global_opacity (1)
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
        radius * 1.52 /get_resolution ($map.zoomLevel) =:> c.r //attention peut etre pas tout le temps

        //rotation of the rectangle to be a losange
        c.cx =:> rot.cx
        c.cy =:> rot.cy
        rot.cx =:> un_rot.cx
        rot.cy =:> un_rot.cy

        //for drag interaction
        picking aka rect

        //always visible data : ID and deactivation mode 
        remotely_icon_svg = loadFromXML ("res/svg/trap_remote_icon.svg")
        contact_icon_svg = loadFromXML ("res/svg/trap_contact_icon.svg")
        
        //text for identification and information
        FillColor _ (0,0,0)
        FillOpacity text_opacity (3)
        1 / circle_opacity.a =:> text_opacity.a
        FontSize _ (0, 12)
        TextAnchor _ (1)
        Text trap_id_text (0,0, "?")
        trap_id =:> trap_id_text.text
        c.cx =:> trap_id_text.x
        c.cy + 5 =:> trap_id_text.y

         
        // state switch
        Switch trap_state_switch(unknown){
            Component unknown
            {   
                //fill in red
                240 =: red.r
                50 =: radius //set radius to maximum possible radius
                0.1 =: trap_out_op.a
                1 =: global_opacity.a               
            }
            Component identified
            {
                240 =: red.r
                1 =: trap_out_op.a
                1 =: global_opacity.a
                radius * 1.52 /get_resolution ($map.zoomLevel) =:> c.r
                1 =: trap_out_op.a

                //Translation to match the content (TODO:should have a unifed technique instead....)  
                Translation rect_pos (0,0)
                content.rect.x =:> rect_pos.tx
                content.rect.y =:> rect_pos.ty


                //add icons for active traps only
                Switch remotely_switch (true){
                    Component true{
                        Translation _ (40,-5)
                        remote_icon << remotely_icon_svg.remotely_icon
                    }
                    Component false{

                    }
                }
    
                //add icons for active traps only
                Switch contact_switch (true){
                    Component true{
                        Translation _ (-10,-5)
                        contact_icon << contact_icon_svg.contact_icon
                    }
                    Component false{

                    }
                }
                contact_deactivate =:> contact_switch.state

            }
            Component deactivated
        {    
                0 =: c.r //set circle radius to zero
                0.1 =: trap_out_op.a
                0.3 =: global_opacity.a
                //fill in grey
                100 =: red.r
        }
        }
        state =:> trap_state_switch.state


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

    }  

    //Translation to match the content (TODO:should have a unifed technique instead....)  
    Translation rect_pos (0,0)
    content.rect.x =:> rect_pos.tx
    content.rect.y =:> rect_pos.ty

    ///////TRAP INFO OVERLAY ON HOVER //////


        overlay_svg = loadFromXML ("res/svg/trap_info.svg")
        
        FSM info_overlay_FSM{
            State idle{

            }
            State visible{
                Translation _ (15,0)
                info << overlay_svg.trap_info
               description =:> info.description_text.text
                      code =:> info.code_text.text
                    hazard =:> info.hazard_text.text
               contact_mode=:> info.contact_text.text
                
            }
            idle -> visible (content.picking.enter)
            visible -> idle (content.picking.leave)
        }



    //menu to manually set the state
    Spike state_manually_updated //utiliser ce spike pour mettre à jour les booléen via ros.
    Spike ask_delete //utiliser pour supprimer le trap

    AssignmentSequence unknown_assignement (1){
        1 =: active
        0 =: identified 
    }

    AssignmentSequence identified_assignement (1){
        1 =: active
        1 =: identified 
    }        

    AssignmentSequence deactivated_assignement (1){
        0 =: active
        1 =: identified 
   }

    AssignmentSequence delete_assignement (1){
        //0 =: active
        //0 =: identified 
        0.01 =: content.global_opacity.a
   }

    TrapStatusSelector menu (this)

    content.picking.right.press -> menu.press
    state_manually_updated -> menu.hide
    ask_delete -> menu.hide
    NativeAction update_trap_activation_state_action(change_activation_action, this, 1)
    deactivated_assignement -> update_trap_activation_state_action
    identified_assignement -> update_trap_activation_state_action
    unknown_assignement -> update_trap_activation_state_action

 //HIGHLIGHT ANIMATION ON REQUEST /////
    Spike start_highlight_Anim
    Spike stop_highlight_Anim

    FSM locate_FSM{
        State idle{

        }
        State animate{
            Double radius(60)
            OutlineWidth _ (4)
            OutlineColor _ ($content.red.value)
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