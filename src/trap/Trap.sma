use core
use gui
use base
use animation

import behavior.DraggableItem
import gui.animation.Animator
import TrapStatusSelector
import ros_node

_native_code_
%{
    #include "cpp/coords-utils.h"
    #include "core/utils/getset.h"
%}


_action_
change_activation_action (Process c)
%{

    Process *data = (Process*) get_native_user_data(c);
    // GET_CHILD(RosNode, data, node);
    // GET_CHILD(IntProperty, data, id);
    // GET_CHILD(BoolProperty, data, active);
    RosNode *node = dynamic_cast<RosNode*>(data->find_child("node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    BoolProperty *active = dynamic_cast<BoolProperty*>(data->find_child("active")); 
    #ifndef NO_ROS
    node ->send_msg_trap_activation(id->get_value(), active->get_value()); 
    #endif
%}

_action_
hide_trap_action(Process c)
 %{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    BoolProperty *deleted = dynamic_cast<BoolProperty*>(data->find_child("deleted"));
#ifndef NO_ROS
    node ->send_msg_trap_deleted(id->get_value(), deleted->get_value());
#endif
%}
   
_action_
update_trap_position_action(Process c)
%{
    Process *data = (Process*) get_native_user_data(c);
    RosNode *node = dynamic_cast<RosNode*>(data->find_child("node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    DoubleProperty *new_lat = dynamic_cast<DoubleProperty*>(data->find_child("lat"));
    DoubleProperty *new_lon = dynamic_cast<DoubleProperty*>(data->find_child("lon")); 
#ifndef NO_ROS
    node -> send_msg_update_trap_position(id->get_value(), new_lat->get_value(), new_lon->get_value());
#endif
%}


_define_
Trap (Process map, Process svg_trap_info, double _lat, double _lon, int _id, Process _node)
{

    Double lat($_lat)
    Double lon($_lon)
    Double altitude_msl(0)
    Int id($_id)
    Bool identified(0)
    Bool active(1)
    Bool deleted (0)
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
    String description("..")
    Double radius(30)
    Bool remotely_deactivate(0)
    Bool contact_deactivate(0)
    Int contact_mode(0)
    String code("?")
    String hazard("?")

    Translation screen_translation (0, 0)

    //encapsulating content to prevent opacities interferences with menu and localization
    Component content {
    
        //Rectangle
        OutlineOpacity trap_out_op (0)
        OutlineColor _ (0, 0, 0)
        OutlineWidth _ (2)
        FillOpacity global_opacity (1)
        FillColor red (240, 0, 0)
        Component losange {
            Rotation rot (45, 0, 0)
            Rectangle rect (-15, -15, 30, 30)
        }
        // for drag interaction
        picking aka losange.rect

        NoOutline _

        // Circle (linked with radius)
        FillOpacity circle_opacity(0.1)
        Circle c (0, 0, 50)

        //always visible data : ID and deactivation mode 
        remotely_icon_svg = loadFromXML ("res/svg/trap_remote_icon.svg")
        contact_icon_svg = loadFromXML ("res/svg/trap_contact_icon.svg")
        
        //text for identification and information
        FillColor _ (0,0,0)
        FillOpacity text_opacity (3)
        1 / circle_opacity.a =:> text_opacity.a

        FontSize _ (0, 10)
        TextAnchor _ (DJN_MIDDLE_ANCHOR)
        Text label_trap_id (0, 5, "?")

         
        // state switch
        Switch trap_state_switch (unknown) {
            Component unknown
            {   
                //fill in red
                240 =: red.r
                50 =: radius //set radius to maximum possible radius
                0.1 =: trap_out_op.a
                1 =: global_opacity.a  
                "#" + id =:> label_trap_id.text             
            }

            Component identified
            {
                240 =: red.r
                1 =: trap_out_op.a
                1 =: global_opacity.a
                //radius * 1.52 /get_resolution ($map.zoomLevel) =:> c.r
                20 =: radius
                1 =: trap_out_op.a

                trap_id =:> label_trap_id.text

                //add icons for active traps only
                Switch remotely_switch (false) {
                    Component true {
                        Translation _ (40, -5)
                        remote_icon << remotely_icon_svg.remotely_icon
                    }
                    Component false
                }
                remotely_deactivate =:> remotely_switch.state
    
                //add icons for active traps only
                Switch contact_switch (false) {
                    Component true {
                        Translation _ (-10, -5)
                        contact_icon << contact_icon_svg.contact_icon
                    }
                    Component false
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
                "#" + id =:> label_trap_id.text
            }
        }
        state =:> trap_state_switch.state


        // Update the position via "screen_translation" in function of lat/lon and current zoom level
        // Allow to drag via "picking"
        DraggableItem draggable_item (map, lat, lon, radius, screen_translation.tx, screen_translation.ty, picking, c.r)

    }  


    ///////TRAP INFO OVERLAY ON HOVER //////
        
    FSM info_overlay_FSM {
        State idle

        State visible{
            Translation _ (0, -15)
            info << clone (svg_trap_info.trap_info)
            description =:> info.description_text.text
            code =:> info.code_text.text
            hazard =:> info.hazard_text.text
            "#" + toString(id) =:> info.id_text.text
            radius =:> info.radius_text.text
            remotely_deactivate ? (contact_deactivate ? "Remote/Contact" : "Remote") : (contact_deactivate ? "Concact" : "...") =:> info.deactivate_text.text

            /*int8 CONTACT_UNKONWN = 0
            int8 CONTACT_AERIAL = 1
            int8 CONTACT_GROUND = 2
            int8 CONTACT_GROUND_MULTIPLE = 3
            int8 CONTACT_AERIAL_AND_GROUND = 4
            int8 CONTACT_AERIAL_OR_GROUND = 5*/
            SwitchList contact_mode_switch (0){
                Component zero {
                    "unknown" =: info.contact_text.text
                }
                Component one {
                    "Aerial" =: info.contact_text.text
                }
                Component two {
                    "Ground" =: info.contact_text.text
                }
                Component three {
                    "Ground Multiple" =: info.contact_text.text
                }
                Component four {
                    "Aerial And Ground" =:info.contact_text.text
                }
                Component five {
                    "Aerial or Ground" =: info.contact_text.text
                }
            }
            contact_mode + 1 =:> contact_mode_switch.index
            
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
        1 =: deleted
        0.01 =: content.global_opacity.a
   }

    TrapStatusSelector menu (this)

    content.picking.right.press -> menu.press
    state_manually_updated -> menu.hide
    ask_delete -> menu.hide

    NativeAction update_trap_activation_state_action(change_activation_action, this, 1)
    NativeAction hide_trap_native(hide_trap_action, this, 1)
    NativeAction update_trap_position_native(update_trap_position_action, this, 1)
    delete_assignement -> hide_trap_native
    deactivated_assignement -> update_trap_activation_state_action
    identified_assignement -> update_trap_activation_state_action
    unknown_assignement -> update_trap_activation_state_action


    /*///// HIGHLIGHT ANIMATION ON REQUEST /////
    Spike start_highlight_anim
    Spike stop_highlight_anim

    FSM locate_FSM {
        State idle

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
       
        idle -> animate (start_highlight_anim)
        animate -> idle (stop_highlight_anim)
    }*/ 

    Spike moved
    lat -> moved
    lon -> moved

    FSM update_trap_position {
        State idle

        State going_to_update {
            Timer t(5000)
        }
        idle -> going_to_update (moved)
        going_to_update -> idle (update_trap_position.going_to_update.t.end, update_trap_position_native)
    }
}