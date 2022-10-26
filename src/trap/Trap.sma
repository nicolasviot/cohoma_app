use core
use gui
use base
use animation

import behavior.DraggableItemWithRadius
import gui.animation.Animator
import TrapStatusSelector
import ros_node

_native_code_
%{
    #include "cpp/coords-utils.h"
%}


_action_
change_activation_action (Process c)
%{

    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("model/id"));
    BoolProperty *active = dynamic_cast<BoolProperty*>(data->find_child("model/active")); 
    #ifndef NO_ROS
    node ->send_msg_trap_activation(id->get_value(), active->get_value()); 
    #endif
%}

_action_
hide_trap_action(Process c)
 %{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("model/id"));
    BoolProperty *deleted = dynamic_cast<BoolProperty*>(data->find_child("model/deleted"));
#ifndef NO_ROS
    node ->send_msg_trap_deleted(id->get_value(), deleted->get_value());
#endif
%}
   
_action_
update_trap_position_action(Process c)
%{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("model/id"));
    DoubleProperty *new_lat = dynamic_cast<DoubleProperty*>(data->find_child("model/lat"));
    DoubleProperty *new_lon = dynamic_cast<DoubleProperty*>(data->find_child("model/lon")); 
#ifndef NO_ROS
    node -> send_msg_update_trap_position(id->get_value(), new_lat->get_value(), new_lon->get_value());
#endif
%}


_define_
//Trap (Process _map, Process svg_trap_info, double _lat, double _lon, int _id, Process _node)
Trap (Process _map, Process _model, Process svg_trap_info, Process _ros_node)
{
    //map aka _map
    model aka _model
    //ros_node aka _ros_node

    Translation screen_translation (0, 0)

    // Encapsulates content to prevent opacities interferences with menu and localization
    Component content {
    
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

        // FIXME: Load only once SVG
        // always visible data : ID and deactivation mode 
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
                50 =: _model.radius //set radius to maximum possible radius
                0.1 =: trap_out_op.a
                1 =: global_opacity.a  
                "#" + _model.id =:> label_trap_id.text             
            }

            Component identified
            {
                240 =: red.r
                1 =: trap_out_op.a
                1 =: global_opacity.a

                20 =: _model.radius
                1 =: trap_out_op.a

                _model.str_id =:> label_trap_id.text

                // Add icons for active traps only
                Switch remotely_switch (false) {
                    Component true {
                        Translation _ (40, -5)
                        remote_icon << remotely_icon_svg.remotely_icon
                    }
                    Component false
                }
                _model.remotely_deactivate =:> remotely_switch.state
    
                // Add icons for active traps only
                Switch contact_switch (false) {
                    Component true {
                        Translation _ (-10, -5)
                        contact_icon << contact_icon_svg.contact_icon
                    }
                    Component false
                }
                _model.contact_deactivate =:> contact_switch.state
            }

            Component deactivated
            {    
                0 =: c.r //set circle radius to zero
                0.1 =: trap_out_op.a
                0.3 =: global_opacity.a
                //fill in grey
                100 =: red.r
                "#" + _model.id =:> label_trap_id.text
            }
        }
        _model.state =:> trap_state_switch.state


        // Update the position via "screen_translation" in function of lat/lon and current zoom level
        // Allow to drag via "picking"
        DraggableItemWithRadius draggable_item (_map, _model.lat, _model.lon, _model.radius, screen_translation.tx, screen_translation.ty, picking, c.r)

    }  


    ///////TRAP INFO OVERLAY ON HOVER //////
        
    FSM info_overlay_FSM {
        State idle

        State visible {
            Translation _ (0, -15)
            info << clone (svg_trap_info.trap_info)
            _model.description =:> info.description_text.text
            _model.code =:> info.code_text.text
            _model.hazard =:> info.hazard_text.text
            "#" + toString(_model.id) =:> info.id_text.text
            _model.radius =:> info.radius_text.text
            _model.remotely_deactivate ? (_model.contact_deactivate ? "Remote/Contact" : "Remote") : (_model.contact_deactivate ? "Contact" : "...") =:> info.deactivate_text.text
            _model.contact_text =:> info.contact_text.text
        }
        idle -> visible (content.picking.enter)
        visible -> idle (content.picking.leave)
    }


    // menu to manually set the state
    Spike state_manually_updated //utiliser ce spike pour mettre à jour les booléen via ros.
    Spike ask_delete //utiliser pour supprimer le trap

    AssignmentSequence unknown_assignement (1){
        1 =: _model.active
        0 =: _model.identified 
    }

    AssignmentSequence identified_assignement (1){
        1 =: _model.active
        1 =: _model.identified 
    }        

    AssignmentSequence deactivated_assignement (1){
        0 =: _model.active
        1 =: _model.identified 
    }

    AssignmentSequence delete_assignement (1){
        //0 =: _model.active
        //0 =: _model.identified 
        1 =: _model.deleted
        0.01 =: content.global_opacity.a
    }

    // FIXME: only once for whole app
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


    Spike moved
    _model.lat -> moved
    _model.lon -> moved

    FSM update_trap_position {
        State idle

        State going_to_update {
            Timer t(5000)
        }
        idle -> going_to_update (moved)
        going_to_update -> idle (update_trap_position.going_to_update.t.end, update_trap_position_native)
    }
}