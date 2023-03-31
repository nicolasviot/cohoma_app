use core
use gui
use base

import ros_node

_native_code_
%{
    #include <iostream>
    using namespace std;

    string get_contact_mode (int contact_mode)
    {
        //cout << "get_contact_mode: " << contact_mode << endl;
        switch (contact_mode) {
            case 10:
                return "";
            case 11:
                return "Aérien";
            case 12:
                return "Terrestre";
            case 13:
                return "Terrestres";
            case 14:
                return "Aérien et Terrestre";
            case 15:
                return "Aérien ou Terrestre";
            default:
                return "";
        }
        return "";
    }
%}


_action_
action_update_active_trap (Process c)
%{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    int uid = static_cast<IntProperty*>(data->find_child("id"))->get_value();
    bool active = static_cast<BoolProperty*>(data->find_child("active"))->get_value();

#ifndef NO_ROS
    node->send_msg_trap_activation (uid, active); 
#endif
%}

_action_
action_delete_trap (Process c)
 %{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    int uid = static_cast<IntProperty*>(data->find_child("id"))->get_value();

#ifndef NO_ROS
    node->send_msg_trap_deleted (uid);
#endif
%}
   
/*_action_
update_trap_position_action(Process c)
%{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    DoubleProperty *new_lat = dynamic_cast<DoubleProperty*>(data->find_child("lat"));
    DoubleProperty *new_lon = dynamic_cast<DoubleProperty*>(data->find_child("lon")); 
#ifndef NO_ROS
    node -> send_msg_update_trap_position(id->get_value(), new_lat->get_value(), new_lon->get_value());
#endif
%}*/


_define_
TrapModel (Process _context, int _id, double _lat, double _lon, Process _ros_node)
{
    ros_node aka _ros_node

    TextPrinter tp

    Int id (_id)                    // ID as known by the Trap Manager
    String str_id (to_string(_id))

    // From operator:
    Bool hidden (false)             // whether the system should consider this trap

    // Detection step info:
    //Int detection_time (0)
    String detection_time ("..:..:..")
    String detection_robot_name ("VAB")
    Double lat (_lat)
    Double lon (_lon)
    Double altitude_msl (0)
    
    // Identification step info:
    Bool identified (false)         // whether the trap has been identified (i.e. QRCode read)
    //Int identification_time (0)
    String identification_time ("..:..:..")
    String identification_robot_name ("")

    // Int IDENTIFICATION_UNKNOWN (0)
    // Int IDENTIFICATION_AERIAL (1)
    // Int IDENTIFICATION_GROUND (2)
    Int identification_mode (0)     // which type of vehicle can read the external QRcode

    String nature ("")              // text describing the kind of trap
    String misc ("")                // description of an hazardous situation to take into account
    Int identifier (0)              // 4 digits

    Double radius (50)              // action radius [m] (50 is the maximum possible radius)
    Bool remote (false)             // whether the trap can be deactivated remotely
    String remote_code ("")         // code to deactivate the trap
    Bool contact (false)            // whether the trap can be deactivated through contact
    
    // Int CONTACT_UNKNOWN (10)
    // Int CONTACT_AERIAL (11)
    // Int CONTACT_GROUND (12)
    // Int CONTACT_GROUND_MULTIPLE (13)
    // Int CONTACT_AERIAL_AND_GROUND (14)
    // Int CONTACT_AERIAL_OR_GROUND (15)
    Int contact_mode (10)           // which type of satellite can deactivate; see enum
    String str_contact_mode ("")

    contact_mode -> (this) {
        this.str_contact_mode = get_contact_mode ($this.contact_mode)
    }

    // Deactivation step info:
    //Int deactivation_time (0)
    String deactivation_time ("..:..:..")
    Bool active (true)              // whether the trap is active

    // Int DEACTIVATION_UNKNOWN (20)
    // Int DEACTIVATION_REMOTE (21)
    // Int DEACTIVATION_LASER_AIR (22)
    // Int DEACTIVATION_LASER_GROUND (23)
    // Int DEACTIVATION_LASER_BOTH (24)
    // Int DEACTIVATION_PHYSICAL (25)
    Int deactivation_action (20)    //the way the trap has to be deactivated

    Int confirmation_mode (0)       // which type of vehicule can read the internal QRcode (cf. identification_mode)
    Bool confirmed (false)          // whether the internal code has been read
    String contact_code ("")        // deactivation code once confirmed

    Bool deleted (false)

    Bool is_selected (false)


    FSM fsm {
        State st_detected
        State st_identified
        State st_deactivated
        State st_hidden
        State st_deleted

        st_detected -> st_identified (identified.true)
        
        st_identified -> st_deactivated (active.false)
        st_deactivated -> st_identified (active.true)

        st_detected -> st_hidden (hidden.true)

        st_detected -> st_deleted (deleted.true)
        st_hidden -> st_deleted (deleted.true)
    }

    "Trap " + str_id + " (" + nature + "): " + fsm.state => tp.input


    // FIXME: OLD (Cohoma v1)

    //Bool remotely_deactivate (0)
    //Bool contact_deactivate (0)


    /*String contact_text ("")

    SwitchList switch_contact_mode (0) {
        Component zero {
            "unknown" =: contact_text
        }
        Component one {
            "Aerial" =: contact_text
        }
        Component two {
            "Ground" =: contact_text
        }
        Component three {
            "Ground Multiple" =: contact_text
        }
        Component four {
            "Aerial and Ground" =:contact_text
        }
        Component five {
            "Aerial or Ground" =: contact_text
        }
    }
    contact_mode + 1 =:> switch_contact_mode.index*/



    /*NativeAction na_update_trap_activation (change_activation_action, this, 1)
    
    AssignmentSequence unknown_assignement (1){
        1 =: active
        0 =: identified 
    }
    unknown_assignement -> na_update_trap_activation

    AssignmentSequence identified_assignement (1){
        1 =: active
        1 =: identified 
    }
    identified_assignement -> na_update_trap_activation     

    AssignmentSequence deactivated_assignement (1){
        0 =: active
        1 =: identified 
    }
    deactivated_assignement -> na_update_trap_activation*/


    NativeAction na_update_active_trap (action_update_active_trap, this, 1)

    AssignmentSequence deactivate (1) {
        _context.w_clock.state_text =: deactivation_time
        0 =: active
    }
    deactivate -> na_update_active_trap

    AssignmentSequence activate (1) {
        "..:..:.." =: deactivation_time
        1 =: active
    }
    activate -> na_update_active_trap


    NativeAction na_delete_trap (action_delete_trap, this, 1)

    // "delete" is a key word
    AssignmentSequence delete_ (1) {
        0 =: is_selected
        1 =: deleted
    }
    delete_ -> _context.set_selected_trap_to_null
    delete_ -> na_delete_trap


    /*NativeAction na_update_trap_position (update_trap_position_action, this, 1)

    FSM fsm_update_position {
        State idle

        State going_to_update {
            Timer t (5000)
        }
        idle -> going_to_update (lat)
        idle -> going_to_update (lon)
        going_to_update -> idle (fsm_update_position.going_to_update.t.end, na_update_trap_position)
    }*/
}