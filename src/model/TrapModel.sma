use core
use gui
use base

/*_native_code_
%{
    #include "cpp/coords-utils.h"
%}*/


/*_action_
change_activation_action (Process c)
%{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
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

    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
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
    RosNode *node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    DoubleProperty *new_lat = dynamic_cast<DoubleProperty*>(data->find_child("lat"));
    DoubleProperty *new_lon = dynamic_cast<DoubleProperty*>(data->find_child("lon")); 
#ifndef NO_ROS
    node -> send_msg_update_trap_position(id->get_value(), new_lat->get_value(), new_lon->get_value());
#endif
%}*/


_define_
TrapModel (Process _context, int _id, double _lat, double _lon) //, Process _ros_node)
{
    //ros_node aka _ros_node

    Int id (_id)
    String str_id ("?")

    Double lat (_lat)
    Double lon (_lon)
    Double altitude_msl(0)
    
    Bool active (1)
    Bool identified (0)
    Bool deleted (0)
    String state ("unknown") //can be unkown, identified, deactivated

    active ? (identified ? "identified" : "unknown") : "deactivated" =:> state


    /*
    string description                  # text describing the kind of trap
    float32 radius                      # action radius [m]
    bool remotely_deactivate            # whether the trap can be deactivated remotely
    bool contact_deactivate             # whether the trap can be deactivated through contact
    int8 contact_mode                   # which type of satellite can deactivate; see enum
    string code                         # code to deactivate the trap
    string hazard                       # description of an hazardous situation to take into account
    */
    String description ("..")
    
    Double radius (50) // Maximum possible radius

    Bool remotely_deactivate (0)
    Bool contact_deactivate (0)

    /*
    int8 CONTACT_UNKONWN = 0
    int8 CONTACT_AERIAL = 1
    int8 CONTACT_GROUND = 2
    int8 CONTACT_GROUND_MULTIPLE = 3
    int8 CONTACT_AERIAL_AND_GROUND = 4
    int8 CONTACT_AERIAL_OR_GROUND = 5
    */
    Int contact_mode (0)

    String contact_text ("")

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
    contact_mode + 1 =:> switch_contact_mode.index


    String code ("?")
    
    String hazard ("?")

}