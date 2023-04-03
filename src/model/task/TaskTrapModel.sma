use core
use gui
use base

import model.trap.TrapModel

_native_code_
%{
    #include <iostream>
%}


_define_
TaskTrapModel (Process _trap)
{
    trap aka _trap

    /*
    builtin_interfaces/Time stamp       # identification stamp
    uint8 robot_id                      # Robot ID, see RobotState.msg
    geographic_msgs/GeoPoint location   # location
    string id                           # 4 digits
    string description                  # text describing the kind of trap
    float32 radius                      # action radius [m]
    bool remotely_deactivate            # whether the trap can be deactivated remotely
    bool contact_deactivate             # whether the trap can be deactivated through contact
    int8 contact_mode                   # which type of satellite can deactivate; see enum
    string code                         # code to deactivate the trap
    string hazard                       # description of an hazardous situation to take into account
    */

    Bool is_selected (0)

    print ("Model of task for trap: " + _trap.id + "\n")
}