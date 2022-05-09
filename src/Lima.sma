use core
use gui
use base
use display
use animation

import gui.animation.Animator
import TaskAreaSummit
import ros_node
_native_code_
%{
#include "cpp/coords-utils.h"

%}


_action_
validate_lima (Process c)
%{

    Process *data = (Process*) get_native_user_data(c);
    RosNode *node = dynamic_cast<RosNode*>(data->find_child("node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    #ifndef NO_ROS
    node ->send_msg_lima(id->get_value()); 
    #endif
    
%}


_define_
Lima (Process map, Process my_node){

    Int id(0)
    String name("")
    Spike pressed
    node aka my_node
    FillOpacity fo (0.8)
    FillColor fill_color(20, 20, 190)
    OutlineWidth outline_width(15)
    OutlineColor outline_color(20, 20, 250)
    Polyline lima {
    }
    lima.press -> pressed

    NativeAction test_lima_action (validate_lima, this, 1)
    pressed -> test_lima_action


}