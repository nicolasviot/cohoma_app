use core
use gui
use base
//use display

import ros_node

_native_code_
%{
    #include "cpp/coords-utils.h"
%}


_action_
validate_lima (Process c)
%{
    Process *data = (Process*) get_native_user_data(c);

    RosNode *ros_node = dynamic_cast<RosNode*>(data->find_child("ros_node"));
    IntProperty *id = dynamic_cast<IntProperty*>(data->find_child("id"));
    #ifndef NO_ROS
    ros_node ->send_msg_lima(id->get_value()); 
    #endif
%}


_define_
Lima (Process _map, Process _ros_node)
{
    //map aka _map
    ros_node aka _ros_node

    Int id(0)
    String name("")

    Spike send_msg_lima
    
    FillOpacity fo (0.8)
    NoFill _
    OutlineWidth outline_width(15)
    OutlineColor outline_color(20, 20, 250)
    OutlineCapStyle _ (1)

    Polyline lima {
        // IS_DEBUG
        Point p1 (300, 90)
        Point p2 (250, 190)
        Point p3 (350, 190)
    }

    FSM hovered_lima {
        State not_hovered {
            220 =: outline_color.b
            15 =: outline_width.width
        }
        State hovered {
            170 =:outline_color.b
            25 =: outline_width.width
        }
        not_hovered -> hovered (lima.enter)
        hovered -> not_hovered (lima.leave)
    }


    FSM lima_selection {
        State idle {
            190 =: outline_color.b
            20 =: outline_color.g
        }
        State first_click{
            20 =: outline_color.b
            190 =: outline_color.g

            Timer t (5000)

            FillColor _ (0, 0, 0)
            FontWeight _ (50)
            FontSize _ (5, 24)
            Text legend(0, 0, "")
            lima.press.x =: legend.x
            lima.press.y =: legend.y
            name =: legend.text
        }
        idle -> first_click (lima.press)
        first_click -> idle (first_click.t.end)
        first_click -> idle (lima.press, send_msg_lima)
    }

    NativeAction na_validate_lima (validate_lima, this, 1)
    send_msg_lima -> na_validate_lima

}