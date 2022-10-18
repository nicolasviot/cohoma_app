use core
use gui
use base
use display
use animation

import task.TaskAreaSummit
import ros_node

_native_code_
%{
#include "cpp/coords-utils.h"
#include "core/utils/getset.h"
%}


_action_
validate_lima (Process c)
%{

    Process *data = (Process*) get_native_user_data(c);
    // GET_CHILD(RosNode, data, node);
    // GET_CHILD(djnn::IntProperty, data, id);
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
    Spike enter
    Spike leave
    node aka my_node
    FillOpacity fo (0.8)
    //FillColor fill_color(20, 20, 190)
    NoFill _
    OutlineWidth outline_width(15)
    OutlineColor outline_color(20, 20, 250)
    OutlineCapStyle _ (1) 
    FSM hovered_lima{
        State not_hovered{
            220 =: outline_color.b
            15 =: outline_width.width
        }
        State hovered{
            170 =:outline_color.b
            25 =: outline_width.width
        }
        not_hovered -> hovered(enter)
        hovered -> not_hovered(leave)
    }
/*    FSM tooltip{
        State idle
        State entered{
            Timer t (500)
        }
        State display_tooltip{
            Translation t(20, 0)

            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            
            Rectangle bg (0, 0, 50, 20, 5, 5)
            FillColor _ (0, 0, 0)
            Text legend (0, 0, "Node 0")
            legend.x =:> bg.x
            legend.y - 12 =:> bg.y
            legend.width =:> bg.width
            legend.height =:> bg.height
            "Node " + toString(id) + " " + usage_status =:> legend.text


        }
        idle -> entered (enter)
        entered -> display_tooltip(entered.t.end)
        entered -> idle (leave)
        display_tooltip -> idle (leave)
}*/


    Polyline lima {
        //Point p1 (300, 90)
        //Point p2 (250, 190)
        //Point p3 (350, 190)
    }
    lima.press -> pressed
    lima.enter -> enter
    lima.leave -> leave

    Spike send_msg_lima
    FSM lima_selection{
        State no_click{
            190 =: outline_color.b
            20 =: outline_color.g
        }
        State first_click{
            20 =: outline_color.b
            190 =: outline_color.g
            Timer t(5000)
            FillColor _ (0, 0, 0)
            FontWeight _ (50)
            FontSize _ (5, 24)
            Text legend(0, 0, "")
            lima.press.x =: legend.x
            lima.press.y =: legend.y
            name =: legend.text
        }
        no_click -> first_click (pressed)
        first_click -> no_click (first_click.t.end)
        first_click -> no_click (pressed, send_msg_lima)

    }

    NativeAction test_lima_action (validate_lima, this, 1)
    send_msg_lima -> test_lima_action


}