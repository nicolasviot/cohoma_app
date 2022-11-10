use core 
use gui
use display
use base
use animation

import behavior.DraggableItemWithRadius
//import gui.animation.Animator

_native_code_
%{
    #include <iostream>
    #include "cpp/coords-utils.h"
%}



_define_
TaskTrap (Process _map, Process _context, Process _model)
{
    //map aka _map
    context aka _context
    model aka _model

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

    Bool selected (0)

    Spike toggle_select


    Translation screen_translation (0, 0)
    
    NoOutline _

    FillColor red (240, 0, 0)

    Component losange {
        Rotation rot (45, 0, 0)
        Rectangle rect (-10, -10, 20, 20)
    }
    // for interactions
    picking aka losange.rect

    OutlineWidth circle_perimeter_width (0)
    OutlineColor yellow (255, 255, 0)
    
    Switch ctrl_trap_selected (not_select) {
    
        Component select { 
            5 =: circle_perimeter_width.width
            255 =: yellow.r
            255 =: yellow.g
            0 =: yellow.b
        }
        Component not_select{
            0 =: circle_perimeter_width.width
            0 =: yellow.r
            0 =: yellow.g
            0 =: yellow.b
        }
   }
   selected ? "select" : "not_select" => ctrl_trap_selected.state


    FillOpacity circle_opacity (0.3)
    Circle c (0, 0, 50)


    picking.press -> toggle_select
    
    //0 =: c.pickable
    //c.press -> toggle_select
    
    toggle_select ->{
        selected ? 0 : 1 =: selected
    }

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItemWithRadius draggable_item (_map, _model.lat, _model.lon, _model.radius, screen_translation.tx, screen_translation.ty, picking, c.r)
	
}