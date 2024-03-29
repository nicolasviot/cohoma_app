use core
use gui
use base

import behavior.DraggableItemWithRadius

_native_code_
%{
    #include <iostream>
%}

_define_
SafetyPilot (Process map, Process _context, Process _model, Process _svg)
{
    //context aka _context
    model aka _model
    
    Translation screen_translation (0, 0)
    
    Component bg {
        NoOutline _
        FillColor _ ($model.color)
        FillOpacity _ (0.2)
        Circle c (0, 0, 50)
    }

    FillColor _ (#000000)

    icon << clone (_svg.icon)
    picking aka icon.picking

    /*Switch switch_type (uav){
        Component uav {
        }
        Component ugv {
        }
    }
    model.type =:> switch_type.state*/

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItemWithRadius draggable_item (map, model.lat, model.lon, model.radius, screen_translation.tx, screen_translation.ty, picking, _context.frame_released, bg.c.r)

}