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
        OutlineColor _ ($model.color)
        OutlineOpacity _ (0.7)
        NoFill _
        Circle c (0, 0, 50)
    }

    FillColor _ ($model.color)

    Scaling iconScale (1,1, 0,0)
    icon << clone (_svg.icon)
    picking aka icon.picking

    $model.color =: icon.marker.fill.value

    10 / map.zoomLevel =:> iconScale.sx 
    iconScale.sx =:> iconScale.sy  

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