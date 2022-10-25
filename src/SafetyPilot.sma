use core
use gui
use base
use animation

import gui.animation.Animator
import behavior.DraggableItem

_native_code_
%{
    #include "cpp/coords-utils.h"

%}

_define_
SafetyPilot (Process map, Process _context, Process _model, Process _svg)
{
    //APP-6A: Symbologie militaire interarmées de l'OTAN

    //context aka _context
    model aka _model
    
    Translation screen_translation (0, 0)
    
    NoOutline _
    FillColor _ ($model.color)
    FillOpacity _ (0.3)
    Circle c (0, 0, 50)

    FillColor _ (0, 0, 0)
    FillOpacity _ (3.3) // 0.3 * 3.3 = 0.99 (opacity = 100%)

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
    DraggableItem draggable_item (map, model.lat, model.lon, model.radius, screen_translation.tx, screen_translation.ty, picking, c.r)

}