use core 
use gui
use display
use base

import behavior.DraggableItemWithRadius

_native_code_
%{
    #include <iostream>
%}


_define_
TaskTrap (Process _map, Process _context, Process _model)
{
    //map aka _map
    context aka _context
    model aka _model

    Translation screen_translation (0, 0)
    
    NoOutline _

    FillColor fill_col ($_context.TRAP_COLOR)

    Component losange {
        Rotation rot (45, 0, 0)
        Rectangle rect (-10, -10, 20, 20)
    }
    // for interactions
    picking aka losange.rect

    OutlineWidth outline_w (0)
    OutlineColor outline_col (255, 255, 0)
    
    FillOpacity fill_op (0.3)
    Circle c (0, 0, 50)


    FSM fsm_selection { 
    
        State not_selected {
            0 =: _model.is_selected

            0 =: outline_w.width
            #000000 =: outline_col.value
        }

        State selected {
            1 =: _model.is_selected

            5 =: outline_w.width
            _context.TASK_SELECTION_COLOR =: outline_col.value
        }
        selected -> not_selected (picking.release)
        not_selected -> selected (picking.release)
    }


    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    // Allow to drag via "picking"
    DraggableItemWithRadius draggable_item (_map, _model.trap.lat, _model.trap.lon, _model.trap.radius, screen_translation.tx, screen_translation.ty, picking, c.r)
	
}