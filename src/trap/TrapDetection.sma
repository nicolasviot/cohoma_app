use core
use gui
use base

import behavior.NotDraggableItem

_native_code_
%{
    #include <iostream>
%}


_define_
TrapDetection (Process _map, Process _context, Process _model)
{
    //map aka _map
    context aka _context
    model aka _model

    TextPrinter tp

    Translation screen_translation (0, 0)

    Component losange {
        Rotation rot (45, 0, 0)

        Component bg {
            //OutlineOpacity o_op (0.0)
            //OutlineColor o_col (#000000)
            //OutlineWidth o_w (1)
            NoOutline _

            FillOpacity f_op (0.6)
            FillColor f_col ($_context.TRAP_DETECTION_COLOR)
            Rectangle rect (-$_context.TRAP_DETECTION_SIZE / 2, -$_context.TRAP_DETECTION_SIZE / 2, $_context.TRAP_DETECTION_SIZE, $_context.TRAP_DETECTION_SIZE, 2, 2)
        }

        /*Component fg {
            OutlineOpacity o_op (0.0)
            _model.is_selected ? 1.0 : 0.0 =:> o_op.a

            OutlineColor o_col ($_context.SELECTION_COLOR)
            OutlineWidth o_w (4)
            NoFill _
            Rectangle feedback (-$_context.TRAP_DETECTION_SIZE / 2, -$_context.TRAP_DETECTION_SIZE / 2, $_context.TRAP_DETECTION_SIZE, $_context.TRAP_DETECTION_SIZE)
        }*/
    }

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    NotDraggableItem not_draggable_item (_map, _model.lat, _model.lon, screen_translation.tx, screen_translation.ty)
    
}