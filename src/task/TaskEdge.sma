use core 
use gui
use display
use base

import graph.Edge

_native_code_
%{
    #include <iostream>
%}


_define_
TaskEdge (Process _context, Process _model) inherits Edge (_context, _model.edge)
{
    //context aka _context
    //model aka _model
   
    //print ("View of task edge: " + _model.node1.id + " -- " + _model.node2.id + " (" + _model.length_meters + ") explored " + _model.explored_percent + "\n")

    _model.edge.length_meters + ". " + _model.explored_percent =:> this.text_tooltip

    FSM fsm_selection { 
    
        State not_selected {
            0 =: _model.is_selected

            //0.0 =: this.bg.opacity.a
        }

        State selected {
            OutlineCapStyle _ (1)            
            OutlineColor color ($_context.TASK_SELECTION_COLOR)
            //OutlineOpacity opacity (1.0)
            OutlineWidth width (24)

            Line selection (0, 0, 0, 0)

            _model.edge.node1.dx_in_map =:> selection.x1
            _model.edge.node1.dy_in_map =:> selection.y1

            _model.edge.node2.dx_in_map =:> selection.x2
            _model.edge.node2.dy_in_map =:> selection.y2

            1 =: _model.is_selected

            //1.0 =: this.bg.opacity.a
        }
        selected -> not_selected (this.mask_release)
        not_selected -> selected (this.mask_release)
    }

    //_context.TASK_SELECTION_COLOR =: this.bg.color.value

    // Set fg in background, behind the tooltip
    moveChild fsm_selection <<
}