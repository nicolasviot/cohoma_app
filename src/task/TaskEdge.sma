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
TaskEdge (Process _context, Process _model, int _width) inherits Edge (_context, _model.edge, _width)
{
    print ("View of task edge: " + _model.edge.node1.id + " -- " + _model.edge.node2.id + " (" + _model.edge.length_meters + ") explored " + _model.explored_percent + "\n")

    _model.edge.length_meters + ". " + _model.explored_percent =:> this.text_tooltip

    FSM fsm_selection { 
    
        State not_selected {
            0 =: _model.is_selected

            0.0 =: this.bg.opacity.a
        }

        State selected {
            1 =: _model.is_selected

            1.0 =: this.bg.opacity.a
        }
        selected -> not_selected (this.mask_release)
        not_selected -> selected (this.mask_release)
    }

    _context.TASK_SELECTION_COLOR =: this.bg.color.value
}