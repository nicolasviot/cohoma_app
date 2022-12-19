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
TaskEdge (Process _context, Process _model) inherits Edge (_context, _model)
{
    //context aka _context
    //model aka _model

    DoubleFormatter df(0, 2)
    100 * _model.explored =:> df.input 
   
    print ("View of task edge: " + _model.node1.id + " -- " + _model.node2.id + " (" + _model.length + " m) explored " + df.output + "%\n")


    FSM ctrl_edge_selected { 
    
        State not_selected {
            0 =: _model.is_selected
        }
        
        State selected {
            /*OutlineColor yellow (255, 255, 0)
            OutlineWidth width (30)
            OutlineCapStyle capstyle(1)
            Line the_surrounding_edge(0, 0, 0, 0)
            x1 =:> the_surrounding_edge.x1
            x2 =:> the_surrounding_edge.x2
            y1 =:> the_surrounding_edge.y1
            y2 =:> the_surrounding_edge.y2
            the_surrounding_edge.press -> press*/
            
            1 =: _model.is_selected
        }
        selected -> not_selected (this.mask_release)
        not_selected -> selected (this.mask_release)
    }

    /*OldEdge the_edge($id_source, $id_dest, 20, nodes)
    the_edge.mask_edge.press -> press
    length =:> the_edge.length
    explored =:> the_edge.explored

    220 =: the_edge.outline_color.r
    20 =: the_edge.outline_color.g
    20 =: the_edge.outline_color.b

    the_edge.edge.x1 =:> x1
    the_edge.edge.x2 =:> x2
    the_edge.edge.y1 =:> y1
    the_edge.edge.y2 =:> y2*/

}