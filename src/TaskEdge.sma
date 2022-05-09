use core 
use gui
use display
use base
use animation

import Edge

_define_
TaskEdge (Process map, int _source, int _dest, Process nodes){

	Int id_source(_source)
	Int id_dest(_dest)
    Double length(0)
    Double explored(0)

    Double x1(0)
    Double x2 (0)
    Double y1(0)
    Double y2(0)

    Bool selected (0)

    Spike press
    FSM ctrl_edge_selected{ 
    
    State select { 
/*        Edge the_surrounding_edge($id_source, $id_dest, 20, nodes)
        255 =: the_surrounding_edge.color.r
        255 =: the_surrounding_edge.color.g
        0 =: the_surrounding_edge.color.b
    	30 =: the_surrounding_edge.width.width*/

        Translation pos(0, 0)
        OutlineColor yellow (255, 255, 0)
        OutlineWidth _(30)
        Line the_surrounding_edge(0, 0, 0, 0)
        x1 =:> the_surrounding_edge.x1
        x2 =:> the_surrounding_edge.x2
        y1 =:> the_surrounding_edge.y1
        y2 =:> the_surrounding_edge.y2
        the_surrounding_edge.press -> press
    }
    State not_select{
    }
    select -> not_select (press)
    not_select -> select (press)

   }

    Edge the_edge($id_source, $id_dest, 20, nodes)
    the_edge.outerEdge.press -> press
    the_edge.pos.tx =:> ctrl_edge_selected.select.pos.tx
    the_edge.pos.ty =:> ctrl_edge_selected.select.pos.ty
   220 =: the_edge.color.r
   20 =: the_edge.color.g
   20 =: the_edge.color.b

   the_edge.edge.x1 =:> x1
   the_edge.edge.x2 =:> x2
   the_edge.edge.y1 =:> y1
   the_edge.edge.y2 =:> y2





	
}