use core 
use gui
use display
use base
use animation

import graph.OldEdge

_define_
OldTaskEdge (Process map, int _source, int _dest, Process nodes)
{
	Int id_source(_source)
	Int id_dest(_dest)
    Double length(0)
    Double explored(0)

    Double x1(0)
    Double x2(0)
    Double y1(0)
    Double y2(0)
   
    Bool selected (0)

    Spike press
    FSM ctrl_edge_selected { 
    
        State not_select{
            0 =: selected
        }
        State select { 
            /*OldEdge the_surrounding_edge($id_source, $id_dest, 20, nodes)
            255 =: the_surrounding_edge.color.r
            255 =: the_surrounding_edge.color.g
            0 =: the_surrounding_edge.color.b
            30 =: the_surrounding_edge.width.width*/

            OutlineColor yellow (255, 255, 0)
            OutlineWidth width(30)
            OutlineCapStyle capstyle(1)
            Line the_surrounding_edge(0, 0, 0, 0)
            x1 =:> the_surrounding_edge.x1
            x2 =:> the_surrounding_edge.x2
            y1 =:> the_surrounding_edge.y1
            y2 =:> the_surrounding_edge.y2
            the_surrounding_edge.press -> press
            1 =: selected
        }
        select -> not_select (press)
        not_select -> select (press)

    }

    OldEdge the_edge($id_source, $id_dest, 20, nodes)
    the_edge.mask_edge.press -> press
    length =:> the_edge.length
    explored =:> the_edge.explored

    220 =: the_edge.outline_color.r
    20 =: the_edge.outline_color.g
    20 =: the_edge.outline_color.b

    the_edge.edge.x1 =:> x1
    the_edge.edge.x2 =:> x2
    the_edge.edge.y1 =:> y1
    the_edge.edge.y2 =:> y2	
}