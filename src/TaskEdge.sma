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


    Bool selected (0)

    Switch ctrl_edge_selected(not_select){ 
    
    Component select { 
        Edge the_surrounding_edge($id_source, $id_dest, 20, nodes)
        255 =: the_surrounding_edge.color.r
        255 =: the_surrounding_edge.color.g
        0 =: the_surrounding_edge.color.b
    	8 =: the_surrounding_edge.width.width
    }
    Component not_select{
    }

   }

   selected?"select":"not_select" => ctrl_edge_selected.state


   Edge the_edge($id_source, $id_dest, 20, nodes)
   the_edge.line.press ->{
   	selected?0:1 =: selected 
   }




	
}