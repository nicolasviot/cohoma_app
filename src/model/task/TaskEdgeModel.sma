use core
use gui
use base

import model.EdgeModel

_native_code_
%{
    #include <iostream>
%}


_define_
TaskEdgeModel (Process _edge, double _explored)
{
	edge aka _edge

	Double explored (_explored)

	DoubleFormatter df_explored (100 * _explored, 2)
	100 * explored => df_explored.input

	String explored_percent ("")
	df_explored.output + " %" =:> explored_percent

	Bool is_selected (0)

	print ("Model of task for edge: " + _edge.node1.id + " -- " + _edge.node2.id + " (" + _edge.length_meters + ") explored = " + explored_percent + "\n")
}