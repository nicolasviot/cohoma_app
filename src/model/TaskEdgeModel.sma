use core
use gui
use base

import EdgeModel

_native_code_
%{
    #include <iostream>
%}


_define_
TaskEdgeModel (Process _node1, Process _node2, double _length, double _explored) inherits EdgeModel (_node1, _node2, _length)
{
	Double explored (_explored)

	Bool is_selected (0)

	print ("Model of task edge: " + _node1.id + " -- " + _node2.id + " (" + this.length + " m) explored = " + explored + "\n")
}