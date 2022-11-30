use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
EdgeModel (Process _node1, Process _node2, double _length)
{
	node1 aka _node1
	node2 aka _node2

	Double length (_length)

	//print ("Model of edge: " + _node1.id + " -- " + _node2.id + " (" + length + " m)\n")

}