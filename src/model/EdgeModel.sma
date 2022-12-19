use core
use gui
use base

_native_code_
%{
    #include <iostream>
	#include <math.h>
%}


_define_
EdgeModel (Process _node1, Process _node2, double _length)
{
	node1 aka _node1
	node2 aka _node2

	Double length (_length)

	DoubleFormatter df_length (_length, 2)

	String length_meters ("")
	df_length.output + " m" =: length_meters

	//print ("Model of edge: " + _node1.id + " -- " + _node2.id + " (" + length + " --> " + length_meters + ")\n")

}