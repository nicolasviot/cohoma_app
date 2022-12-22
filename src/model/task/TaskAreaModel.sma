use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
TaskAreaModel (double _area, double _explored)
{
	Double area (_area) // FIXME ???

	Double explored (_explored)

	DoubleFormatter df_explored (100 * _explored, 2)
	100 * explored => df_explored.input

	String explored_percent ("")
	df_explored.output + " %" =:> explored_percent

	List points

	Bool is_selected (0)

	print ("Model of task for area: (" + area + ") explored = " + explored_percent + "\n")
}