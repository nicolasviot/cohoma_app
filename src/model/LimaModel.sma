use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
LimaModel (int _id, string _name) // int _phase)
{
	Int id (_id)
	String name (_name)

	// Cross a LIMA --> change the phase
	//Int phase (_phase)

	List points
	
	//print ("Model of Lima (" + id + ") '" + name + "': phase = " + phase + "\n")
	print ("Model of Lima (" + id + ") '" + name + "'\n")
}