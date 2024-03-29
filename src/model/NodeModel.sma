use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
NodeModel (string _id, int _phase, string _label, double _lat, double _lon, double _alt, int _is_mandatory)
{
	String id (_id)
	
	// Cross a LIMA --> change the phase
	Int phase (_phase)
	
	String label (_label)
	Bool is_empty_label (0)
	if (_label == "") {
		//1 =: is_empty_label
		is_empty_label = 1
	}
	// FIXME: if label can be updated
	//TextComparator tc (_label, "")
	//label =:> tc.left
	//is_empty_label aka tc.output

	Double lat (_lat)
	Double lon (_lon)
	Double alt (_alt)

	//Bool is_PPO (_is_PPO) // Point de Passage Oblige pour le VAB
	//Bool is_POS (_is_POS) // (Point de) Passage Oblige pour tous les satellites terrestres
	
	// "Mandatory" (= "Compulsory" in ROS msg)
	Bool is_mandatory (_is_mandatory)

	// "Forced" (= "Locked" in ROS msg)
	Bool is_forced (0)

	// { default, start, end, forced, mandatory }
	String status ("")

    is_mandatory ? "mandatory" : "default" =: status

	//print ("Model of node (" + id + ") '" + label + "': phase = " + phase + "\n")

	Double dx_in_map (0)
	Double dy_in_map (0)
}