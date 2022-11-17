use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
NodeModel (Process _context, int _id, int _phase, string _label, double _lat, double _lon, double _alt, int _is_mandatory)
{
	//context aka _context

	Int id (_id)
	
	// Cross a LIMA --> change the phase
	Int phase (_phase)
	
	String label (_label)

	Double lat (_lat)
	Double lon (_lon)
	Double alt (_alt)

	Bool is_mandatory (0)

	//Bool is_PPO (_is_PPO) // Point de Passage Oblige
	//Bool is_POS (_is_POS) // (Point de) Passage Oblige pour Satellites

	//Bool is_locked (0) // FIXME: not used

	String status ("")

	print ("Model of node (" + id + ") '" + label + "': phase = " + phase + "\n")

	Double dx_map (0)
	Double dy_map (0)
}