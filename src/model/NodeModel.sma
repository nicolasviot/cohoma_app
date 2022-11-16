use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
//NodeModel (Process _context, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id)
NodeModel (Process _context, int _id, string _label, double _lat, double _lon, double _alt)
//NodeModel (Process _context, int _id, string _label, double _lat, double _lon, double _alt, int _is_PPO)
//NodeModel (Process _context, int _id, string _label, double _lat, double _lon, double _alt, int _is_PPO, int _phase)
{
	//context aka _context

	Int id (_id)
	String label (_label)
	Double lat (_lat)
	Double lon (_lon)
	Double alt (_alt)

	//Bool is_PPO (_is_PPO) // Point de Passage Obligatoire
	Bool is_mandatory (0)

	//Bool is_locked (0) // FIXME: not used
	//Int phase (_phase) // FIXME: not used

	String status ("")

	print ("Model of node (" + id + ") " + label + "\n")

	Double dx_map (0)
	Double dy_map (0)
}