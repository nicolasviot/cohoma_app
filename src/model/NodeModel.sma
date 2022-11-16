use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
//NodeModel (Process _context, int _id, string _label, double _lat, double _lon, double _alt, int _isPPO)
NodeModel (Process _context, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id)
{
	//context aka _context

	Int id (_id)
	String label (_label)
	Double lat (_lat)
	Double lon (_lon)
	Double alt (_alt)

	Bool isPPO (_isPPO) // Point de Passage Obligatoire

	Bool islocked (0) // FIXME: not used
	
	String status ("")

	Int phase (0) // FIXME: not used

	print ("Model of node (" + id + ") " + label + "\n")

}