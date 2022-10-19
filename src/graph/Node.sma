use core
use gui
use base
import GraphNode

_define_
Node (Process map, Process _context, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id)
{
	//context aka _context

	Double lat(_lat)
	Double lon(_lon)
	Double alt(_alt)
	Bool isPPO(_isPPO)
	Bool islocked(0)
	String label(_label)
	String status ("")
	Int id(_id)
	Spike pressed
	Int phase(0)


	GraphNode wpt (map, _context, _id, _lat, _lon, 80, 80, 80)
	id =:> wpt.id

	isPPO ? "mandatory" : "default" =: wpt.usage_status
	wpt.usage_status => status

	islocked =:> wpt.islocked
	
	lat =:> wpt.lat
	lon =:> wpt.lon
	label =:> wpt.label

}