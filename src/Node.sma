use core
use gui
use base
import GraphNode

_define_
Node (Process map, Process f, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id, Process _context)
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


	GraphNode wpt (map, f,  $lat, $lon, 80, 80, 80)

	isPPO ? "mandatory" : "default" =: wpt.usage_status

	islocked =:> wpt.islocked
	id - 1 =:> wpt.id
	wpt.usage_status => status
	lat =:> wpt.lat
	lon =:> wpt.lon
	label =:> wpt.label

	AssignmentSequence export_id (1) {
		id =: _context.selected_id
		this =: _context.current_wpt
	} 
	wpt.interact_mask.press -> export_id
	wpt.interact_mask.enter -> {
		wpt =: _context.entered_wpt
	}

}