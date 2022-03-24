use core
use gui
use base
import Waypoints

_define_
Node(Process map, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id, Process _manager_id){


Double lat(_lat)
Double lon(_lon)
Double alt(_alt)
Bool isPPO(_isPPO)
String label(_label)
String status ("")
Int id(_id)
Spike pressed

manager aka _manager_id
Waypoints wpt(map, $lat, $lon, 80, 80, 80)
status => wpt.usage_status
lat =:> wpt.lat
lon =:> wpt.lon

AssignmentSequence export_id (1){
	id =: manager.selected_id
	this =: manager.current_wpt
} 
wpt.c.press -> export_id
wpt.c.enter -> {wpt =: manager.entered_wpt}

LogPrinter lp ("debug enter/leave")

Switch tooltip_switch(off){
	Component off{

	}
	Component on{
		FSM tooltip {
			State idle
			State entering{
				Timer t(1000)
				"entered" =: lp.input
			}
			State display_tooltip{
				Timer t2(3000)
				Translation t ($wpt.pos.tx, $wpt.pos.ty)
				wpt.pos.tx =:> t.tx
				wpt.pos.ty =:> t.ty

				FillColor black(0, 0, 0)
				Rectangle rec($wpt.c.cx, $wpt.c.cy, 50, 50)
				wpt.c.cx =:> rec.x
				wpt.c.cy =:> rec.y
				NoFill _
				FillColor text_color(130, 130, 0)
				Text txt($wpt.c.cx, $wpt.c.cy, "test")
				wpt.c.cx =:> txt.x
				wpt.c.cy =:> txt.y
			}
			idle -> entering (wpt.c.enter)
			entering -> display_tooltip (entering.t.end)
			entering -> idle (wpt.c.leave)
			display_tooltip -> idle (wpt.c.leave)
			display_tooltip -> idle (display_tooltip.t2.end)
		}
	}
}

}