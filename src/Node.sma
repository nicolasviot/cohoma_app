use core
use gui
use base
import GraphNode

_define_
Node(Process map, Process f, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id, Process _manager_id){


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

manager aka _manager_id
GraphNode wpt(map, f,  $lat, $lon, 80, 80, 80)
isPPO?"mandatory":"default" =:wpt.usage_status
islocked =:> wpt.islocked
id - 1 =:> wpt.id
wpt.usage_status => status
lat =:> wpt.lat
lon =:> wpt.lon
label =:> wpt.label
AssignmentSequence export_id (1){
	id =: manager.selected_id
	this =: manager.current_wpt
} 
wpt.interact_mask.press -> export_id
wpt.interact_mask.enter -> {wpt =: manager.entered_wpt}

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
				Rectangle rec($wpt.screen_translation.tx, $wpt.screen_translation.ty, 50, 50)
				wpt.screen_translation.tx =:> rec.x
				wpt.screen_translation.ty =:> rec.y
				NoFill _
				FillColor text_color(130, 130, 0)
				Text txt($wpt.screen_translation.tx, $wpt.screen_translation.ty, "test")
				wpt.screen_translation.tx =:> txt.x
				wpt.screen_translation.ty =:> txt.y
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