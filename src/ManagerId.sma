use core
use gui
use base

_define_
ManagerId (int _selected){
	LogPrinter lp ("debug in ManagerId")
	LogPrinter lp_enter ("debug in ManagerId (entered)")
	
	Int selected_id(_selected)
	Ref current_wpt(0)
	Ref entered_wpt (0)

	selected_id =:> lp.input
}
