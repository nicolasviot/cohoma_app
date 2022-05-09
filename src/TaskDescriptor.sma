use core
use base
use gui
use display
import CheckBox




_define_
TaskDescriptor (Process arg, double x, double y, string init_state){

	Translation t (x, y)
	FillColor fc(180, 30, 30)
	FillOpacity fo(0.5)

	//Rectangle icon (0, 0, 40, 20)
	CheckBox cb("title", 0, 0)

	Switch descriptor(area){
		Component area{
			Rectangle _(60, 0, 30, 30)
		}
		Component edge{
			OutlineWidth _(5)
			OutlineColor _(230, 10, 10)
			Line _ (60, 15, 90, 15)
		}
		Component trap_identification{
			Text _ (60, 15, "?")
			Rotation _ (45, 70, 10)
			Rectangle _(60, 0, 20, 20)
			
			
		}

		Component trap_deactivation{
			Text _ (60, 15, "!")
			Rotation _ (45, 70, 10)
			Rectangle _(60, 0, 20, 20)
			
			
		}
	}
	$init_state =: descriptor.state

}