use core
use base
use gui
use display
import CheckBox




_define_
TaskDescriptor (Process arg, double x, double y, string text){

	Translation t (x, y)
	FillColor fc(180, 30, 30)
	FillOpacity fo(0.5)

	//Rectangle icon (0, 0, 40, 20)
	CheckBox cb("title", 0, 0)

	Switch descriptor(area){
		Component area{
			Rectangle _(45, 0, 30, 30)
		}
		Component edge{
			Line _ (45, 0, 75, 0)
		}
		Component trap_identification{
			Rotation _ (45, 60, 15)
			Rectangle _(45, 0, 30, 30)
			Text _ (45, 0, "?")
			
		}

		Component trap_deactivation{
			Rotation _ (45, 60, 15)
			Rectangle _(45, 0, 30, 30)
			Text _ (45, 0, "!")
			
		}
	}

}