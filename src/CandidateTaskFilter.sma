use core 
use gui
use display
use base

import TaskDescriptor

import Button

_define_
CandidateTaskFilter (Process frame){

Translation _t(15, 20)

FillColor fc(164, 164, 164)
Rectangle bg (0, 0, 350, 250, 5, 5)
Component AreaTaskFilter{

	List tasks{
		TaskDescriptor _ (frame, 20, 0, "area")
		TaskDescriptor _ (frame, 20, 40, "area")
	}


}
Translation _t_2(100, 0)
Component SegmentTaskFilter{

	List tasks{
		TaskDescriptor _ (frame, 20, 0, "edge")
		
	}

}
Translation _t_3(100, 0)
Component TrapTaskFilter{


	List tasks{
		TaskDescriptor  _(frame, 20, 0, "trap_identification")
		TaskDescriptor _(frame, 20, 0, "trap_deactivation")		
	}
}

Spike send_selected_tasks
Translation _t_4(0, 250)
Button send_selected_tasks_but (frame, "send selection ", 0, 0)
send_selected_tasks_but.click -> send_selected_tasks
}