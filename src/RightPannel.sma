use core
use gui
use base

import Button
import ItineraryPannel
import CandidateTaskFilter


_define_
RightPannel (Process root, Process frame){


Translation _ (10, 0)
Spike plan_request 
Spike validate_plan  
Spike update_graph
Spike test_multiple_itineraries_spike
Spike test_allocation_spike
Spike test_lima_spike

     //legend for NavGraph
Component NavGraph
{
	nav_svg = loadFromXML ("res/svg/GraphNav_legend.svg")
	nav << nav_svg.GraphNav

    //TODO Use the buuton to send and updated graph via Ros
	

	button aka nav.update_button.update_bg

	FSM button_FSM {
		State idle{
			#666682 =: button.fill.value
		}
		State hover{
			#8080FF =: button.fill.value
		}
		State pressed{
			#1A1AFF =: button.fill.value
		}
		idle -> hover (button.enter)
		hover -> idle (button.leave)
		idle -> pressed (button.press)
		pressed -> idle (button.release, plan_request)
		hover -> pressed (button.press)
	}
}

Translation _(0, 220)
ItineraryPannel itineraryPannel(0, 0, root.l.map.layers.itineraries.id)
Translation _ (0, 320)


CandidateTaskFilter _(frame)

Translation _ (0, 100)
Component IObuttons{
	Button send_plan_req (frame, " request plan ", 20, 20)
	send_plan_req.click -> plan_request
	Button valid_plan (frame, " validate plan ", 100, 20)
	valid_plan.click -> validate_plan

	Button update_graph_but (frame, "send graph", 20, 150)
	update_graph_but.click -> update_graph

	Button test_multiple_itineraries_button (frame, "test multiple itineraries", 100, 150)
	test_multiple_itineraries_button.click -> test_multiple_itineraries_spike

	Button test_allocation_button(frame, "test allocation", 20, 250)
	test_allocation_button.click -> test_allocation_spike
	Button test_lima_button(frame, "test lima", 100, 250)
	test_lima_button.click -> test_lima_spike
}



}