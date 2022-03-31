use core
use gui
use base

import Button
import ItineraryPannel

_define_
RightPannel (Process root, Process frame){


Translation _ (20, 20)
Spike plan_request 
Spike validate_plan  
Spike update_graph
Spike test_multiple_itineraries_spike



ItineraryPannel itineraryPannel(0, 0)
Translation _ (0, 500)


Component IObuttons{
	Button send_plan_req (frame, " request plan ", 20, 20)
	send_plan_req.click -> plan_request
	Button valid_plan (frame, " validate plan ", 200, 20)
	valid_plan.click -> validate_plan

	Button update_graph_but (frame, "send graph", 20, 200)
	update_graph_but.click -> update_graph

	Button test_multiple_itineraries_button (frame, "test multiple itineraries", 200, 200)
	test_multiple_itineraries_button.click -> test_multiple_itineraries_spike

}



}