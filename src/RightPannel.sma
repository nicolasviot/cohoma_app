use core
use gui
use base

import Button
import ItineraryPannel
import CandidateTaskFilter


_define_
RightPannel (Process root, Process frame){


Translation _ (20, 20)
Spike plan_request 
Spike validate_plan  
Spike update_graph
Spike test_multiple_itineraries_spike

     //legend for NavGraph
Component NavGraph
{
	nav_svg = loadFromXML ("res/svg/GraphNav_legend.svg")
	//Translation legend (350, 400)
	nav << nav_svg.GraphNav
	//Translation legend_off (-350, -400)

    //TODO Use the buuton to send and updated graph via Ros
	nav.update_button.rect.press -> plan_request
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

}



}