use core
use gui
use base

import Button
_define_
GraphPannel (Process root, Process frame){


Translation _ (20, 20)
Spike plan_request 
Spike validate_plan  
Spike update_graph

Component Descripteur1 {

	//TODO replace with svg
	FillColor fc (190, 190, 190)	
	Rectangle bg (0, 0, 300, 200)
	FillColor _ (0, 0, 0)

	Text txt (50, 50, "Description du plan numéro un")

}




Translation _(0, 220)
Component Descripteur2 {
	FillColor fc (190, 190, 190)
	Rectangle bg (0, 0, 300, 200)
	FillColor _ (0, 0, 0)

	Text txt (50, 50, "Description du plan numéro deux")

}

Translation _(0, 220)
Component Descripteur3 {
	FillColor fc (190, 190, 190)
	Rectangle bg (0, 0, 300, 200)
	FillColor _ (0, 0, 0)
	Text txt (50, 50, "Description du plan numéro trois")

}
Translation _(0, 220)

Component IObuttons{
	Button send_plan_req (frame, " request plan ", 20, 20)
	send_plan_req.click -> plan_request
	Button valid_plan (frame, " validate plan ", 200, 20)
	valid_plan.click -> validate_plan

	Button update_graph_but (frame, "send graph", 20, 200)
	update_graph_but.click -> update_graph
}



}