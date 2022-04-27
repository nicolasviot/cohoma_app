use core
use gui
use base

import ros_node

_action_
validate_plan_fun (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
 	
  	#ifndef NO_ROS
  if (node)
 	 node->send_validation_plan(); 
 	#endif
%}

_action_
update_graph_fun (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  
  	#ifndef NO_ROS
  if (node)
 	 node->send_msg_navgraph_update();  
 	#endif
%}

_action_
plan_request_fun (Process c)
%{

	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	
  	#ifndef NO_ROS
  if (node)
 	 node->send_msg_planning_request(); 
 	#endif
%}

_action_
send_selected_allocation_fun (Process c)
%{

	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	
  	#ifndef NO_ROS
  if (node)
 	 node->send_selected_tasks(); 
 	#endif
%}


_action_
test_multiple_itineraries (Process c)
%{

	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	node ->test_multiple_itineraries(); 
  	
%}

_define_
RosManager (Process _parent, Process _map, Process _manager){

	parent aka _parent
	map aka _map
	manager aka _manager

	Spike update_graph
	Spike plan_request
	Spike validate_plan
	Spike test_multiple_itineraries_spike
	Spike test_allocation_spike

	RosNode node(map, manager) 
  	NativeAction validate_plan_action (validate_plan_fun, node, 1)
  	validate_plan -> validate_plan_action

  	NativeAction update_graph_action(update_graph_fun, node, 1)
  	update_graph -> update_graph_action

  	NativeAction plan_request_action (plan_request_fun, node, 1)
  	plan_request -> plan_request_action

  	NativeAction test_multiple_itineraries_action(test_multiple_itineraries, node, 1)
  	test_multiple_itineraries_spike -> test_multiple_itineraries_action
  	NativeAction test_send_allocated_action (send_selected_allocation_fun, node, 1)
  	test_allocation_spike -> test_send_allocated_action
}