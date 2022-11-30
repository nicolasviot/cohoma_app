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
test_lima (Process c)
%{

	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
	#ifndef NO_ROS
  	node ->send_msg_lima(1); 
	#endif
  	
%}

_action_
send_selected_tasks_native (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
	RosNode *node = dynamic_cast<RosNode*>(data);
	#ifndef NO_ROS
	node->send_selected_tasks();
	#endif
%}


_action_
test_visibility_map_native (Process c)
%{
	#ifndef NO_ROS
		Process *data = (Process*) get_native_user_data(c);
		RosNode *node = dynamic_cast<RosNode*>(data);
		node->test_draw_visibility_map();
	#endif
%}



_define_
RosManager (Process _parent, Process _map, Process _context, Process _model_manager)
{
	parent aka _parent
	map aka _map
	context aka _context
	model_manager aka _model_manager

	Spike update_graph
	Spike plan_request
	Spike validate_plan

	Spike test_allocation_spike
	Spike test_lima_spike
	Spike send_selected_tasks
	Spike test_visibility_map

	RosNode node (map, context, model_manager)

  	NativeAction validate_plan_action (validate_plan_fun, node, 1)
  	validate_plan -> validate_plan_action

  	NativeAction update_graph_action(update_graph_fun, node, 1)
  	update_graph -> update_graph_action

  	NativeAction plan_request_action (plan_request_fun, node, 1)
  	plan_request -> plan_request_action

  	NativeAction test_send_allocated_action (send_selected_allocation_fun, node, 1)
  	test_allocation_spike -> test_send_allocated_action

  	NativeAction test_lima_action (test_lima, node, 1)
  	test_lima_spike -> test_lima_action

  	NativeAction send_selected_tasks_action(send_selected_tasks_native, node, 1)
  	send_selected_tasks -> send_selected_tasks_action
	
  	NativeAction test_visibility_map_action(test_visibility_map_native, node, 1)
  	test_visibility_map -> test_visibility_map_action
}