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
		std::cout << "TODO" << std::endl;
 		//node->send_validation_plan(); 
 	#endif
%}

_action_
update_graph_fun (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	#ifndef NO_ROS
  	if (node)
		std::cout << "TODO" << std::endl;
 		//node->send_msg_navgraph_update();  
 	#endif
%}

_action_
plan_request_fun (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	#ifndef NO_ROS
  	if (node)
		std::cout << "TODO" << std::endl;
 		//node->send_msg_planning_request(); 
 	#endif
%}

_action_
send_selected_tasks_fun (Process c)
%{

	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	#ifndef NO_ROS
  	if (node)
 		std::cout << "TODO" << std::endl;
		//node->send_selected_tasks();
 	#endif
%}

_action_ 
send_chat_message_fun (Process c)
%{

	std::string text = "test de chat";
	int type = 1;
	double lat = 4;
	double lng = 1.5;
	double alt = 0;
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	#ifndef NO_ROS
  	if (node)
		node->send_msg_chat(text, type, lat, lng, alt);
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
	Spike send_selected_tasks

	// ROS node
	RosNode node (map, context, model_manager)

  	NativeAction validate_plan_action (validate_plan_fun, node, 1)
  	validate_plan -> validate_plan_action

  	NativeAction update_graph_action(update_graph_fun, node, 1)
  	update_graph -> update_graph_action

  	NativeAction plan_request_action (plan_request_fun, node, 1)
  	plan_request -> plan_request_action

  	NativeAction send_selected_tasks_action(send_selected_tasks_fun, node, 1)
  	send_selected_tasks -> send_selected_tasks_action

	NativeAction send_chat_message_action(send_chat_message_fun, node, 1)
  	context.ros_test -> send_chat_message_action
}