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
ask_itinerary_fun (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	#ifndef NO_ROS
  	if (node)
		node->send_msg_itinerary_request(); 
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

_action_ 
send_group_config_fun (Process c)
%{
	Process *data = (Process*) get_native_user_data(c);
 	RosNode *node = dynamic_cast<RosNode*>(data);
  	#ifndef NO_ROS
  	if (node)
		node->send_msg_group_config();
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
	Spike ask_itinerary
	Spike validate_plan
	Spike send_selected_tasks

	// ROS node
	RosNode node (map, context, model_manager)

  	NativeAction validate_plan_action (validate_plan_fun, node, 1)
  	validate_plan -> validate_plan_action

  	NativeAction update_graph_action(update_graph_fun, node, 1)
  	update_graph -> update_graph_action

  	NativeAction ask_itinerary_action (ask_itinerary_fun, node, 1)
  	ask_itinerary -> ask_itinerary_action

  	NativeAction send_selected_tasks_action(send_selected_tasks_fun, node, 1)
  	send_selected_tasks -> send_selected_tasks_action

	NativeAction send_chat_message_action(send_chat_message_fun, node, 1)
  	context.ros_test -> send_chat_message_action

	NativeAction send_group_config_action(send_group_config_fun, node, 1)
  	model_manager.group_config_updated-> send_group_config_action


}