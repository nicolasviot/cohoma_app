use core
use gui
use base

import ros_node

_action_
validate_plan_fun (Process c)
%{
  ((RosNode*)c->find_child("this"))->send_validation_plan(); 
%}

_action_
update_graph_fun (Process c)
%{
  ((RosNode*)c->find_child("this"))->send_msg_navgraph_update(); 
%}

_action_
plan_request_fun (Process c)
%{
  ((RosNode*)c->find_child("this"))->send_msg_planning_request(); 
%}


_define_
RosManager (Process _parent, Process _map, Process _manager){

	parent aka _parent
	map aka _map
	manager aka _manager

	Spike update_graph
	Spike plan_request
	Spike validate_plan

	RosNode node(map, manager) 
  	NativeAction validate_plan_action (validate_plan_fun, this, 1)
  	validate_plan -> validate_plan_action

  	NativeAction update_graph_action(update_graph_fun, this, 1)
  	update_graph -> update_graph_action

  	NativeAction plan_request_action (plan_request_fun, this, 1)
  	plan_request -> plan_request_action


}