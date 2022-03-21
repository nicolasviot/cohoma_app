use core
use base
use display
use gui

import ros_node

_define_
RosManager (Process _parent, Process _map, Process _manager){

	parent aka _parent
	map aka _map
	manager aka _manager

	RosNode node(map, manager) 




}