use core
use gui
use base

import ros_node

_native_code_
%{
    #include <iostream>
	using namespace std;
%}


_action_
action_validate_lima (Process c)
%{
    Process *self = (Process*) get_native_user_data(c);

    RosNode *node = dynamic_cast<RosNode*>(self->find_child("ros_node"));
    IntProperty *id = static_cast<IntProperty*>(self->find_child("id"));
	
	static_cast<BoolProperty*>(self->find_child("is_validated"))->set_value(true, true);

	//cout << "action_validate_lima " << id->get_value() << endl;

    #ifndef NO_ROS
    node->send_msg_lima(id->get_value()); 
    #endif
%}


_define_
LimaModel (int _id, string _name, Process _ros_node) // int _phase)
{
	Int id (_id)
	String name (_name)

	ros_node aka _ros_node

	// Cross a LIMA --> change the phase
	//Int phase (_phase)

	Bool is_validated (false)

	List points
	
	//print ("Model of Lima (" + id + ") '" + name + "': phase = " + phase + "\n")
	print ("Model of Lima (" + id + ") '" + name + "'\n")

	NativeAction na_validate_lima (action_validate_lima, this, 1)
}