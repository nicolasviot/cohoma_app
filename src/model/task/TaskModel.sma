use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
TaskModel ()
{
	// Selected by the user
	Bool is_selected (0)

	// Assigned to a satellite
	Bool is_assigned (0)
	Ref ref_assigned_vehicle (null)
}