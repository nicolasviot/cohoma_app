use core
use gui
use base
use display

import TaskArea
import TaskEdge
import TaskTrap


_define_
TaskLayer (Process map){


	List Areas{
		TaskArea _(map)
	}
	List Edges{
	}
	List Traps{
		TaskTrap _(map, 1, 43.315313261816485, 1.404974527891014)
	}
}