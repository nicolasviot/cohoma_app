use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap


_define_
TaskLayer (Process map){


	List areas{
		//TaskArea _(map)
	}


	List edges{
		//TaskEdge _(map, 1, 2, nodes)
	}
	
	List traps{
		//TaskTrap _(map, 1, 44.27432196595285, 1.729783361205679)
	}
	

}