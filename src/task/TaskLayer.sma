use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap


_define_
TaskLayer (Process _map, Process _context)
{

	List areas{
		//TaskArea _(map)
	}
	
	Scaling sc (1, 1, 0, 0)
    _context.map_scale =:> sc.sx, sc.sy

    Translation pos (0, 0)
    _context.map_translation_x =:> pos.tx
    _context.map_translation_y =:> pos.ty

	List edges{
		//TaskEdge _(map, 1, 2, nodes)
	}

	List traps{
		//TaskTrap _(map, 1, 44.27432196595285, 1.729783361205679)
		//TaskTrap test (map, 23, 43.315893, 1.403865)
	}
	

}