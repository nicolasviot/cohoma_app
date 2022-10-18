use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap


_define_
TaskLayer (Process map)
{

	List areas{
		//TaskArea _(map)
	}
	
	Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy

    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty

	List edges{
		//TaskEdge _(map, 1, 2, nodes)
	}

	List traps{
		//TaskTrap _(map, 1, 44.27432196595285, 1.729783361205679)
		//TaskTrap test (map, 23, 43.315893, 1.403865)
	}
	

}