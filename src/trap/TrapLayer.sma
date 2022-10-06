use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap
import Trap


_define_
TrapLayer (Process map) {

	Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy
	
    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty

	List traps {
		Trap test (map, 43.315893, 1.403865, 23, map)
		//Trap test (map, 44.2737, 1.72897, 200, map)
		//Trap test (map, 48.86109526727752, 1.8933138875646296, 200, map)
	}
}