use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap
import Trap


_define_
TrapLayer (Process _map, Process _context)
{
	Scaling sc (1, 1, 0, 0)
    _map.zoom =:> sc.sx, sc.sy

    Translation pos (0, 0)
    _map.xpan - _map.cur_ref_x + _map.px0 =:> pos.tx
    _map.ypan - _map.cur_ref_y + _map.py0 =:> pos.ty

	List traps {
		Trap debug (_map, $_context.init_lat, $_context.init_lon - 0.0015, 200, null)
	}
}