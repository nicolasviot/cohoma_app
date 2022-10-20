use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap
import Trap


_define_
TrapLayer (Process _map, Process _context, Process _model_manager)
{
	//map aka _map
	//context aka _context
	//model_manager aka _model_manager

	// Load only once SVG file
	svg_info = loadFromXML ("res/svg/trap_info.svg")
	svg_trap_info aka svg_info // To be accessible with a "find_child"

	Scaling sc (1, 1, 0, 0)
    _map.zoom =:> sc.sx, sc.sy

    Translation pos (0, 0)
    _map.xpan - _map.cur_ref_x + _map.px0 =:> pos.tx
    _map.ypan - _map.cur_ref_y + _map.py0 =:> pos.ty

	List traps {
		
	}

	if (_model_manager.IS_DEBUG)
	{
		addChildrenTo traps {
			Trap debug_trap1 (_map, svg_info, $_context.init_lat, $_context.init_lon - 0.0015, 200, null)
			Trap debug_trap2 (_map, svg_info, $_context.init_lat + 0.0005, $_context.init_lon - 0.003, 223, null)
		}
	}
}