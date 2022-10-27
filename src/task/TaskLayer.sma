use core
use gui
use base
use display

import task.TaskArea
import task.TaskEdge
import task.TaskTrap

_native_code_
%{
    #include <iostream>
%}


_define_
TaskLayer (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	List areas {
		//TaskArea _(map)
	}
	
	Scaling sc (1, 1, 0, 0)
    _context.map_scale =:> sc.sx, sc.sy

    Translation pos (0, 0)
    _context.map_translation_x =:> pos.tx
    _context.map_translation_y =:> pos.ty

	List edges {
		//TaskEdge _(map, 1, 2, nodes)
	}

	List traps {

	}
	

	if (_model_manager.IS_DEBUG)
	{
		_model_manager.traps.$added -> na_trap_added:(this) {
			print ("TASK: new model of trap added to list " + this.model_manager.traps.size + "\n")
			model = getRef (&this.model_manager.traps.$added)
			addChildrenTo this.traps {
				TaskTrap task_trap (this.map, this.context, model)
			}
		}

		_model_manager.traps.$removed -> na_trap_removed:(this) {
			print ("TASK: model of trap removed from list " + this.model_manager.traps.size + "\n")
			//model = getRef (&this.model_manager.traps.$removed)
		}
	}
}