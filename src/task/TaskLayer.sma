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

	// AREAS
	List areas {
		//TaskArea _(map)
	}
	
	Scaling sc (1, 1, 0, 0)
    _context.map_scale =:> sc.sx, sc.sy

    Translation pos (0, 0)
    _context.map_translation_x =:> pos.tx
    _context.map_translation_y =:> pos.ty

	// EDGES
	Component edges {
		OutlineColor outline_color ($_context.TASK_EDGE_COLOR)

		List lst
	}


	// TRAPS
	List traps {

	}


	_model_manager.task_edges.$added -> na_task_edges_added:(this) {
		print ("New model of task edge(s) added to list " + this.model_manager.task_edges.size + "\n")
		//model = getRef (&this.model_manager.task_edges.$added)
    	//addChildrenTo this.edges {
		//	TaskEdge task (this.map, this.context, model)
		//}

		for model : this.model_manager.task_edges {
			addChildrenTo this.edges.lst {
				TaskEdge task_edge (this.context, model)
			}
		}
	}

	_model_manager.task_edges.$removed -> na_task_edges_removed:(this) {
		print ("Model of task edge(s) removed from list " + this.model_manager.task_edges.size + "\n")
	}
	

	// FIXME: DEBUG
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