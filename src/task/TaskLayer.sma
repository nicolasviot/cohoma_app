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
	
	Scaling sc (1, 1, 0, 0)
    _context.map_scale =:> sc.sx, sc.sy

    Translation pos (0, 0)
    _context.map_translation_x =:> pos.tx
    _context.map_translation_y =:> pos.ty


    // **************************************************************************************************
    //
    //  AREAS (ZONES)
    //
    // **************************************************************************************************
	List areas {

	}

	_model_manager.task_areas.$added -> na_task_areas_added:(this) {
		print ("New model of task for area(s) added to list " + this.model_manager.task_areas.size + "\n")

		for model : this.model_manager.task_areas {
			addChildrenTo this.areas {
				TaskArea task (this.map, this.context, model)
			}
		}
	}

	_model_manager.task_areas.$removed -> na_task_areas_removed:(this) {
		print ("Model of task for area(s) removed from list " + this.model_manager.task_areas.size + "\n")
	}


    // **************************************************************************************************
    //
    //  EDGES
    //
    // **************************************************************************************************
	Component edges {
		OutlineColor outline_color ($_context.TASK_EDGE_COLOR)

		List lst
	}

	_model_manager.task_edges.$added -> na_task_edges_added:(this) {
		print ("New model of task for edge(s) added to list " + this.model_manager.task_edges.size + "\n")
		//model = getRef (&this.model_manager.task_edges.$added)
    	//addChildrenTo this.edges {
		//	TaskEdge task (this.map, this.context, model)
		//}

		for model : this.model_manager.task_edges {
			addChildrenTo this.edges.lst {
				TaskEdge task (this.context, model)
			}
		}
	}

	_model_manager.task_edges.$removed -> na_task_edges_removed:(this) {
		print ("Model of task for edge(s) removed from list " + this.model_manager.task_edges.size + "\n")
	}


	// **************************************************************************************************
    //
    //  TRAPS
    //
    // **************************************************************************************************
	List traps {

	}
	

	// **************************************************************************************************
    //
    //  DEBUG
    //
    // **************************************************************************************************
	if (_model_manager.IS_DEBUG)
	{
		// TRAPS
		_model_manager.task_traps.$added -> na_task_trap_added:(this) {
			print ("New model of task for trap added to list " + this.model_manager.task_traps.size + "\n")

			model = getRef (&this.model_manager.task_traps.$added)
			addChildrenTo this.traps {
				TaskTrap task (this.map, this.context, model)
			}
		}

		_model_manager.task_traps.$removed -> na_task_trap_removed:(this) {
			print ("Model of task for trap removed from list " + this.model_manager.task_traps.size + "\n")
		}
	}
}