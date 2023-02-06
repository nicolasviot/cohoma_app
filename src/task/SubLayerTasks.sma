/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2023)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *	Contributors:
 *    Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use gui
use base
use display

import SubLayer
import task.TaskArea
import task.TaskEdge
import task.TaskTrap

_native_code_
%{
    #include <iostream>
%}


_action_
action_clean_tasks (Process src, Process self)
{
	print ("CLEAR (View) " + self.ui.areas.size + " tasks about an AREA, " + self.ui.edges.lst.size + " tasks about an EDGE and " + self.ui.traps.size + " tasks about a TRAP\n")

	for task_area : self.ui.areas {
		delete task_area
	}

	for task_edge : self.ui.edges.lst {
		delete task_edge
	}

	for task_trap : self.ui.traps {
		delete task_trap
	}
}


_define_
SubLayerTasks (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager
	
	Spike clean_only_views
	Spike clean_views_then_models	

	NativeAction na_clean_only_views (action_clean_tasks, this, 1)
	clean_only_views -> na_clean_only_views

	NativeAction na_clean_task_views_then_models (action_clean_tasks, this, 1)
	clean_views_then_models -> na_clean_task_views_then_models

	// Then, clean the models
	na_clean_task_views_then_models -> _model_manager.clear_tasks


	addChildrenTo this.switch.true {

		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		//  AREAS (ZONES)
		List areas


		//  EDGES
		Component edges {
			OutlineColor outline_color ($_context.TASK_EDGE_COLOR)

			List lst
		}

		//  TRAPS
		List traps
	}

	ui aka this.switch.true


	_model_manager.task_areas.$added -> na_task_areas_added:(this) {
		print ("New model of task for area(s) added to list " + this.model_manager.task_areas.size + "\n")

		for model : this.model_manager.task_areas {
			addChildrenTo this.ui.areas {
				TaskArea task (this.map, this.context, model)
			}
		}
	}

	// FIXME: We can't clear the model first, because views are coupled to these models and it generates errors in std-out:
	/*_model_manager.task_areas.$removed -> na_task_areas_removed:(this) {
		print ("Model of task for area(s) removed from list " + this.model_manager.task_areas.size + "\n")
	
		for task : this.ui.areas {
			delete task
		}
	}*/


	_model_manager.task_edges.$added -> na_task_edges_added:(this) {
		print ("New model of task for edge(s) added to list " + this.model_manager.task_edges.size + "\n")

		for model : this.model_manager.task_edges {
			addChildrenTo this.ui.edges.lst {
				TaskEdge task (this.context, model, $this.context.TASK_EDGE_WIDTH)
			}
		}
	}

	// FIXME: We can't clear the model first, because views are coupled to these models and it generates errors in std-out:
	/*_model_manager.task_edges.$removed -> na_task_edges_removed:(this) {
		print ("Model of task for edge(s) removed from list " + this.model_manager.task_edges.size + "\n")
	
		for task : this.ui.edges.lst {
			delete task
		}
	}*/


	_model_manager.task_traps.$added -> na_task_traps_added:(this) {
		print ("New model of task for trap(s) added to list " + this.model_manager.task_traps.size + "\n")

		for model : this.model_manager.task_traps {
			addChildrenTo this.ui.traps {
				TaskTrap task (this.map, this.context, model)
			}
		}
	}

	/*_model_manager.task_traps.$removed -> na_task_traps_removed:(this) {
		print ("Model of task for trap(s) removed from list " + this.model_manager.task_traps.size + "\n")
	
		for task : this.ui.traps {
			delete task
		}
	}*/
	

	// **************************************************************************************************
    //
    //  DEBUG
    //
    // **************************************************************************************************
	/*if (_model_manager.IS_DEBUG)
	{
		// TRAPS
		_model_manager.task_traps.$added -> na_task_traps_added:(this) {
			print ("New model of task for trap added to list " + this.model_manager.task_traps.size + "\n")

			model = getRef (&this.model_manager.task_traps.$added)
			addChildrenTo this.ui.traps {
				TaskTrap task (this.map, this.context, model)
			}
		}

		_model_manager.task_traps.$removed -> na_task_traps_removed:(this) {
			print ("Model of task for trap removed from list " + this.model_manager.task_traps.size + "\n")
		}
	}*/
}