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
import TrapDetection

_native_code_
%{
    #include <iostream>
%}


_define_
SubLayerTrapDetections (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	addChildrenTo this.switch.true {

		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		List trap_detections
	}

	ui aka this.switch.true

	//Component bindings

	_model_manager.trap_detections.$added -> na_trap_detection_added:(this) {
		print ("New model of trap detection added to list " + this.model_manager.trap_detections.size + " (" + this.ui.trap_detections.size + " views)\n")

		//model = getRef (&this.model_manager.trap_detections.$added)

		for (int i = this.ui.trap_detections.size + 1; i <= this.model_manager.trap_detections.size; i++)
		{
			model = &this.model_manager.trap_detections.[i]
			
			addChildrenTo this.ui.trap_detections {
				TrapDetection trap_detection (this.map, this.context, model)
				//TrapDetection trap_detection (this.map, this.context, this.model_manager.trap_detections.[i])
			}	
		}
	}

	_model_manager.trap_detections.$removed -> na_trap_detection_removed:(this) {
		print ("Model of trap detection removed from list " + this.model_manager.trap_detections.size + " (" + this.ui.trap_detections.size + " views)\n")

		//model = getRef (&this.model_manager.trap_detections.$removed)
	}

}