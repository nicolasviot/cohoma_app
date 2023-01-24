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
import Trap

_native_code_
%{
    #include <iostream>
%}


_define_
SubLayerTraps (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	// Load only once SVG file
	svg_trap_info = load_from_XML ("res/svg/trap_info.svg")
	svg_info aka svg_trap_info // To be accessible with a "find_child"

	svg_trap_remotely_icon = load_from_XML ("res/svg/trap_remote_icon.svg")
	svg_remotely_icon aka svg_trap_remotely_icon // To be accessible with a "find_child"

    svg_trap_contact_icon = load_from_XML ("res/svg/trap_contact_icon.svg")
	svg_contact_icon aka svg_trap_contact_icon // To be accessible with a "find_child"

	addChildrenTo this.switch.true {

		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		List traps
	}

	ui aka this.switch.true


	_model_manager.traps.$added -> na_trap_added:(this) {
		print ("New model of trap added to list " + this.model_manager.traps.size + "\n")

		model = getRef (&this.model_manager.traps.$added)
    	addChildrenTo this.ui.traps {
			Trap trap (this.map, this.context, model, this.svg_info, this.svg_remotely_icon, this.svg_contact_icon)
		}
	}

	_model_manager.traps.$removed -> na_trap_removed:(this) {
		print ("Model of trap removed from list " + this.model_manager.traps.size + "\n")

		//model = getRef (&this.model_manager.traps.$removed)
	}

}