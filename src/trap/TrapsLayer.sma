use core
use gui
use base
use display

import Trap

_native_code_
%{
    #include <iostream>
%}


_define_
TrapsLayer (Process _map, Process _context, Process _model_manager)
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

    Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	List traps {
		
	}

	_model_manager.traps.$added -> na_trap_added:(this) {
		print ("New model of trap added to list " + this.model_manager.traps.size + "\n")
		model = getRef (&this.model_manager.traps.$added)
    	addChildrenTo this.traps {
			Trap trap (this.map, this.context, model, this.svg_info, this.svg_remotely_icon, this.svg_contact_icon)
		}
	}

	_model_manager.traps.$removed -> na_trap_removed:(this) {
		print ("Model of trap removed from list " + this.model_manager.traps.size + "\n")
		//model = getRef (&this.model_manager.traps.$removed)
	}

}