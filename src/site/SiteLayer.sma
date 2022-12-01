use core
use gui
use base
use display

import Lima
import ExclusionArea

_native_code_
%{
    #include <iostream>
%}


_define_
SiteLayer (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	Scaling sc (1, 1, 0, 0)
	_context.map_scale =:> sc.sx, sc.sy

	Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	// Exclusion areas
	List exclusion_areas

	OutlineCapStyle _ (1)

	// Limas
	List limas


	_model_manager.limas.$added -> na_node_added:(this) {
		print ("New model of Lima added to list " + this.model_manager.limas.size + "\n")
		
		for model : this.model_manager.limas {
			addChildrenTo this.limas {
				Lima lima (this.map, this.context, model) //, null)
			}
		}
	}

	_model_manager.limas.$removed -> na_node_removed:(this) {
		print ("Model of Lima removed from list " + this.model_manager.limas.size + "\n")
	}


	// DEBUG
	for model : _model_manager.limas {
		addChildrenTo this.limas {
			Lima lima (_map, _context, model) //, null)
		}
	}
}