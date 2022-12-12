use core
use gui
use base

import Node
import Edge

_native_code_
%{
    #include <iostream>
%}


_define_
NavGraph (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	Int id (0)

	//Spike create_bindings
	//Spike clear

	Scaling sc (1, 1, 0, 0)
	_context.map_scale =:> sc.sx, sc.sy

	Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	// EDGES
	Component edges {
		OutlineColor outline_color ($_context.EDGE_COLOR)

		List lst
	}

	_model_manager.edges.$added -> na_edges_added:(this) {
		print ("New model of edge added dynamically to list " + this.model_manager.edges.size + "\n")
		
		model = getRef (&this.model_manager.edges.$added)
    	addChildrenTo this.edges.lst {
			Edge edge (this.context, model)
		}
	}

	_model_manager.edges.$removed -> na_edge_removed:(this) {
		print ("Model of edge removed dynamically from list " + this.model_manager.edges.size + "\n")
	}


	// NODES
	List nodes

	_model_manager.nodes.$added -> na_node_added:(this) {
		print ("New model of node added dynamically to list " + this.model_manager.nodes.size + "\n")
		
		model = getRef (&this.model_manager.nodes.$added)
    	addChildrenTo this.nodes {
			Node node (this.map, this.context, model)
		}
	}

	_model_manager.nodes.$removed -> na_node_removed:(this) {
		print ("Model of node removed dynamically from list " + this.model_manager.nodes.size + "\n")
	}


	// DEBUG
	if (_model_manager.IS_DEBUG)
	{
		for model : _model_manager.nodes {
			addChildrenTo this.nodes {
				Node node (_map, _context, model)
			}
		}

		for model : _model_manager.edges {
			addChildrenTo this.edges.lst {
				Edge edge (_context, model)
			}
		}
	}
}
