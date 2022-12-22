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


	// **************************************************************************************************
    //
    //  EDGES
    //
    // **************************************************************************************************
	Component edges {
		OutlineColor outline_color ($_context.EDGE_COLOR)

		List lst
	}

	_model_manager.edges.$added -> na_edges_added:(this) {
		if (this.edges.lst.size == 0) {
			print ("New model of edge added: " + this.model_manager.edges.size + " edges. View is empty (" + this.edges.lst.size + " edges)\n")

			for model : this.model_manager.edges {
				addChildrenTo this.edges.lst {
					Edge edge (this.context, model)
				}
			}
		}
		else if (this.edges.lst.size == this.model_manager.edges.size - 1) {
			print ("New model of edge added: " + this.model_manager.edges.size + " edges. View has " + this.edges.lst.size + " edges\n")

			model = getRef (&this.model_manager.edges.$added)
			addChildrenTo this.edges.lst {
				Edge edge (this.context, model)
			}
		}
		else {
			print ("FIXME TODO: New model of edge added: " + this.model_manager.edges.size + " edges. View has " + this.edges.lst.size + " edges\n")
		}
	}

	_model_manager.edges.$removed -> na_edges_removed:(this) {
		print ("Model of edge removed: " + this.model_manager.edges.size + " edges.\n")
	}


	// **************************************************************************************************
    //
    //  NODES
    //
    // **************************************************************************************************
	List nodes

	_model_manager.nodes.$added -> na_nodes_added:(this) {
		if (this.nodes.size == 0) {
			print ("New model of node added: " + this.model_manager.nodes.size + " nodes. View is empty (" + this.nodes.size + " nodes)\n")

			for model : this.model_manager.nodes {
				addChildrenTo this.nodes {
					Node node (this.map, this.context, model)
				}
			}
		}
		else if (this.nodes.size == this.model_manager.nodes.size - 1) {
			print ("New model of node added: " + this.model_manager.nodes.size + " nodes. View has " + this.nodes.size + " nodes\n")

			model = getRef (&this.model_manager.nodes.$added)
			addChildrenTo this.nodes {
				Node node (this.map, this.context, model)
			}
		}
		else {
			print ("FIXME TODO: New model of node added: " + this.model_manager.nodes.size + " nodes. View has " + this.nodes.size + " nodes\n")
		}
	}

	_model_manager.nodes.$removed -> na_nodes_removed:(this) {
		print ("Model of node removed: " + this.model_manager.nodes.size + " nodes.\n")
	}


	// **************************************************************************************************
    //
    //  DEBUG
    //
    // **************************************************************************************************
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
