use core
use gui
use base

import Node
import Edge

_native_code_
%{
    #include <iostream>
	using namespace std;
%}


_action_
action_node_ids_added (Process src, Process self)
{
	if (self.nodes.size == 0) {
		print ("New model of node added: " + self.model_manager.node_ids.size + " nodes. View is empty (" + self.nodes.size + " nodes)\n")

		for node_id : self.model_manager.node_ids {
			string str_id = toString (node_id)

			model = find (self.model_manager.nodes, str_id)

			Node (self.nodes, str_id, self.map, self.context, model)
		}
	}
	else if (self.nodes.size == self.model_manager.node_ids.size - 1) {
		print ("New model of node added: " + self.model_manager.node_ids.size + " nodes. View has " + self.nodes.size + " nodes\n")

	}
	else {
		print ("FIXME TODO: New model of node added: " + self.model_manager.node_ids.size + " nodes. View has " + self.nodes.size + " nodes\n")
	}
}


_action_
action_edge_ids_added (Process src, Process self)
{
	if (self.edges.lst.size == 0) {
		print ("New model of edge added: " + self.model_manager.edge_ids.size + " edges. View is empty (" + self.edges.lst.size + " edges)\n")

		for edge_id : self.model_manager.edge_ids {
			string str_id = toString (edge_id)

			model = find (self.model_manager.edges, str_id)
			if (&model != null) {
				Edge (self.edges, str_id, self.context, model)
			}
		}
	}
	else if (self.edges.lst.size == self.model_manager.edge_ids.size - 1) {
		print ("New model of edge added: " + self.model_manager.edge_ids.size + " edges. View has " + self.edges.lst.size + " edges\n")

	}
	else {
		print ("FIXME TODO: New model of edge added: " + self.model_manager.edge_ids.size + " edges. View has " + self.edges.lst.size + " edges\n")
	}
}


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

	/*_model_manager.edges.$added -> na_edges_added:(this) {
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
	}*/

	NativeAction na_edge_ids_added (action_edge_ids_added, this, 1)
	_model_manager.edge_ids.$added -> na_edge_ids_added


	// **************************************************************************************************
    //
    //  NODES
    //
    // **************************************************************************************************
	List nodes

	/*_model_manager.nodes.$added -> na_nodes_added:(this) {
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
	}*/

	NativeAction na_node_ids_added (action_node_ids_added, this, 1)
	_model_manager.node_ids.$added -> na_node_ids_added


	// **************************************************************************************************
    //
    //  DEBUG
    //
    // **************************************************************************************************
	/*if (_model_manager.IS_DEBUG)
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
	}*/
}
