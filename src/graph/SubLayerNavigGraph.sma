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

import SubLayer
import Node
import Edge

_native_code_
%{
    #include <iostream>
	using namespace std;
%}

//**************************************************************************************************
//
//  NODES
//
//**************************************************************************************************
_action_
action_node_ids_added (Process src, Process self)
{
	if (self.ui.nodes.size == 0) {
		print ("New model of node added: " + self.model_manager.node_ids.size + " nodes. View is empty (" + self.ui.nodes.size + " nodes)\n")

		for node_id : self.model_manager.node_ids {
			string str_id = toString (node_id)

			model = find (self.model_manager.nodes, str_id)
			if (&model != null) {
				Node (self.ui.nodes, "", self.map, self.context, model)
			}
			else {
				print ("ERROR: No model of node for id " + node_id + "\n")
			}
		}
	}
	else if (self.ui.nodes.size == self.model_manager.node_ids.size - 1) {
		print ("New model of node added: " + self.model_manager.node_ids.size + " nodes. View has " + self.ui.nodes.size + " nodes\n")

		node_id = getRef (self.model_manager.node_ids.$added)
		string str_id = toString (node_id)

		model = find (self.model_manager.nodes, str_id)
		if (&model != null) {
			Node (self.ui.nodes, "", self.map, self.context, model)
		}
		else {
			print ("ERROR: No model of node for id " + node_id + "\n")
		}
	}
	else {
		print ("FIXME TODO: New model of node added: " + self.model_manager.node_ids.size + " nodes. View has " + self.ui.nodes.size + " nodes\n")
	}
}

_action_
action_node_ids_removed (Process src, Process self)
{
	print ("FIXME TODO: Model of node removed: " + self.model_manager.node_ids.size + " edges. View has " + self.ui.nodes.size + " edges\n")
}


//**************************************************************************************************
//
//  EDGES
//
//**************************************************************************************************
/*_action_
action_edge_ids_added (Process src, Process self)
{
	if (self.ui.edges.lst.size == 0) {
		print ("New model of edge added: " + self.model_manager.edge_ids.size + " edges. View is empty (" + self.ui.edges.lst.size + " edges)\n")

		for edge_id : self.model_manager.edge_ids {
			string str_id = toString (edge_id)

			model = find (self.model_manager.edges, str_id)
			if (&model != null) {
				Edge (self.ui.edges.lst, "", self.context, model, $self.context.EDGE_WIDTH)
			}
			else {
				print ("ERROR: No model of edge for id " + edge_id + "\n")
			}
		}
	}
	else if (self.ui.edges.lst.size == self.model_manager.edge_ids.size - 1) {
		print ("New model of edge added: " + self.model_manager.edge_ids.size + " edges. View has " + self.ui.edges.lst.size + " edges\n")

		edge_id = getRef (self.model_manager.edge_ids.$added)
		string str_id = toString (edge_id)

		model = find (self.model_manager.edges, str_id)
		if (&model != null) {
			Edge (self.ui.edges.lst, "", self.context, model, $self.context.EDGE_WIDTH)
		}
		else {
			print ("ERROR: No model of node for id " + edge_id + "\n")
		}
	}
	else {
		print ("FIXME TODO: New model of edge added: " + self.model_manager.edge_ids.size + " edges. View has " + self.ui.edges.lst.size + " edges\n")
	}
}

_action_
action_edge_ids_removed (Process src, Process self)
{
	print ("FIXME TODO: Model of edge removed: " + self.model_manager.edge_ids.size + " edges. View has " + self.ui.edges.lst.size + " edges\n")
}*/


_define_
SubLayerNavigGraph (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	Int id (0)

	//Spike create_bindings
	//Spike clear

	//NativeAction na_edge_ids_added (action_edge_ids_added, this, 1)
	//_model_manager.edge_ids.$added -> na_edge_ids_added

	//NativeAction na_edge_ids_removed (action_edge_ids_removed, this, 1)			
	//_model_manager.edge_ids.$removed -> na_edge_ids_removed

	NativeAction na_node_ids_added (action_node_ids_added, this, 1)
	_model_manager.node_ids.$added -> na_node_ids_added

	NativeAction na_node_ids_removed (action_node_ids_removed, this, 1)
	_model_manager.node_ids.$removed -> na_node_ids_removed


	addChildrenTo this.switch.true {
		Component layer {

			Translation pos (0, 0)
			_context.map_translation_x =:> pos.tx
			_context.map_translation_y =:> pos.ty

			// EDGES
			/*Component edges {
				OutlineColor outline_color ($_context.EDGE_COLOR)
				//OutlineOpacity _ (0.3)

				List lst
			}*/

			// NODES
			List nodes			
		}
	}

	ui aka this.switch.true.layer


	//____________________
	//
	//  DEBUG
	//
	if (_model_manager.IS_DEBUG)
	{
		//print ("DEBUG: Load " + _model_manager.node_ids.size + " nodes and " + _model_manager.edge_ids.size + " edges...\n")

		for node_id : _model_manager.node_ids {
			string str_id = toString(node_id)

			model = find (_model_manager.nodes, str_id)
			if (&model != null) {
				Node (ui.nodes, "", _map, _context, model)
			}
			else {
				print ("No model of node with id " + node_id + "\n")
			}
		}

		/*for edge_id : _model_manager.edge_ids {
			string str_id = toString (edge_id)

			model = find (_model_manager.edges, str_id)
			if (&model != null) {
				Edge (ui.edges.lst, "", _context, model, $_context.EDGE_WIDTH)
			}
			else {
				print ("No model of edge with id " + edge_id + "\n")
			}
		}*/
	}
}
