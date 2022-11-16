use core
use gui
use display
use base

_native_code_
%{
	#include <iostream>
%}


_define_
ItineraryOnMap (Process _context, Process _model)
{
	//context aka _context
	model aka _model

	TextComparator compare_selected_uid ("", "")
	_model.uid =:> compare_selected_uid.left
	toString(_context.selected_itinerary_id) =:> compare_selected_uid.right

	

	/*Switch switch (false) {
		Component false{

		}
		Component true {

		}
	}
	compare_selected_uid.output =:> switch.state*/

	FillColor fill (#FF0000)
	//Rectangle r (200, 200, 100, 100, 0, 0)

	List edges

	print ("Itinerary on Map " + _model.type + "\n")

	_model.nodes_ids.size -> na_size:(this) {
		print ("Node ID size changed to itinerary " + this.model.type + ": " + this.model.nodes_ids.size + "\n")
	}

	_model.nodes_ids.$added -> na_node_id_added:(this) {
		if (this.model.nodes_ids.size > 0)
		{
			print ("Node ID added to itinerary " + this.model.type + ": " + this.model.nodes_ids.size + "\n")

			/*for node_id : this.model.nodes_ids {
				print ("Node " + node_id + "\n")
			}*/
			for (int i = 1; i < this.model.nodes_ids.size; i++) {
				print ("New edge from " + this.model.nodes_ids.[i] + " to " + this.model.nodes_ids.[i+1] + "\n")
				//addChildrenTo this.edges {
				//	Edge edge (this.map, this.context, model, this.svg_info, this.svg_remotely_icon, this.svg_contact_icon)
			}
		}
		else {
			print ("NONE node ID added to itinerary " + this.model.type + " --> empty itinerary !\n")
		}
	}

	_model.nodes_ids.$removed -> na_node_id_removed:(this) {
		print ("Node ID removed from itinerary " + this.model.type + ": " + this.model.nodes_ids.size + "\n")
		//model = getRef (&this.model_manager.traps.$removed)
	}


	/*for (int i = 1; i < ite_edges_size; i++) {
	ParentProcess* edge = Edge( new_ite_edges, "", ros_itinerary.second[i-1] + 1, ros_itinerary.second[i] + 1, 20, _nodes);
	SET_CHILD_VALUE (Int, edge, outline_color/value, unselected, true)
	new Binding (edge, "binding_edge_released", edge, "mask_edge/release", _edge_released_na, "");*/
}