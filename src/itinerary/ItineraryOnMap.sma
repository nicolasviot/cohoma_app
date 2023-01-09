use core
use gui
use display
use base

import ItineraryEdge

_native_code_
%{
	#include <iostream>
%}


_define_
ItineraryOnMap (Process _context, Process _model_manager, Process _model)
{
	context aka _context
	model_manager aka _model_manager
	model aka _model

	OutlineColor outline_color (0)
	_model.is_selected ? _context.SELECTED_ITINERARY_COL : _context.UNSELECTED_ITINERARY_COL =:> outline_color.value

	// Sub views
	List itinerary_edges

	print ("Itinerary on Map " + _model.type + "\n")

	/*Switch switch (false) {
		Component false 
		Component true 
	}
	_model.is_selected =:> switch.state*/


	_model.node_ids.$added -> na_node_id_added:(this) {
		if (this.model.node_ids.size > 0)
		{
			print ("Node IDs added to itinerary " + this.model.type + ": " + this.model.node_ids.size + " nodes\n")

			for (int i = 1; i < this.model.node_ids.size; i++) {
				//print ("New edge from " + this.model.node_ids.[i] + " to " + this.model.node_ids.[i+1] + "\n")

				string edge_id = toString(this.model.node_ids.[i]) + "_" + toString(this.model.node_ids.[i+1])
				edge_model = find (this.model_manager.edges, edge_id)
				if (&edge_model != null)
				{
					addChildrenTo this.itinerary_edges {
						Component _ {
							ItineraryEdge edge (this.context, edge_model)
							edge.click -> this.model.select
						}
					}
				}
			}
		}
		else {
			print ("NONE node ID added to itinerary " + this.model.type + " --> empty itinerary !\n")
		}
	}

	_model.node_ids.$removed -> na_node_id_removed:(this) {
		print ("Node IDs removed from itinerary " + this.model.type + ": " + this.model.node_ids.size + " nodes\n")

		for edge_parent : this.itinerary_edges {
			delete edge_parent
		}
	}

}