use core
use gui
use base

//import EdgeModel

_native_code_
%{
    #include <iostream>

	extern Process* get_edge_model (Process* self, int index_1, int index_2);
%}


_define_
ItineraryModel (Process _context, Process _model_manager, string _type)
{
	//context aka _context
	model_manager aka _model_manager

	// Const
	String type (_type)

	// Can be modified
	String uid ("")

	print ("Model of itinerary '" + type + "'\n")

	// 'e5bd15ed-d7c3-4ddd-b80a-a1a8121f9e7e' -- 'Planning shortest OK: a path including PPOs was found with cost 501.248'
	// 'b5ac3bb1-593c-4dff-b1bd-9dcca5c39ab2' -- 'Planning safest OK: a path including PPOs was found with cost 501.248'
	// '4e5aa5e6-0ed4-430f-9094-d0e8295594e7' -- 'Planning tradeoff OK: a path including PPOs was found with cost 501.248'
	String description_input ("")
	
	Regex regex ("^Planning (\\S*) .* cost (\\S*)$")

	String legend ("???")
	String cost ("0.0")
	
	description_input =:> regex.input
	regex.[1] =:> legend
  	regex.[2] =:> cost
	
	List node_ids
	List node_indexes

	List edges

	node_ids.$added -> na_node_id_added:(this) {
		if (this.node_ids.size > 0)
		{
			print ("Node ID added to itinerary " + this.type + ": " + this.node_ids.size + " nodes\n")

			for (int i = 1; i < this.node_ids.size; i++) {
				print ("New edge from " + this.node_ids.[i] + " to " + this.node_ids.[i+1] + "\n")
				edge_model = get_edge_model (this.model_manager, $this.node_ids.[i], $this.node_ids.[i+1])
				if (&edge_model != null) {
					addChildrenTo this.edges {
						edge_model
					}
				}
				else {
					print ("ERROR: NO model of edge from " + this.node_ids.[i] + " to " + this.node_ids.[i+1] + "\n")
				}
			}
		}
		else {
			print ("NONE node ID added to itinerary " + this.type + " --> empty itinerary !\n")
		}
	}

	node_ids.$removed -> na_node_id_removed:(this) {
		print ("Node ID removed from itinerary " + this.type + ": " + this.node_ids.size + "\n")
	}

	LogPrinter lp ("Itinerary regexp (debug): ")
	//type + " (" + uid + "): " + description_input =:> lp.input
	type + " (" + uid + "): " + node_ids.size =:> lp.input
}