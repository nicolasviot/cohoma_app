use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
ItineraryModel (Process _context, string _type)
{
	//context aka _context

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
	
	List nodes_ids

	LogPrinter lp ("Itinerary regexp (debug): ")
	//type + " (" + uid + "): " + description_input =:> lp.input
	type + " (" + uid + "): " + nodes_ids.size =:> lp.input
}