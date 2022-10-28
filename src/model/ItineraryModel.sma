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

	String type (_type)

	// ex msg : "b39409be39-39090 Planning shortest|safest|tradeoff OK a path including PPO ... with cost 286.458"
	String description_input ("")
	
	Regex regex (".* Planning (\\S*) .* cost (\\S*)")

	String legend ("")
	String cost ("0.0")
	
	description_input =:> regex.input
	//regex.[1] =:> legend
  	//regex.[2] =:> cost

	// FIXME
	"Itinerary: " + type =: legend
	
	LogPrinter lp ("Itinerary regexp (debug): ")
	description_input =:> lp.input

	print ("Model of itinerary '" + type + "'\n")

}