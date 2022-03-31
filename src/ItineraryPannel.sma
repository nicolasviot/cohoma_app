use core 
use gui
use base
use display 

import ItineraryDescriptor
_define_
ItineraryPannel(double _dx, double _dy){
	Translation t (_dx, _dy)

	Int idSelected(1)

	List itineraries_descriptors{
		ItineraryDescriptor _ (10, 10)
		ItineraryDescriptor _ (10, 120)
		ItineraryDescriptor _ (10, 230)	
	}

	"selected" =: itineraries_descriptors.[$idSelected].fsm.state



	
}