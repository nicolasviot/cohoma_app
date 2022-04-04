use core
use gui
use base

import Node
import Edge
import ManagerId

_define_
Itineraries (Process _map, Process f, Process navgraph){
	map aka _map
	Spike create_bindings
	Spike clear
	Int id (0)
	LogPrinter debug_itineraries("debug itineraries : ")
	id =:> debug_itineraries.input
	ManagerId manager(0)


	List itinerary_unique{

	}


	id -> (this){
		for (int i = 1; i < $this.itinerary_unique.size; i++){
			for (int j = 1; j <$this.itinerary_unique.[i]; j++){
				if ($this.id == i){
					this.itinerary_unique.[i].[j].color.value = 0x1E90FF
				}
				else{
					this.itinerary_unique.[i].[j].color.value = 0x232323	
				}

			}
		}
	}


}
