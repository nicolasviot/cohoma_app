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
import ItineraryOnMap

_native_code_
%{
	#include <iostream>
%}


_define_
SubLayerItineraries (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	map aka _map
	context aka _context
	//model_manager aka _model_manager

	addChildrenTo this.switch.true {

		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		// Itineraries on map
		Component itineraries_on_map {
			
			// Create a view for each model of itinerary
			for model : _model_manager.itineraries {
		
				ItineraryOnMap itinerary (_context, _model_manager, model)

				model.is_selected.true -> (itinerary) {
					moveChild itinerary >>
				}
			}
		}
	}

}
