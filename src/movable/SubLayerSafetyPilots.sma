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
import SafetyPilot

_define_
SubLayerSafetyPilots (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	//map aka _map
	//context aka _context
	//model_manager aka _model_manager

	// Load only once SVG file
	svg_safety_pilot = load_from_XML_once("res/svg/safety_pilot.svg")

	addChildrenTo this.switch.true {
		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		SafetyPilot drone_safety_pilot (_map, _context, _model_manager.safety_pilots.drone_safety_pilot, svg_safety_pilot)
		SafetyPilot ground_safety_pilot (_map, _context, _model_manager.safety_pilots.ground_safety_pilot, svg_safety_pilot)
	}
}