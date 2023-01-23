use core
use gui
use base

import SubLayer
import Vehicle

_define_
SubLayerVehicles (Process _layer_model, Process _map, Process _context, Process _model_manager) inherits SubLayer (_layer_model)
{
	// Load only once SVG files
	svg_vab = load_from_XML_once ("res/svg/vab.svg")
  	svg_robot = load_from_XML_once ("res/svg/robot.svg")
  	svg_drone = load_from_XML_once ("res/svg/drone.svg")

	addChildrenTo this.switch.true {
		Translation pos (0, 0)
		_context.map_translation_x =:> pos.tx
		_context.map_translation_y =:> pos.ty

		Vehicle vab (_map, _context, _model_manager.vehicles.vab, svg_vab)
		Vehicle agilex1 (_map, _context, _model_manager.vehicles.agilex1, svg_robot)
		Vehicle agilex2 (_map, _context, _model_manager.vehicles.agilex2, svg_robot)
		Vehicle lynx (_map, _context, _model_manager.vehicles.lynx, svg_robot)
		Vehicle spot (_map, _context, _model_manager.vehicles.spot, svg_robot)
		Vehicle drone (_map, _context, _model_manager.vehicles.drone, svg_drone)
	}
}