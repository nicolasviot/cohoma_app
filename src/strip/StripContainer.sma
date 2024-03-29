use core
use gui
use base
use gui
  
import Strip

_define_
StripContainer (Process _context, Process _model_manager, Process _frame)
{  
  // Load only once SVG file
  svg_strip = load_from_XML ("res/svg/stripV2.svg")

  Layer layer (0, 0 ,1350, 0) { //bindings on y and h under

    Translation tr (0, 0)
    _frame.height - _context.STRIP_HEIGHT+1  =:> tr.ty

    Strip strip_vab (_context, _model_manager.vehicles.vab, svg_strip, 0)

    Strip strip_agilex_1 (_context, _model_manager.vehicles.agilex1, svg_strip, 1)

    Strip strip_agilex_2 (_context, _model_manager.vehicles.agilex2, svg_strip, 2)

    Strip strip_lynx (_context, _model_manager.vehicles.lynx, svg_strip, 3)

    Strip strip_spot (_context, _model_manager.vehicles.spot, svg_strip, 4)

    Strip strip_drone (_context, _model_manager.vehicles.drone, svg_strip, 5)
  }
  _frame.height - _context.STRIP_HEIGHT =:> layer.y
  _context.STRIP_HEIGHT =:> layer.h
}