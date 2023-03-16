use core
use gui
use base
use gui
  
import Operator

_define_
OperatorsList (Process _context, Process _model_manager)
{
  //context aka _context
  //model_manager aka _model_manager

  Double height (0)

  FillColor _ (#FF0000)
  Rectangle fake (0, 100, 150, 150, 0, 0)

  //Translation tr (0, 0)
  //_frame.height - _context.STRIP_HEIGHT+1  =:> tr.ty

  Operator ot (_context, null)

  //Operator og1 (_context, _model_manager.vehicles.agilex1, svg_strip, 1)

  //Operator og2 (_context, _model_manager.vehicles.agilex2, svg_strip, 2)

  //Operator og3 (_context, _model_manager.vehicles.lynx, svg_strip, 3)

}