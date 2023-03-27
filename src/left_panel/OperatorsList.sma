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

  Int DELTA_Y_BETWEEN_OPERATORS (5)
  Double height (0)

  //FillColor _ (#FF0000)
  //Rectangle bg_debug (0, 0, 50, 50, 0, 0)
  //height =:> bg_debug.height

  Operator ot (_context, _model_manager.operators.ot)
  
  Operator og1 (_context, _model_manager.operators.og1)
  ot.y + ot.height + $DELTA_Y_BETWEEN_OPERATORS =:> og1.y

  Operator og2 (_context, _model_manager.operators.og2)
  og1.y + og1.height + $DELTA_Y_BETWEEN_OPERATORS =:> og2.y

  Operator og3 (_context, _model_manager.operators.og3)
  og2.y + og2.height + $DELTA_Y_BETWEEN_OPERATORS =:> og3.y

  og3.y + og3.height =:> height

  _context.show_drop_zones_strip =:> og1.accept_drop
  _context.show_drop_zones_strip =:> og2.accept_drop
  _context.show_drop_zones_strip =:> og3.accept_drop

}