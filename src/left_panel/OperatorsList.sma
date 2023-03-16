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

  Double height (100)

  FillColor _ (#FF0000)
  Rectangle bg_debug (0, 0, 50, 100, 0, 0)
  height =:> bg_debug.height

  Operator ot (_context, _model_manager.operators.ot)
  
  Operator og1 (_context, _model_manager.operators.og1)
  ot.y + ot.height =:> og1.y

  Operator og2 (_context, _model_manager.operators.og2)
  og1.y + og1.height =:> og2.y

  Operator og3 (_context, _model_manager.operators.og3)
  og2.y + og2.height =:> og3.y
}