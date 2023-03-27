use core
use base
use gui

_native_code_
%{
#include "cpp/coords-utils.h"
%}

_define_
Reticule (Process _frame, Process _context)
{
  Spike show
  Spike hide

  Switch switch (hidden) {
    Component hidden

    Component visible {
      OutlineColor _(Black)
      Line l1 (0, 0, 0, 0)
      Line l2 (0, 0, 0, 0)
      _frame.move.x =:> l1.x1, l1.x2
      _frame.move.y =:> l2.y1, l2.y2
      _frame.width =:> l2.x2
      _frame.height =:> l1.y2 
      
      Component bg {
        FillColor _ (White)
        NoOutline _
        FillOpacity _ (0.5)
        Rectangle r (0, 0, 100, 30)
        _frame.move.x =:> r.x
        _frame.move.y =:> r.y
      }
      
      FillColor _(Black)
      Text label (0, 0, "")
      _context.pointer_lat + " " + _context.pointer_lon =:> label.text

      bg.r.x + 10 =:> label.x
      bg.r.y + 20 =:> label.y

      label.width + 20 =:> bg.r.width
    }
  }

  FSM fsm {
    State hidden
    State visible

    hidden -> visible (show)
    visible -> hidden (hide)
  }
  fsm.state => switch.state

  _context.start_drag_map_item -> show
  _context.stop_drag_map_item -> hide

  //TextPrinter tp
  //"FSM in reticule = " + fsm.state =:> tp.input

}
