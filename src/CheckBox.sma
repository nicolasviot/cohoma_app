/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
use core
use base
use display
use gui

_define_
CheckBox (string _label, double x_, double y_) {
  Translation t (x_, y_)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  /*----- interface -----*/

  Int check_color (#dd9408)
  Int idle_color (#ffffff)
  Int min_height (0)
  Int min_width (0)

  //Translation offset (0, 0)
  
  FillColor text_color (#ffffff)
  Text thisLabel (23, 14, _label)
  OutlineWidth ow (2)
  OutlineColor oc (#535353)
  FillColor fc (#ffffff)
  
  Rectangle r (0, 0, 16, 16, 2, 2)
  //this.height/2 - r.height/2 =:> offset.ty

  Spike press
  r.press -> press

  FSM fsm {
    State visible {
      //idle_color =: oc.value
      check_color =: fc.value
      1 =: ow.width
      OutlineColor oc (#ffffff)
      OutlineJoinStyle _ (1)
      OutlineCapStyle _ (1)
      OutlineWidth _ (2)
      Polyline p {
        Point _ (4, 8)
        Point _ (8, 13)
        Point _ (13, 4)
      }
    }
    State hidden {
      idle_color =: fc.value
      2 =: ow.width
    }
    visible->hidden (press)
    hidden->visible (press)
  }

  thisLabel.press->press
  label aka thisLabel.text
  thisLabel.width + 23 =:> min_width
  min_height = 16
  state aka fsm.state
}
