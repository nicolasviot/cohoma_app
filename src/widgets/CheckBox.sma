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
CheckBox (string _label, double _x, double _y) {
  Translation t (_x, _y)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  /*----- interface -----*/

  Int check_color (#dd9408)
  Int idle_color (#ffffff)
  Int min_height (0)
  Int min_width (0)

  NoFill _
  NoOutline _
  PickFill _
  Rectangle mask (0, 0, 50, 22, 0, 0)
  FillColor _ (#303030)
  Rectangle _ (0, 0, 22, 22, 2, 2)

  FillColor text_color (#ffffff)
  FontWeight fw  (DJN_NORMAL)
  Text thisLabel (26, 17, _label)
  thisLabel.width + 22 =:> mask.width

  OutlineWidth ow (2)
  OutlineColor oc (#535353)
  FillColor fc (#ffffff)
  
  Rectangle r (3, 3, 16, 16, 2, 2)

  FSM ctrl_hover {
    State idle {
      50 =: fw.weight
    }
    State hover {
      75 =: fw.weight
    }
    idle -> hover (mask.enter)
    hover -> idle (mask.leave)
  }

  Spike press
  mask.press -> press

  FSM fsm {
    // Visible
    State true {
      check_color =: fc.value
      1 =: ow.width
      OutlineColor oc (#ffffff)
      OutlineJoinStyle _ (1)
      OutlineCapStyle _ (1)
      OutlineWidth _ (2)
      Polyline p {
        Point _ (7, 11)
        Point _ (11, 16)
        Point _ (14, 7)
      }
    }

    // Hidden
    State false {
      idle_color =: fc.value
      2 =: ow.width
    }

    true -> false (press)
    false -> true (press)
  }

  label aka thisLabel.text
  thisLabel.width + 25 =:> min_width
  min_height = 22
  state aka fsm.state
}
