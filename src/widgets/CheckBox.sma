/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2023)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *	Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *    Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use base
use display
use gui

_define_
CheckBox (string _label, double _x, double _y)
{
  Translation t (_x, _y)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  
  Int checked_color (#dd9408)
  Int unchecked_color (#ffffff)

  Bool is_checked (true)
  
  Spike checked
  Spike unchecked
  /*----- interface -----*/

  Int min_height (22)
  Int min_width (0)

  // Mask
  NoFill _
  NoOutline _
  PickFill _
  Rectangle mask (0, 0, 50, $min_height, 0, 0)
  min_width =:> mask.width

  // Square background
  FillColor _ (#303030)
  Rectangle _ (0, 0, $min_height, $min_height, 2, 2)

  // Label
  FillColor text_color (#ffffff)
  FontWeight fw  (DJN_NORMAL)
  Text txt_label (26, 17, _label)
  txt_label.width + 25 =:> min_width
  
  label aka txt_label.text

  // Square border
  OutlineWidth ow (2)
  OutlineColor oc (#535353)
  FillColor fc ($checked_color)
  Rectangle r (3, 3, 16, 16, 2, 2)
  
  FSM fsm_hover {
    State idle {
      50 =: fw.weight
    }
    State hover {
      75 =: fw.weight
    }
    idle -> hover (mask.enter)
    hover -> idle (mask.leave)
  }

  FSM fsm_check {
    State st_checked {
      1 =?: is_checked

      checked_color =: fc.value
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

    State st_unchecked {
      0 =?: is_checked

      unchecked_color =: fc.value
      2 =: ow.width
    }

    st_checked -> st_unchecked (mask.press, unchecked)
    st_checked -> st_unchecked (is_checked.false) //, unchecked)

    st_unchecked -> st_checked (mask.press, checked)
    st_unchecked -> st_checked (is_checked.true) //, checked)
  }
}
