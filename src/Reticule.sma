use core
use base
use gui

_native_code_
%{
#include "cpp/coords-utils.h"
%}

_define_
Reticule (Process map, Process f)
{
  Spike show_reticule
  Spike hide_reticule
  Spike show_reticule2
  Spike hide_reticule2

  Double pointer_lat2 (0)
  Double pointer_lon2 (0)

  Switch ui (hidden) {
    Component hidden

    Component visible {
      OutlineColor _(Black)
      Line l1 (0, 0, 0, 0)
      Line l2 (0, 0, 0, 0)
      f.move.x =:> l1.x1, l1.x2
      f.move.y =:> l2.y1, l2.y2
      f.width =:> l2.x2
      f.height =:> l1.y2 
      
      Component bg {
        FillColor _ (White)
        NoOutline _
        FillOpacity _ (0.5)
        Rectangle r (0, 0, 100, 30)
        f.move.x =:> r.x
        f.move.y =:> r.y
      }
      
      FillColor _(Black)
      Text label (0, 0, "")
      //map.pointer_lat + " " + map.pointer_lon =:> label.text

      bg.r.x + 10 =:> label.x
      bg.r.y + 20 =:> label.y

      label.width + 20 =:> bg.r.width
    }
  }

  FSM reticule {
    State off {
      "hidden" =: ui.state
    }
    State on_from_wp {
      "visible" =: ui.state
      map.pointer_lat + " " + map.pointer_lon =:> ui.visible.label.text
    }
    State on_from_movable_obj {
      "visible" =: ui.state
      pointer_lat2 + " " + pointer_lon2 =:> ui.visible.label.text
    }
    off -> on_from_wp (show_reticule)
    on_from_wp -> off (hide_reticule)
    off -> on_from_movable_obj (show_reticule2)
    on_from_movable_obj -> off (hide_reticule2)
  }

}
