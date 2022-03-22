use core
use base
use gui

_native_code_
%{
#include "cpp/coords-utils.h"
%}

_define_
Reticule (Process map, Process f) {
  Spike show_reticule
  Spike hide_reticule

  FSM reticule{
    State off 
    State on {
      OutlineColor _(Black)
      Line l1 (0, 0, 0, 0)
      Line l2 (0, 0, 0, 0)
      f.move.x =:> l1.x1, l1.x2
      f.move.y =:> l2.y1, l2.y2
      f.width =:> l2.x2
      f.height =:> l1.y2 
      FillColor _ (White)
      NoOutline _
      Component rr {
        FillOpacity _ (0.5)
        Rectangle bg2 (0, 0, 100, 30)
        f.move.x =:> bg2.x
        f.move.y =:> bg2.y
      }
      FillColor _(Black)
      Text t2 (0, 0, "")
      String ty ("")
      String tx ("")
      lat2tiley ($map.pointer_lat, $map.zoomLevel + 1) => ty
      lon2tilex ($map.pointer_lon, $map.zoomLevel + 1) => tx
      "" + map.pointer_lat + " " + map.pointer_lon =:> t2.text
      rr.bg2.x + 10 =:> t2.x
      rr.bg2.y + 20 =:> t2.y

      //t1.width + 20 =:> rr.bg1.width
      t2.width + 20 =:> rr.bg2.width
    }
    off -> on (show_reticule)
    on -> off (hide_reticule)
  }
}