use core
use base
use display
use gui

_define_
CLabel (string _label)
{
  Translation tr (0, 0)

  /*----- interface -----*/
  x aka tr.tx
  y aka tr.ty
  /*----- interface -----*/

  FillOpacity fo (0.8)

  OutlineWidth _ (1)
  OutlineColor outline_color (#777777)
  FillColor light_grey (#D3D3D3)
  Rectangle bg (0, -15, 1, 30, 15, 15)
  
  FillColor black (#000000)
  FontWeight _ (DJN_BOLD) // (DJN_NORMAL)
  FontSize _ (5, 20)
  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  Text legend (0, 8, _label)
  legend.width + 10 =:> bg.width
  - bg.width / 2 =:> bg.x
}
