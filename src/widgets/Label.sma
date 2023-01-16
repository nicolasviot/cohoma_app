use core
use base
use display
use gui

_define_
Label (string _label)
{
  Translation tr (0, 0)

  /*----- interface -----*/
  x aka tr.tx
  y aka tr.ty
  /*----- interface -----*/

  FillOpacity fo (0.8)

  OutlineWidth _ (1)
  OutlineColor _ (#777777)
  FillColor light_grey (#D3D3D3)
  Rectangle bg (0, -15, 1, 30, 5, 5)
  
  FillColor black (#000000)
  FontWeight _ (DJN_NORMAL)
  FontSize _ (5, 24)
  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  Text legend (0, 7, _label)
  legend.width + 10 =:> bg.width
  - bg.width / 2 =:> bg.x
}
