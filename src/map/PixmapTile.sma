use core
use base
use gui

_native_code_
%{
#include <iostream>
#include "cpp/coords-utils.h"
#include <cmath>
%}

_define_
PixmapTile (int _x, int _y, int _zoomLevel, int row_m, int col_m, NativeCode loader, string name, string _proxy, Process opacity)
{
  Int Z (_zoomLevel)
  Int X (col_m)
  Int Y (row_m)
  Double lat0 (tiley2lat (row_m, _zoomLevel))
  Double lon0 (tilex2lon (col_m, _zoomLevel))
  String layer_name (name)
  String proxy (_proxy)
  tiley2lat ($Y, $Z) => lat0
  tilex2lon ($X, $Z) => lon0
  Bool updated (0)
  Int x0 (0)
  Int y0 (0)
  lon2px ($lon0, $Z) =:> x0
  lat2py ($lat0, $Z) =:> y0

  FillOpacity fo (1)
  OutlineOpacity oo (1)
  opacity =:> fo.a, oo.a
  
  Int x (_x)
  Int y (_y)

  Image img ("src/img/default.png", _x, _y, 256, 256)
  NativeAsyncAction update_image (loader, this, 0)
  Z->update_image
  X->update_image
  Y->update_image

  /*
  NoFill _
  OutlineColor _ (Black)
  Rectangle r (_x, _y, 256, 256)
  img.{x, y, width, height} =:> r.{x, y, width, height}
  
  Component sub {
    NoOutline _
    FillColor _ (White)
    FillOpacity _ (0.5)
    Rectangle bg (0, 0, 100, 20)
    img.x =:> bg.x
    img.y + 128 =:> bg.y
  }
  FillColor _ (Black)
  FontSize _(1, 10)
  Text t (0, 0, "")
  sub.bg.x + 5 =:> t.x
  sub.bg.y + 15 =:> t.y
  t.width + 10 =:> sub.bg.width
  Double x_col (0)
  Double y_row (0)
  img.x / 256 =:> x_col
  img.y / 256 =:> y_row
  "x: " + x_col + " y: " + y_row =:> t.text
  //"Y: " + Y + " X: " + X =:> t.text
  //"col: " + i_col + " row: " + i_row =:> t.text
  */
}
