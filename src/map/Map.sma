/*
 *  Smala Map component
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2021)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

 use core
 use base
 use gui

 import map.PanAndZoom
 import Reticule

 _native_code_
 %{
//#include <chrono>
//#include <thread>

#include <math.h>
//#include <iostream>
#include "exec_env/global_mutex.h"
//#include "core/execution/graph.h"
#include "core/core-dev.h" 
#include "cpp/tiles_manager.h"
#include "cpp/coords-utils.h"
%}

_define_
Map (Process f, int _x, int _y, double _width, double _height, double _lat, double _lon, int _zoom)
{
  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty

  Double init_lat (_lat)
  Double init_lon (_lon)
  Double pointer_lat (_lat)
  Double pointer_lon (_lon)
  Double lat_center (_lat)
  Double lon_center (_lon)
  Double mod_x_zooom_in (0)
  Double mod_y_zooom_in (0)

  Double scaling_factor_correction(0)
  // mercator scaling correction, use pointer_lat to keep updating the correction
  //, but we should use lat_center (not updated in this version), if we have an app that do not use the pointer

  //(1 + c2*(cos(2*f) - 1)) / cos(f) where c2 = 0.00001120378
  Double c2 (0.00001120378)
  Cosine cos1 (0)
  Cosine cos2 (0)
  2 * pointer_lat * 2 * 3.14159265359 / 360=:> cos1.input
  pointer_lat * 2 * 3.14159265359 / 360 =:> cos2.input
  (1 + c2 * (cos1.output - 1 )) / cos2.output =:> scaling_factor_correction


  Double t0_x (0)
  Double t0_y (0)
  Int width (_width)
  Int height (_height)

  Int zoomLevel (_zoom)

  Int nbCols (ceil(_width/256) + 4)
  Int nbRows (ceil(_height/256) + 4)
  ceil(width/256) + 4 =:> nbCols
  ceil(height/256) + 4 =:> nbRows

  Spike move_left
  Spike move_right
  Spike move_up
  Spike move_down

  Spike end_zoom_in
  Spike end_zoom_out
  Spike zoom_in_req
  Spike zoom_out_req
  Spike incr_zoom
  Spike decr_zoom

  Int pix_offset_x (0)
  Int pix_offset_y (0)
  Int buff_pix_offset_x (1)
  Int buff_pix_offset_y (1)
  Int cur_ref_x (0)
  Int cur_ref_y (0)

  Double new_dx (0)
  Double new_dy (0)
  Double new_t0_x (0)
  Double new_t0_y (0)

  int t0_col = lon2tilex (_lon, _zoom) // indice x de la tuile couvrant le point de référence
  int t0_row = lat2tiley (_lat, _zoom) // indice y de la tuile couvrant le point de référence
  
  double center_px = lon2px (_lon, _zoom) // coordonnée x en pixels du point de référence
  double center_py = lat2py (_lat, _zoom) // coordonnée y en pixels du point de référence
  double lat0 = tiley2lat (t0_row, _zoom) // lat du coin supérieur gauche de la tuile du point de référence
  double lon0 = tilex2lon (t0_col, _zoom) // lon du coin supérieur gauche de la tuile du point de référence
  double corner_px = lon2px (lon0, _zoom) // coordonnée x en pixels du coind supérieur gauche de la tuile couvrant point de référence
  double corner_py = lat2py (lat0, _zoom) // coordonnée y en pixels du coind supérieur gauche de la tuile couvrant point de référence

  double _dx = center_px - corner_px
  double _dy = center_py - corner_py
  //double init_cx = width/2 - _dx
  //double init_cy = height/2 - _dy

  //compute how many tiles before the central one
  int even_rows = $nbRows%2 == 0
  int even_cols = $nbCols%2 == 0
  int n_rows_before = ceil($nbRows/2)
  int n_cols_before = ceil($nbCols/2)

  
  int _px0 = - 512
  int _py0 = - 512
  if (!even_rows) {
    _py0 -= 128
  }
  if (!even_cols) {
    _px0 -= 128
  }
  
  Int px0 (_px0 + _dx)
  Int py0 (_py0 + _dy)

  Int col_0 (t0_col - n_cols_before)
  Int row_0 (t0_row - n_rows_before)
  
  Int dx (_dx)
  Int dy (_dy)
  dx + _px0 =:> px0
  dy + _py0 =:> py0
  
  Double buff_lon (0)
  Double buff_lat (0)

  Component g_map {
    NoFill _
    NoOutline _
    PickFill _
    Rectangle pick_area (0, 0, 0, 0, 0, 0)
    RectangleClip clip_area (0, 0, _width, _height)


    PanAndZoom pz (f.move, pick_area, pick_area.wheel.dy)
    this.{width,height} =:> clip_area.{width,height}, pick_area.{width, height}

    Component layers
  }
  zoom aka g_map.pz.zoom
  xpan aka g_map.pz.xpan
  ypan aka g_map.pz.ypan
  acc_dx aka g_map.pz.acc_dx.result
  acc_dy aka g_map.pz.acc_dy.result
  layers aka g_map.layers
  enter aka g_map.pick_area.enter
  leave aka g_map.pick_area.leave
  
  Reticule reticule (this, f)
  
  xpan->{ (xpan - cur_ref_x)/256 =: pix_offset_x }
  ypan->{ (ypan - cur_ref_y)/256 =: pix_offset_y }
  
  Int pointer_col (0)
  Int pointer_row (0)

  Int x_odd (!even_cols)
  Int y_odd (!even_rows)
  
  FSM fsm {
    State idle {
      //Calcul des coordonnées du pointeur
      px2lon (t0_x + g_map.pick_area.move.x - px0 - (xpan - cur_ref_x), $zoomLevel) => pointer_lon
      py2lat (t0_y - g_map.pick_area.move.y + py0 + (ypan - cur_ref_y), $zoomLevel) => pointer_lat
 
      floor((g_map.pick_area.move.local_x - (xpan - cur_ref_x) + 512)/256.0) + x_odd =:> pointer_col
      floor((g_map.pick_area.move.local_y - (ypan - cur_ref_y) + 512)/256.0) + y_odd =:> pointer_row

      (g_map.pick_area.move.local_x - (xpan - cur_ref_x - acc_dx) + 512)%256.0 =:> mod_x_zooom_in
      (g_map.pick_area.move.local_y - (ypan - cur_ref_y - acc_dy) + 512)%256.0 =:> mod_y_zooom_in
    }
    State pressed {
    }
    idle->pressed (f.press)
    pressed->idle (f.release)
  }

  Double new_lon (0)
  Double new_lat (0)

  incr_zoom-> {
    zoomLevel + 1 =: zoomLevel
    xpan + new_dx =: xpan
    ypan + new_dy =: ypan
  }

  decr_zoom-> {
    zoomLevel - 1 =: zoomLevel
    xpan + new_dx =: xpan
    ypan + new_dy =: ypan
  }

  AssignmentSequence prepare_zoom_in (1) {
    px2lon (new_t0_x + g_map.pick_area.move.x - px0 - (xpan - cur_ref_x), zoomLevel+1) =: new_lon
    py2lat (new_t0_y - g_map.pick_area.move.y + py0 + (ypan - cur_ref_y), zoomLevel+1) =: new_lat
    lon2px ($new_lon, zoomLevel + 1) - lon2px ($buff_lon, zoomLevel + 1) =: new_dx
    lat2py ($buff_lat, zoomLevel + 1) - lat2py ($new_lat, zoomLevel + 1) =: new_dy
  }
  
  AssignmentSequence prepare_zoom_out (1) {
    px2lon (new_t0_x + g_map.pick_area.move.x - px0 - (xpan - cur_ref_x), zoomLevel-1) =: new_lon
    py2lat (new_t0_y - g_map.pick_area.move.y + py0 + (ypan - cur_ref_y), zoomLevel-1) =: new_lat
    lon2px ($new_lon, zoomLevel - 1) - lon2px ($buff_lon, zoomLevel - 1) =: new_dx
    lat2py ($buff_lat, zoomLevel - 1) - lat2py ($new_lat, zoomLevel - 1) =: new_dy
  }

  FSM zoom_fsm {
    State idle {
      Bool max (0)
      Bool min (0)
      zoomLevel == 19 =:> max
      zoomLevel == 1 =:> min
      Switch check_zoom (both) {
        Component both {
          g_map.pick_area.wheel.dy > 0 -> zoom_in_req
          g_map.pick_area.wheel.dy < 0 -> zoom_out_req
        }
        Component only_in {
          g_map.pick_area.wheel.dy > 0 -> zoom_in_req
        }
        Component only_out {
          g_map.pick_area.wheel.dy < 0 -> zoom_out_req
        }
      }
      min == 1 ? "only_in" : (max == 1 ? "only_out" : "both") =:> check_zoom.state 
    }
    State zooming_in
    State zooming_out
    idle->zooming_in (zoom_in_req)
    zooming_in->idle (end_zoom_in, incr_zoom)
    idle->zooming_out (zoom_out_req)
    zooming_out->idle (end_zoom_out, decr_zoom)
  }

  AssignmentSequence prepare_move_right (1) {
    pix_offset_x =: buff_pix_offset_x
    cur_ref_x + 256*pix_offset_x =: cur_ref_x
  }
  (pix_offset_x >=  1) -> prepare_move_right
  prepare_move_right -> move_right
  
  AssignmentSequence prepare_move_left (1) {
    pix_offset_x =: buff_pix_offset_x
    cur_ref_x - abs(256*pix_offset_x) =: cur_ref_x
  }

  (pix_offset_x <= -1) -> prepare_move_left
  prepare_move_left -> move_left

  AssignmentSequence prepare_move_down (1) {
    pix_offset_y =: buff_pix_offset_y
    cur_ref_y + (pix_offset_y*256) =: cur_ref_y
  }
  (pix_offset_y >=  1) -> prepare_move_down
  prepare_move_down -> move_down

  AssignmentSequence prepare_move_up (1) {
    pix_offset_y =: buff_pix_offset_y
    cur_ref_y - abs(pix_offset_y*256) =: cur_ref_y
  }
  (pix_offset_y <= -1) -> prepare_move_up
  prepare_move_up -> move_up
}
