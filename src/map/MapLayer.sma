use core
use base
use gui
use animation

_native_code_
%{
#include "cpp/coords-utils.h"
#include "cpp/tiles_manager.h"
#include "cpp/map_move.h"
#include "core/control/native_action.h"
#include <cmath>

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"
%}

import PixmapTile
import gui.animation.Animator

_define_
MapLayer (Process f, Process map, string name, string proxy)
{
  check_and_build_dir ("cache/" + name)

  pointer_lat aka map.pointer_lat
  pointer_lon aka map.pointer_lon
  pointer_col aka map.pointer_col
  pointer_row aka map.pointer_row
  nbRows aka map.nbRows
  nbCols aka map.nbCols
  mod_x_zooom_in aka map.mod_x_zooom_in
  mod_y_zooom_in aka map.mod_y_zooom_in
  offset_x aka map.buff_pix_offset_x
  offset_y aka map.buff_pix_offset_y
  dx aka map.dx
  dy aka map.dy
  buff_lon aka map.buff_lon
  buff_lat aka map.buff_lat
  t0_x aka map.t0_x
  t0_y aka map.t0_y
  new_t0_x aka map.new_t0_x
  new_t0_y aka map.new_t0_y

  zoomLevel aka map.zoomLevel

  //NativeAction move_left_l1 (fn_move_left_l1, this, 1)
  //NativeAction move_right_l1 (fn_move_right_l1, this, 1)
  //NativeAction move_up_l1 (fn_move_up_l1, this, 1)
  //NativeAction move_down_l1 (fn_move_down_l1, this, 1)

  NativeAction move_left_l2 (fn_move_left_l2, this, 1)
  NativeAction move_right_l2 (fn_move_right_l2, this, 1)
  NativeAction move_up_l2 (fn_move_up_l2, this, 1)
  NativeAction move_down_l2 (fn_move_down_l2, this, 1)

  NativeAction zoom_in (fn_zoom_in, this, 1)
  NativeAction zoom_out (fn_zoom_out, this, 1)


  // map.move_left     -> move_left_l1
  // move_left_l1  -> move_left_l2
  // map.move_right    -> move_right_l1
  // move_right_l1 -> move_right_l2
  // map.move_up       -> move_up_l1
  // move_up_l1    -> move_up_l2
  // map.move_down     -> move_down_l1
  // move_down_l1  -> move_down_l2
  map.move_left  -> move_left_l2
  map.move_right -> move_right_l2
  map.move_up   -> move_up_l2
  map.move_down   -> move_down_l2

  int cur_row = $map.row_0
  int cur_col = $map.col_0

  Double opacity (1)


  FillOpacity fo (1)
  OutlineOpacity oo (1)
  opacity =:> fo.a, oo.a
  List layers {
    Component _ {
      Double opacity (0)
      Double zoom (1)
      Scaling sc (1, 1, 0, 0)
      zoom =:> sc.sx, sc.sy
      Translation pan_tr (0,0)

      List tiles {
        for (int i = 0; i < $nbRows; i++) {
          List row {
            for (int j = 0; j < $nbCols; j++) {
              PixmapTile _ (j*256, i*256, $zoomLevel, cur_row, cur_col, name, proxy, opacity)
              cur_col++
            }
          }
          cur_row++
          cur_col = $map.col_0
        }
      }

      // FillColor _ (Blue)
      //FillOpacity _ (1)
      // Rectangle _ (0, 0, $map.width*2, $map.height*2, 0, 0)
    }
    Component _ {
      cur_row = $map.row_0
      Double opacity (1)
      Double zoom (1)
      Scaling sc (1, 1, 0, 0)
      zoom =:> sc.sx, sc.sy
      Translation pan_tr (0,0)

      List tiles {
        for (int i = 0; i < $nbRows; i++) {
          List row {
            for (int j = 0; j < $nbCols; j++) {
              PixmapTile _ (j*256, i*256, $zoomLevel, cur_row, cur_col, name, proxy, opacity)
              cur_col++
            }
          }
          cur_row++
          cur_col = $map.col_0
        }
      }

      //FillColor _ (Red)
      // FillOpacity _ (0.1)
      //Rectangle _ (0, 0, $map.width*2, $map.height*2, 0, 0)
    }
  }
  RefProperty ref_corner_tile (layers.[1].tiles.[1].[1])
  //RefProperty ref_layer_current (layers.[1])
  RefProperty ref_layer_above (layers.[2])

  DerefDouble ref_opacity_above (ref_layer_above, "opacity", DJNN_GET_ON_CHANGE)
  //DerefDouble ref_opacity_current (ref_layer_current, "opacity", DJNN_GET_ON_CHANGE)
  DerefDouble ref_tr_above_tx (ref_layer_above, "pan_tr/tx", DJNN_GET_ON_CHANGE)
  DerefDouble ref_tr_above_ty (ref_layer_above, "pan_tr/ty", DJNN_GET_ON_CHANGE)
  //DerefDouble ref_tr_current_tx (ref_layer_current, "pan_tr/tx", DJNN_GET_ON_CHANGE)
  //DerefDouble ref_tr_current_ty (ref_layer_current, "pan_tr/ty", DJNN_GET_ON_CHANGE)
  DerefDouble ref_sc_above_cx (ref_layer_above, "sc/cx", DJNN_GET_ON_CHANGE)
  DerefDouble ref_sc_above_cy (ref_layer_above, "sc/cy", DJNN_GET_ON_CHANGE)
  //DerefDouble ref_sc_current_cx (ref_layer_current, "sc/cx", DJNN_GET_ON_CHANGE)
  //DerefDouble ref_sc_current_cy (ref_layer_current, "sc/cy", DJNN_GET_ON_CHANGE)
  //DerefDouble ref_zoom_current  (ref_layer_current, "zoom", DJNN_GET_ON_CHANGE)
  DerefDouble ref_zoom_above    (ref_layer_above, "zoom", DJNN_GET_ON_CHANGE)
  DerefDouble ref_y_0 (ref_corner_tile, "y0", DJNN_GET_ON_CHANGE)
  DerefDouble ref_x_0 (ref_corner_tile, "x0", DJNN_GET_ON_CHANGE)
 
  // 1 =: ref_opacity_current.value //demarre avec 1 dessus ... puis inversement 
  // 0 =: ref_opacity_above.value 

  ref_y_0.value   =:> t0_y
  ref_x_0.value   =:> t0_x

  move_left_l2  -> set_tile_0 : (this) {
    setRef (this.ref_corner_tile, this.layers.[2].tiles.[1].[1])
  }
  move_right_l2 -> set_tile_0
  move_up_l2    -> set_tile_0
  move_down_l2  -> set_tile_0


  map.zoom_in_req->zoom_in
  map.zoom_out_req->zoom_out

  zoom_in->set_corner_tile_in:(this) {
    this.buff_lon = this.pointer_lon
    this.buff_lat = this.pointer_lat
    this.new_t0_x = this.layers.[2].tiles.[1].[1].x0
    this.new_t0_y = this.layers.[2].tiles.[1].[1].y0
  }
  set_corner_tile_in->map.prepare_zoom_in

  zoom_out->set_corner_tile_out:(this) {
    this.buff_lon = this.pointer_lon
    this.buff_lat = this.pointer_lat
    this.new_t0_x = this.layers.[2].tiles.[1].[1].x0
    this.new_t0_y = this.layers.[2].tiles.[1].[1].y0
  }
  set_corner_tile_out->map.prepare_zoom_out


  Spike update_layer_after_zoom_in
  Spike update_layer_after_zoom_out

  update_layer_after_zoom_in->switch_layers:(this) {
    // moveChild this.layers.[2] << //sans ca le graph ne se met pas a jour ??
    setRef (this.ref_corner_tile, this.layers.[2].tiles.[1].[1])
    //setRef (this.ref_layer_current, this.layers.[1])
    //setRef (this.ref_layer_above, this.layers.[2])
  }
  update_layer_after_zoom_out->switch_layers

  update_layer_after_zoom_in->map.end_zoom_in
  update_layer_after_zoom_out->map.end_zoom_out

  FSM zoom_control {
    State idle {
      map.real_xpan_intermediaire =:> /*ref_tr_current_tx.value,*/ ref_tr_above_tx.value
      map.real_ypan_intermediaire =:> /*ref_tr_current_ty.value,*/ ref_tr_above_ty.value

      1 =: /*ref_zoom_current.value,*/ ref_zoom_above.value
    }
    State zooming_in {
      0 =: map.zoom_animator.inc.state, map.zoom_animator.gen.input
      //map.zoom_animator.output =:> ref_opacity_above.value
      1 =: ref_opacity_above.value
      // 1 - map.zoom_animator.output =:> ref_opacity_current.value
      map.xpan + map.px0 + map.new_dx =: ref_tr_above_tx.value
      map.ypan + map.py0 + map.new_dy =: ref_tr_above_ty.value
      f.move.x =: /*ref_sc_current_cx.value,*/ ref_sc_above_cx.value
      f.move.y =: /*ref_sc_current_cy.value,*/ ref_sc_above_cy.value
      // 1 + map.zoom_animator.output =:> ref_zoom_current.value
      0.5 + map.zoom_animator.output/2 =:> ref_zoom_above.value
    }
    State zooming_out {
      print ("out ")
      0 =: map.zoom_animator.inc.state, map.zoom_animator.gen.input
      //map.zoom_animator.output =:> ref_opacity_above.value
      1 =: ref_opacity_above.value
      // 1 - map.zoom_animator.output =:> ref_opacity_current.value
      map.xpan + map.px0 + map.new_dx =: ref_tr_above_tx.value
      map.ypan + map.py0 + map.new_dy =: ref_tr_above_ty.value
      f.move.x =: /*ref_sc_current_cx.value,*/ ref_sc_above_cx.value
      f.move.y =: /*ref_sc_current_cy.value,*/ ref_sc_above_cy.value
      // 1 - map.zoom_animator.output =:> ref_zoom_current.value
      2 - map.zoom_animator.output =:> ref_zoom_above.value
    }
    idle->zooming_in (map.prepare_zoom_in, map.zoom_animator.start)
    zooming_in->idle (map.zoom_animator.end, update_layer_after_zoom_in)
    idle->zooming_out (map.prepare_zoom_out, map.zoom_animator.start)
    zooming_out->idle (map.zoom_animator.end, update_layer_after_zoom_out)
  }
}
