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

  NativeAction move_left (fn_move_left, this, 1)
  NativeAction move_right (fn_move_right, this, 1)
  NativeAction move_up (fn_move_up, this, 1)
  NativeAction move_down (fn_move_down, this, 1)

  NativeAction zoom_in (fn_zoom_in, this, 1)
  NativeAction zoom_out (fn_zoom_out, this, 1)

  map.move_left  -> move_left
  map.move_right -> move_right
  map.move_up   -> move_up
  map.move_down   -> move_down

  int cur_row = $map.row_0
  int cur_col = $map.col_0

  Double opacity (1)

  FillOpacity fo (1)
  OutlineOpacity oo (1)
  opacity =:> fo.a, oo.a

  // TODO : VIRER LA LISTE .. useless
  List layers {
    Component _ {
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
    }
  }

  // TODO :remove Ref and deref ... use aka ?
  RefProperty ref_corner_tile (layers.[1].tiles.[1].[1])
  DerefDouble ref_y_0 (ref_corner_tile, "y0", DJNN_GET_ON_CHANGE)
  DerefDouble ref_x_0 (ref_corner_tile, "x0", DJNN_GET_ON_CHANGE)
  ref_y_0.value =:> t0_y
  ref_x_0.value =:> t0_x


  // TODO : simplfier au moment ou on retire la liste
  ref_layer aka layers.[1]
  ref_opacity aka ref_layer.opacity
  ref_tr_tx aka ref_layer.pan_tr.tx
  ref_tr_ty aka ref_layer.pan_tr.ty
  ref_sc_cx aka ref_layer.sc.cx
  ref_sc_cy aka ref_layer.sc.cy
  ref_zoom aka ref_layer.zoom

  Spike update_layer_after_zoom_in
  Spike update_layer_after_zoom_out

  move_left  -> set_tile_0 : (this) {
    setRef (this.ref_corner_tile, this.layers.[1].tiles.[1].[1])
  }
  move_right -> set_tile_0
  move_up    -> set_tile_0
  move_down  -> set_tile_0
  update_layer_after_zoom_in -> set_tile_0
  update_layer_after_zoom_out -> set_tile_0

  map.zoom_in_req->zoom_in
  map.zoom_out_req->zoom_out

  zoom_in->set_corner_tile_in:(this) {
    this.buff_lon = this.pointer_lon
    this.buff_lat = this.pointer_lat
    this.new_t0_x = this.layers.[1].tiles.[1].[1].x0
    this.new_t0_y = this.layers.[1].tiles.[1].[1].y0
  }
  set_corner_tile_in->map.prepare_zoom_in

  zoom_out->set_corner_tile_out:(this) {
    this.buff_lon = this.pointer_lon
    this.buff_lat = this.pointer_lat
    this.new_t0_x = this.layers.[1].tiles.[1].[1].x0
    this.new_t0_y = this.layers.[1].tiles.[1].[1].y0
  }
  set_corner_tile_out->map.prepare_zoom_out

  update_layer_after_zoom_in->map.end_zoom_in
  update_layer_after_zoom_out->map.end_zoom_out

  FSM zoom_control {
    State idle {
      map.real_xpan_intermediaire =:> ref_tr_tx
      map.real_ypan_intermediaire =:> ref_tr_ty

      1 =: ref_zoom
    }
    State zooming_in {
      0 =: map.zoom_animator.inc.state, map.zoom_animator.gen.input
      1 =: ref_opacity
      map.xpan + map.px0 + map.new_dx =: ref_tr_tx
      map.ypan + map.py0 + map.new_dy =: ref_tr_ty
      map.move_x =: ref_sc_cx
      map.move_y =: ref_sc_cy
      0.5 + map.zoom_animator.output/2 =:> ref_zoom
    }
    State zooming_out {
      0 =: map.zoom_animator.inc.state, map.zoom_animator.gen.input
      1 =: ref_opacity
      map.xpan + map.px0 + map.new_dx =: ref_tr_tx
      map.ypan + map.py0 + map.new_dy =: ref_tr_ty
      map.move_x =: ref_sc_cx
      map.move_y =: ref_sc_cy
      2 - map.zoom_animator.output =:> ref_zoom
    }
    idle->zooming_in (map.prepare_zoom_in, map.zoom_animator.start)
    zooming_in->idle (map.zoom_animator.end, update_layer_after_zoom_in)
    idle->zooming_out (map.prepare_zoom_out, map.zoom_animator.start)
    zooming_out->idle (map.zoom_animator.end, update_layer_after_zoom_out)
  }
}
