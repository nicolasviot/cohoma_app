use core
use gui
use base
use animation

import gui.animation.Animator

_native_code_
%{
    #include "cpp/coords-utils.h"
%}


_define_
DraggableItemWithRadius (Process _map, Process _lat, Process _lon, Process _radius_meter, Process _tx, Process _ty, Process _picking, Process _radius_pixel)
{
    //TextPrinter tp

    // 0 = not lazy (updated on first activation)
    AssignmentSequence update_radius_pixel (0) {
        _radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =: _radius_pixel
        //_radius_meter + " m --> " + _radius_pixel + " px" =: tp.input
    }

    // FSM to manage zoom in/out
    FSM zoom_fsm {
        State idle {
            //(lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _tx
            //_map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _ty

            // FIXME: scaling_factor_correction is updated each time the mouse move
            //_radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =:> _radius_pixel
            _radius_meter -> update_radius_pixel
            _map.zoomLevel -> update_radius_pixel
            
            //_radius_meter + " m --> " + _radius_pixel + " px" =:> tp.input
        }

        State zoom_in {

            Double new_x (0)
            Double new_y (0)
            Double new_r (0)
            (lon2px ($_lon, $_map.zoomLevel + 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel + 1) =: new_y
            _radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel + 1) =: new_r

            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input

            Double init_x (0)
            Double init_y (0)
            Double init_r (0)
            _tx =: init_x
            _ty =: init_y
            _radius_pixel =: init_r

            Double dx (0)
            Double dy (0)
            Double dr (0)
            new_x - init_x =: dx
            new_y - init_y =: dy
            new_r - init_r =: dr

            //"zoom IN: (" + _radius_meter + " m) " + init_r + " --> " + new_r + " px" =:> tp.input

            anim.output * (dx + _map.new_dx) + init_x =:> _tx
            anim.output * (dy + _map.new_dy) + init_y =:> _ty
            anim.output * dr + init_r =:> _radius_pixel
        }

        State zoom_out {

            Double new_x (0)
            Double new_y (0)
            Double new_r (0)
            (lon2px ($_lon, $_map.zoomLevel - 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel - 1) =: new_y
            _radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel - 1) =: new_r
            
            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input

            Double init_x (0)
            Double init_y (0)
            Double init_r (0)
            _tx =: init_x
            _ty =: init_y
            _radius_pixel =: init_r
            
            Double dx (0)
            Double dy (0)
            Double dr (0)
            new_x - init_x =: dx
            new_y - init_y =: dy
            new_r - init_r =: dr

            //"zoom OUT: (" + _radius_meter + " m) " + init_r + " --> " + new_r + " px" =:> tp.input

            anim.output * (dx + _map.new_dx) + init_x =:> _tx
            anim.output * (dy + _map.new_dy) + init_y =:> _ty
            anim.output * dr + init_r =:> _radius_pixel
        }

        idle -> zoom_in (_map.prepare_zoom_in)
        zoom_in -> idle (zoom_in.anim.end)
        idle -> zoom_out (_map.prepare_zoom_out)
        zoom_out -> idle (zoom_out.anim.end)
    }


    // FSM to manage drag
    FSM drag_fsm {
        State no_drag {
            (lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _tx
            _map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _ty
        }

        State drag {
            Double offset_x (0)
            Double offset_y (0)
            
            _picking.press.x - _tx =: offset_x
            _picking.press.y - _ty =: offset_y
            _picking.move.x - offset_x => _tx
            _picking.move.y - offset_y => _ty

            px2lon ($_tx + _map.t0_x, $_map.zoomLevel) => _lon, _map.reticule.pointer_lon2
            py2lat (_map.t0_y - $_ty, $_map.zoomLevel) => _lat, _map.reticule.pointer_lat2
        }
        no_drag -> drag (_picking.left.press, _map.reticule.show_reticule2)
        drag -> no_drag (_picking.left.release, _map.reticule.hide_reticule2)
    }

}