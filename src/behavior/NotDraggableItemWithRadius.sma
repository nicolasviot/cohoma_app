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
NotDraggableItemWithRadius (Process _map, Process _lat, Process _lon, Process _radius_meter, Process _tx, Process _ty, Process _radius_pixel)
{
    //TextPrinter tp
    
    // FSM to manage zoom in/out
    FSM zoom_fsm {
        State idle {
            (lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _tx
            _map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _ty

            _radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel) =:> _radius_pixel
            
            //_radius_meter + " m --> " + _radius_pixel + " px" =:> tp.input
        }

        State zoom_in {

            Double new_x (0)
            Double new_y (0)
            Double new_r (0)
            (lon2px ($_lon, $_map.zoomLevel + 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel + 1) =: new_y
            _radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel + 1) =: new_r

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

            _map.zoom_animator.output * (dx + _map.new_dx) + init_x =:> _tx
            _map.zoom_animator.output * (dy + _map.new_dy) + init_y =:> _ty
            _map.zoom_animator.output * dr + init_r =:> _radius_pixel
        }

        State zoom_out {

            Double new_x (0)
            Double new_y (0)
            Double new_r (0)
            (lon2px ($_lon, $_map.zoomLevel - 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel - 1) =: new_y
            _radius_meter * _map.scaling_factor_correction / get_resolution ($_map.zoomLevel - 1) =: new_r
    
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

            _map.zoom_animator.output * (dx + _map.new_dx) + init_x =:> _tx
            _map.zoom_animator.output * (dy + _map.new_dy) + init_y =:> _ty
            _map.zoom_animator.output * dr + init_r =:> _radius_pixel
        }

        idle -> zoom_in (_map.prepare_zoom_in)
        zoom_in -> idle (_map.zoom_animator.end)
        idle -> zoom_out (_map.prepare_zoom_out)
        zoom_out -> idle (_map.zoom_animator.end)
    }

}