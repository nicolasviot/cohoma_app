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
NotDraggableItem (Process _map, Process _lat, Process _lon, Process _tx, Process _ty)
{
    // FSM to manage zoom in/out
    FSM fsm {
        State idle {
            (lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _tx
            _map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _ty
        }

        State zoom_in {
            Double new_x (0)
            Double new_y (0)
            (lon2px ($_lon, $_map.zoomLevel + 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel + 1) =: new_y

            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input
            Double dx (0)
            Double dy (0)
            Double init_x (0)
            Double init_y (0)
            _tx =: init_x
            _ty =: init_y
            new_x - init_x =: dx
            new_y - init_y =: dy

            anim.output * (dx + _map.new_dx) + init_x =:> _tx
            anim.output * (dy + _map.new_dy) + init_y =:> _ty
        }

        State zoom_out {
            Double new_x (0)
            Double new_y (0)
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel - 1) =: new_y
            (lon2px ($_lon, $_map.zoomLevel - 1) - _map.new_t0_x) =: new_x

            Animator anim (200, 0, 1, DJN_IN_SINE, 0, 1)
            0 =: anim.inc.state, anim.gen.input
            Double dx (0)
            Double dy (0)
            Double init_x (0)
            Double init_y (0)
            _tx =: init_x
            _ty =: init_y
            new_x - init_x =: dx
            new_y - init_y =: dy
            
            anim.output * (dx + _map.new_dx) + init_x =:> _tx
            anim.output * (dy + _map.new_dy) + init_y =:> _ty
        }

        idle -> zoom_in (_map.prepare_zoom_in)
        zoom_in -> idle (zoom_in.anim.end)
        idle -> zoom_out (_map.prepare_zoom_out)
        zoom_out -> idle (zoom_out.anim.end)
    }

}