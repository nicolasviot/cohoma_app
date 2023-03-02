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
DraggableItem (Process _map, Process _context, Process _lat, Process _lon, Process _dx, Process _dy, Process _picking, Process _frame_released)
{
    //TextPrinter tp

    // FSM to manage zoom in/out
    FSM zoom_fsm {

        State idle {
            //(lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _dx
            //_map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _dy
        }

        State zoom_in {

            Double new_x (0)
            Double new_y (0)
            (lon2px ($_lon, $_map.zoomLevel + 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel + 1) =: new_y

            Double init_x (0)
            Double init_y (0)
            _dx =: init_x
            _dy =: init_y

            Double dx (0)
            Double dy (0)
            new_x - init_x =: dx
            new_y - init_y =: dy

            _map.zoom_animator.output * (dx + _map.new_dx) + init_x =:> _dx
            _map.zoom_animator.output * (dy + _map.new_dy) + init_y =:> _dy
        }

        State zoom_out {

            Double new_x (0)
            Double new_y (0)
            (lon2px ($_lon, $_map.zoomLevel - 1) - _map.new_t0_x) =: new_x
            _map.new_t0_y - lat2py ($_lat, $_map.zoomLevel - 1) =: new_y
            
            Double init_x (0)
            Double init_y (0)
            _dx =: init_x
            _dy =: init_y
            
            Double dx (0)
            Double dy (0)
            new_x - init_x =: dx
            new_y - init_y =: dy

            _map.zoom_animator.output * (dx + _map.new_dx) + init_x =:> _dx
            _map.zoom_animator.output * (dy + _map.new_dy) + init_y =:> _dy
        }

        idle -> zoom_in (_map.prepare_zoom_in)
        zoom_in -> idle (_map.zoom_animator.end)
        idle -> zoom_out (_map.prepare_zoom_out)
        zoom_out -> idle (_map.zoom_animator.end)
    }


    // FSM to manage drag
    FSM drag_fsm {

        State no_drag {
            (lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _dx
            _map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _dy
        }

        State no_drag_while_shift_key {
            (lon2px ($_lon, $_map.zoomLevel) - _map.t0_x) =:> _dx
            _map.t0_y - lat2py ($_lat, $_map.zoomLevel) =:> _dy
        }

        State drag {
            Double offset_x (0)
            Double offset_y (0)
            
            _picking.press.x - _dx =: offset_x
            _picking.press.y - _dy =: offset_y
            _picking.move.x - offset_x => _dx
            _picking.move.y - offset_y => _dy

            px2lon ($_dx + _map.t0_x, $_map.zoomLevel) => _lon, _map.reticule.pointer_lon2
            py2lat (_map.t0_y - $_dy, $_map.zoomLevel) => _lat, _map.reticule.pointer_lat2
        }
        
        no_drag -> drag (_picking.left.press, _map.reticule.show2)
        drag -> no_drag (_picking.left.release, _map.reticule.hide2)
        drag -> no_drag (_frame_released, _map.reticule.hide2) // Occurs when release is done outside parent layer

        no_drag -> no_drag_while_shift_key (_context.shift)
        no_drag_while_shift_key -> no_drag (_context.shift_r)
    }

}