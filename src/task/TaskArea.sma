use core
use gui
use base
use display

import behavior.NotDraggableItem

_native_code_
%{
    #include <iostream>
%}


_define_
TaskArea (Process _map, Process _context, Process _model)
{
    //map aka _map
    //context aka _context
    //model aka _model

    FillOpacity fill_op (0.25)
    FillColor fill_col (#8C1E1E)
    OutlineWidth outline_w (0)
    OutlineColor outline_col (#000000)

    Polygon poly_gon

    Component behaviors

    for (int i = 1; i <= _model.points.size; i++) {
        print ("Task Area: View of point: lat = " + _model.points.[i].lat + " -- lon = " + _model.points.[i].lon + "\n")
        
        addChildrenTo poly_gon.points {
            PolyPoint _ (0, 0)
        }

        addChildrenTo behaviors {
            NotDraggableItem _ (_map, _model.points.[i].lat, _model.points.[i].lon, poly_gon.points.[i].x, poly_gon.points.[i].y)
        }
    }

    FSM fsm_selection { 
    
        State not_selected {
            0 =: _model.is_selected

            0 =: outline_w.width
            #000000 =: outline_col.value
        }

        State selected {
            1 =: _model.is_selected

            5 =: outline_w.width
            _context.TASK_SELECTION_COLOR =: outline_col.value
        }
        selected -> not_selected (poly_gon.release)
        not_selected -> selected (poly_gon.release)
    }
}