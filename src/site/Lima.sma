use core
use gui
use base

import behavior.NotDraggableItem
import widgets.CLabel

_native_code_
%{
    #include <iostream>
%}


_define_
Lima (Process _map, Process _context, Process _model)
{
    //map aka _map
    //context aka _context
    //model aka _model

    //print ("New view of Lima " + _model.name + "\n")

    Spike double_click
    double_click -> _model.na_validate_lima
    
    FillOpacity fill_op (0.8)
    OutlineOpacity outline_op (0.5)

    NoFill _
    OutlineWidth outline_width (10)
    OutlineColor outline_color (#1414AF)
    
    Polyline poly_line

    Component behaviors

    for (int i = 1; i <= _model.points.size; i++) {
        //print ("View of point: lat = " + _model.points.[i].lat + " -- lon = " + _model.points.[i].lon + "\n")
        
        addChildrenTo poly_line.points {
            PolyPoint _ (0, 0)
        }

        addChildrenTo behaviors {
            NotDraggableItem _ (_map, _model.points.[i].lat, _model.points.[i].lon, poly_line.points.[i].x, poly_line.points.[i].y)
        }
    }

    CLabel label1 (toString(_model.name))
    CLabel label2 (toString(_model.name))

    if (_model.points.size > 0)
    {
        NotDraggableItem _ (_map, _model.points.[1].lat, _model.points.[1].lon, label1.x, label1.y)
        NotDraggableItem _ (_map, _model.points.[_model.points.size].lat, _model.points.[_model.points.size].lon, label2.x, label2.y)
    }

    FSM hovered_lima {
        State not_hovered {
            10 =: outline_width.width
        }
        State hovered {
            15 =: outline_width.width
        }
        not_hovered -> hovered (poly_line.enter)
        hovered -> not_hovered (poly_line.leave)
    }

    FSM lima_selection {
        State idle {
            #1414AF =: outline_color.value, label1.outline_color.value, label2.outline_color.value

            0.8 =:> fill_op.a
            0.5 =:> outline_op.a
        }
        State first_click {
            #1414FF =: outline_color.value, label1.outline_color.value, label2.outline_color.value

            1.0 =:> fill_op.a
            0.7 =:> outline_op.a

            Timer t (4000)

            poly_line.press -> double_click
        }
        State validated {
            #1488FF =: outline_color.value, label1.outline_color.value, label2.outline_color.value

            1.0 =:> fill_op.a, outline_op.a
        }
        idle -> first_click (poly_line.press)
        first_click -> idle (first_click.t.end)
        first_click -> validated (_model.is_validated)
    }

}