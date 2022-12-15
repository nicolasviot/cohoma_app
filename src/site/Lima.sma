use core
use gui
use base

import behavior.NotDraggableItem

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
    
    //FillOpacity fo (0.8)
    NoFill _
    OutlineWidth outline_width(15)
    OutlineColor outline_color(20, 20, 250)
    
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

    FSM hovered_lima {
        State not_hovered {
            220 =: outline_color.b
            15 =: outline_width.width
        }
        State hovered {
            170 =:outline_color.b
            25 =: outline_width.width
        }
        not_hovered -> hovered (poly_line.enter)
        hovered -> not_hovered (poly_line.leave)
    }


    FSM lima_selection {
        State idle {
            190 =: outline_color.b
            20 =: outline_color.g
        }
        State first_click {
            20 =: outline_color.b
            190 =: outline_color.g

            Timer t (5000)

            Translation tr (0, 0)
            poly_line.bounding_box.x + poly_line.bounding_box.width / 2 =: tr.tx
            poly_line.bounding_box.y + poly_line.bounding_box.height / 2 =: tr.ty

            OutlineWidth _ (1)
            OutlineColor _ (#777777)
            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            Rectangle bg (0, -15, 1, 30, 5, 5)
            
            FillColor black (#000000)
            FontWeight _ (DJN_NORMAL)
            FontSize _ (5, 24)
            TextAnchor _ (DJN_MIDDLE_ANCHOR)
            Text legend (0, 6, toString(_model.name))
            legend.width + 10 =:> bg.width
            - bg.width / 2 =:> bg.x
            //poly_line.press.x =: legend.x
            //poly_line.press.y =: legend.y
        }
        idle -> first_click (poly_line.press)
        first_click -> idle (first_click.t.end)
        first_click -> idle (poly_line.press, double_click)
    }

}