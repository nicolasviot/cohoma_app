use core
use gui
use base
use display

//import ros_node
import behavior.NotDraggableItem

_native_code_
%{
    #include <iostream>
%}


_define_
ExclusionArea (Process _map, Process _context, Process _model)
{
    //map aka _map
    //context aka _context
    //model aka _model

    Component bg {
        FillOpacity fo (0.2)
        FillColor fill_color (0)
        OutlineWidth _ (6)
        OutlineColor outline_color (0)
        OutlineCapStyle _ (1)
    
        Polygon poly_gon

        Component behaviors

        for (int i = 1; i <= _model.points.size; i++) {
            //print ("View of point: lat = " + _model.points.[i].lat + " -- lon = " + _model.points.[i].lon + "\n")
            
            addChildrenTo poly_gon.points {
                PolyPoint _ (0, 0)
            }

            addChildrenTo behaviors {
                NotDraggableItem _ (_map, _model.points.[i].lat, _model.points.[i].lon, poly_gon.points.[i].x, poly_gon.points.[i].y)
            }
        }

        Switch switch (UNKNOWN) {
            Component UNKNOWN

            // Restricted Fire Area (deactivation only on clearance)
            Component RFA {

            }

            // No Fire Area (deactivation forbidden)
            Component NFA {
                0.3 =: fo.a
                #14B4B4 =: fill_color.value
                #14B4B4 =: outline_color.value
            }

            // No Fly Zone
            Component NFZ {
                0.3 =: fo.a
                #FAFA32 =: fill_color.value
                #FAFA32 =: outline_color.value
            }

            // Free Fire Area (deactivation allowed)
            Component FFA {
            }

            // Restricted Operation Zone / forbidden to all vehicles
            Component ROZ_ALL {
                0.7 =: fo.a
                #FF0000 =: fill_color.value
                #FF0000 =: outline_color.value
            }

            // Restricted Operation Zone / forbidden to ground vehicles
            Component ROZ_GROUND {
                0.3 =: fo.a
                #FFA500 =: fill_color.value
                #FFA500 =: outline_color.value
            }
        }
        _model.type =:> switch.state
    }

    Translation tr (0, 0)

    FillOpacity text_opacity (0.8)
    FillColor _ (#EAEAEA)
    TextAnchor _ (DJN_MIDDLE_ANCHOR)

    FontSize _ (5, 28)
    FontWeight _ (DJN_BOLD)
    Text lbl_type (0, 0, "")
    _model.type =: lbl_type.text

    FontSize _ (5, 24)
    FontWeight _ (DJN_BOLD)
    Text lbl_name (0, 40, toString(_model.name))
    //_model.name =: lbl_name.text
    

    // FSM to manage zoom in/out
    FSM fsm {
        State idle {
            0.8 =: text_opacity.a
            bg.poly_gon.bounding_box.x + bg.poly_gon.bounding_box.width / 2 =:> tr.tx
            bg.poly_gon.bounding_box.y + bg.poly_gon.bounding_box.height / 2 =:> tr.ty
        }

        State zoom_in {
            Timer t (200)
            0.0 =: text_opacity.a
        }

        State zoom_out {
            Timer t (200)
            0.0 =: text_opacity.a
        }

        idle -> zoom_in (_map.prepare_zoom_in)
        zoom_in -> idle (zoom_in.t.end)
        idle -> zoom_out (_map.prepare_zoom_out)
        zoom_out -> idle (zoom_out.t.end)
    }

}