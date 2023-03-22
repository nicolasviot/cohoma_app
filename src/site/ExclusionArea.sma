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
ExclusionArea (Process _map, Process _context, Process _model)
{
    //map aka _map
    //context aka _context
    //model aka _model

    Component bg {
        FillOpacity fill_op (0.5)
        FillColor fill_col (#00CF84)
        OutlineWidth _ (1)
        OutlineColor outline_col (#FFFFFF)
        OutlineCapStyle _ (1)
    
        Polygon poly_gon

        Component behaviors

        for (int i = 1; i <= _model.points.size; i++) {
            //print ("Exclusion Area: View of point: lat = " + _model.points.[i].lat + " -- lon = " + _model.points.[i].lon + "\n")
            
            _model.barycenter.lat + _model.points.[i].lat =: _model.barycenter.lat
            _model.barycenter.lon + _model.points.[i].lon =: _model.barycenter.lon

            addChildrenTo poly_gon.points {
                PolyPoint _ (0, 0)
            }

            addChildrenTo behaviors {
                NotDraggableItem _ (_map, _model.points.[i].lat, _model.points.[i].lon, poly_gon.points.[i].x, poly_gon.points.[i].y)
            }
        }
        _model.barycenter.lat / _model.points.size =: _model.barycenter.lat
        _model.barycenter.lon / _model.points.size =: _model.barycenter.lon

        /*Switch switch (UNKNOWN) {
            Component UNKNOWN

            // Restricted Fire Area (deactivation only on clearance)
            Component RFA {
            }

            // No Fire Area (deactivation forbidden)
            Component NFA {
            }

            // No Fly Zone
            Component NFZ {
            }

            // Free Fire Area (deactivation allowed)
            Component FFA {
            }

            // Restricted Operation Zone / forbidden to all vehicles
            Component ROZ_ALL {
            }

            // Restricted Operation Zone / forbidden to ground vehicles
            Component ROZ_GROUND {
            }
        }
        _model.type =:> switch.state*/
    }


    Translation tr (0, 0)
    Scaling scale (1, 1, 0, 0)
    _context.map_scale =:> scale.sx, scale.sy

    FillOpacity text_opacity (0.8)
    FillColor _ (#EAEAEA)
    TextAnchor _ (DJN_MIDDLE_ANCHOR)
    FontWeight _ (DJN_BOLD)

    FontSize _ (5, 30)
    Text lbl_type (0, 0, "")
    _model.type =: lbl_type.text

    FontSize _ (5, 24)
    Text lbl_name (0, 40, toString(_model.name))
    //_model.name =: lbl_name.text
    
    NotDraggableItem _ (_map, _model.barycenter.lat, _model.barycenter.lon, tr.tx, tr.ty)

}