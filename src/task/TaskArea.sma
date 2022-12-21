use core
use gui
use base
use display
use animation

import task.TaskAreaSummit

_native_code_
%{
    #include "cpp/coords-utils.h"
%}


_define_
TaskArea (Process map) {

    Int nb_summit(0)
    Bool selected(0)
    Double area_prop(0)
    Double explored(0)

    FillOpacity _ (0.25)
    FillColor color (140, 30, 30)
    OutlineWidth perimeter_width(0)
    OutlineColor yellow(255, 255, 0)

    Switch ctrl_area_selected (not_select) { 
    
        Component select { 
            5 =: perimeter_width.width
            255 =: yellow.r
            255 =: yellow.g
            0 =: yellow.b
        }
        Component not_select{
            0 =: perimeter_width.width
            0 =: yellow.r
            0 =: yellow.g
            0 =: yellow.b
        }
   }
   selected ? "select" : "not_select" => ctrl_area_selected.state

    Polygon area {
        area.release -> {
            selected ? 0 : 1 =: selected
        }
    }
}