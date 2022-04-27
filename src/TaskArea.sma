use core
use gui
use base
use display
use animation

import gui.animation.Animator
import TaskAreaSummit

_native_code_
%{
#include "cpp/coords-utils.h"

%}


_define_
TaskArea (Process map){
    Int nb_summit(0)
    Bool selected(0)
    FillOpacity _ (0.5)
    FillColor _ (140, 30, 30)
    OutlineWidth perimeter_width(0)
    OutlineColor yellow(255, 255, 0)
    Switch ctrl_area_selected(not_select){ 
    
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
   selected?"select":"not_select" => ctrl_area_selected.state


 Polygon area {
        Point pt1(-50, -20)
        Point pt2(-20, 20)
        Point pt3(20, 20)
        Point pt4(50, -20)
    } 
    TaskAreaSummit sum1(map, 44.27432196595285, 1.729783361205679)
    TaskAreaSummit sum2(map, 44.27432196595285 + 0.002, 1.729783361205679)
    TaskAreaSummit sum3(map, 44.27432196595285 + 0.002, 1.729783361205679 + 0.002)
    TaskAreaSummit sum4(map, 44.27432196595285, 1.729783361205679 + 0.002)
    sum1.x =:> area.pt1.x
    sum1.y =:> area.pt1.y
    sum2.x =:> area.pt2.x
    sum2.y =:> area.pt2.y
    sum3.x =:> area.pt3.x
    sum3.y =:> area.pt3.y
    sum4.x =:> area.pt4.x
    sum4.y =:> area.pt4.y

    area.press -> {
        selected?0:1 =: selected
    }
    
    

}