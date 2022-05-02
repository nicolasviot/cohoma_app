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
Lima (Process map){


    FillOpacity _ (0.8)
    FillColor _ (20, 20, 190)
    OutlineWidth _(5)
    OutlineColor _(20, 20, 250)
    Polyline lima0 {
        Point pt1(0, 0)
        Point pt2(0, 0)
        } 
    Polyline lima1 {
        Point pt1(0, 0)
        Point pt2(0, 0)
        }
    Polyline lima2{
        Point pt1(0, 0)
        Point pt2(0, 0)
        }
    //Caylus data
    TaskAreaSummit sum1(map, 44.27454525208339, 1.72954755492094)
    TaskAreaSummit sum2(map, 44.27382741959425, 1.730230127226058)
    TaskAreaSummit sum3(map, 44.27459179197952, 1.729637632977594)
    TaskAreaSummit sum4(map, 44.27306606824116, 1.72774131662401)
    TaskAreaSummit sum5(map, 44.27595901914592, 1.727783939576377)
    TaskAreaSummit sum6(map, 44.27515916379403, 1.725966143121076)



    sum1.x =:> lima0.pt1.x
    sum1.y =:> lima0.pt1.y
    sum2.x =:> lima0.pt2.x
    sum2.y =:> lima0.pt2.y

    sum3.x =:> lima1.pt1.x
    sum3.y =:> lima1.pt1.y
    sum4.x =:> lima1.pt2.x
    sum4.y =:> lima1.pt2.y
    
    sum5.x =:> lima2.pt1.x
    sum5.y =:> lima2.pt1.y
    sum6.x =:> lima2.pt2.x
    sum6.y =:> lima2.pt2.y
    

}