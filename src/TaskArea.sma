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


    FillOpacity _ (0.5)
    FillColor _ (140, 30, 30)

 Polygon area {
        Point pt1(-50, -20)
        Point pt2(-20, 20)
        Point pt3(20, 20)
        Point pt4(50, -20)
    } 
    TaskAreaSummit sum1(map, 43.316021818382886, 1.4041900634765625)
    TaskAreaSummit sum2(map, 43.316006206187375, 1.4047694206237793)
    TaskAreaSummit sum3(map, 43.3159281451497, 1.4054131507873535)
    TaskAreaSummit sum4(map, 43.31569396143501, 1.4050912857055664)
    sum1.x =:> area.pt1.x
    sum1.y =:> area.pt1.y
    sum2.x =:> area.pt2.x
    sum2.y =:> area.pt2.y
    sum3.x =:> area.pt3.x
    sum3.y =:> area.pt3.y
    sum4.x =:> area.pt4.x
    sum4.y =:> area.pt4.y
    
    

}