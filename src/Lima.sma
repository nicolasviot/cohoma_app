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

    Int id(0)

    FillOpacity fo (0.8)
    FillColor fill_color(20, 20, 190)
    OutlineWidth outline_width(5)
    OutlineColor outline_color(20, 20, 250)
    Polyline lima {
    }


}