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
ExclusionArea (Process map, string _status){



    String name("")
    FillOpacity fo(0.2)
    FillColor fill_color (20, 190, 20)
    OutlineWidth _(6)
    OutlineColor outline_color(20, 190, 20)

/*    uint8 TYPE_UNKNOWN    = 0 # Unknown zone type
    uint8 TYPE_RFA        = 1 # Restricted Fire Area (deactivation only on clearance)
    uint8 TYPE_NFA        = 2 # No Fire Area (deactivation forbidden)
    uint8 TYPE_NFZ        = 3 # No Fly Zone
    uint8 TYPE_FFA        = 4 # Free Fire Area (deactivation allowed)
    uint8 TYPE_ROZ_ALL    = 5 # Restricted Operation Zone (forbidden to all vehicles)
    uint8 TYPE_ROZ_GROUND = 6 # Restricted Operation Zone (forbidden to ground vehicles)

*/

     Polygon area {  
     } 

    String status(_status)
    Switch type(_status){
        Component limits{
            0 =: fo.a
            20 =: outline_color.r
            190 =: outline_color.g
            20 =: outline_color.b
         
        }
        Component roz_all{
            0.7 =: fo.a
            255 =: fill_color.r
            0 =: fill_color.g
            0 =: fill_color.b

            255 =: outline_color.r
            0 =: outline_color.g
            0 =: outline_color.b 
        }
        Component roz_ground{
            0.3 =: fo.a
            180 =: fill_color.b
            180 =: fill_color.g
            20 =: fill_color.r
            180 =: outline_color.b
            180 =: outline_color.g
            20 =: outline_color.r
        }
        Component ffa{
            0.3 =: fo.a
            180 =: fill_color.b
            180 =: fill_color.g
            20 =: fill_color.r
            180 =: outline_color.b
            180 =: outline_color.g
            20 =: outline_color.r



        }
        Component nfz{
            0.3 =: fo.a
            255 =: fill_color.r
            0 =: fill_color.g
            0 =: fill_color.b
            255 =: outline_color.r
            0 =: outline_color.g
            0 =: outline_color.b
        }
        Component nfa
        Component rfa
        Component unknown


    }
    LogPrinter lp ( "debug" )

    status =:> lp.input
    status =:> type.state
    

}