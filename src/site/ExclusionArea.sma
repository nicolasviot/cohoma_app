use core
use gui
use base
use display
use animation

import task.TaskAreaSummit

/*_native_code_
%{
    #include "cpp/coords-utils.h"
%}*/


_define_
ExclusionArea (Process map, string _status)
{
    String name ("")
 
    /*
    uint8 TYPE_UNKNOWN    = 0 # Unknown zone type
    uint8 TYPE_RFA        = 1 # Restricted Fire Area (deactivation only on clearance)
    uint8 TYPE_NFA        = 2 # No Fire Area (deactivation forbidden)
    uint8 TYPE_NFZ        = 3 # No Fly Zone
    uint8 TYPE_FFA        = 4 # Free Fire Area (deactivation allowed)
    uint8 TYPE_ROZ_ALL    = 5 # Restricted Operation Zone (forbidden to all vehicles)
    uint8 TYPE_ROZ_GROUND = 6 # Restricted Operation Zone (forbidden to ground vehicles)
    */
    String status (_status)

    FillColor  white(234, 234, 234)
    FontSize _ (5, 30)
    FontWeight _ (75)
    
    Text zone_label(0, 0, "")
    name =:> zone_label.text

    Text type_label(0, 0, "")
    status =:> type_label.text
    
    Double barycenterX(0)
    Double barycenterY(0)
    barycenterX - zone_label.width / 2 =:> zone_label.x
    barycenterY =:> zone_label.y

    barycenterX  - type_label.width / 2 =:> type_label.x
    barycenterY + zone_label.height  =:> type_label.y

    FillOpacity fo(0.2)
    FillColor fill_color (20, 190, 20)
    OutlineWidth _(6)
    OutlineColor outline_color(20, 190, 20)
    OutlineCapStyle _ (1)
 
    Polygon area {  
    }

    Switch type (_status) {
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

            20 =: outline_color.r
            180 =: outline_color.g
            180 =: outline_color.b
        }
        Component ffa{
            0.3 =: fo.a
            20 =: fill_color.r
            180 =: fill_color.g
            180 =: fill_color.b
            
            20 =: outline_color.r
            180 =: outline_color.g
            180 =: outline_color.b
        }
        Component nfz{
            0.3 =: fo.a
            250 =: fill_color.r
            250 =: fill_color.g
            50 =: fill_color.b

            255 =: outline_color.r
            250=: outline_color.g
            50 =: outline_color.b
        }
        Component nfa
        Component rfa
        Component unknown
    }
    status =:> type.state

    LogPrinter lp ("Excelusion area (debug): ")
    status =:> lp.input

}