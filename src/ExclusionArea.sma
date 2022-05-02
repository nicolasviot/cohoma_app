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




    FillOpacity _(0.2)
    OutlineWidth _(6)

/*    uint8 TYPE_UNKNOWN    = 0 # Unknown zone type
    uint8 TYPE_RFA        = 1 # Restricted Fire Area (deactivation only on clearance)
    uint8 TYPE_NFA        = 2 # No Fire Area (deactivation forbidden)
    uint8 TYPE_NFZ        = 3 # No Fly Zone
    uint8 TYPE_FFA        = 4 # Free Fire Area (deactivation allowed)
    uint8 TYPE_ROZ_ALL    = 5 # Restricted Operation Zone (forbidden to all vehicles)
    uint8 TYPE_ROZ_GROUND = 6 # Restricted Operation Zone (forbidden to ground vehicles)

*/
    String status(_status)
    Switch type(_status){
        Component limits{
            NoFill _
            OutlineColor green(20, 190, 20)
            Polygon area {
                Point pt1(-50, -20)
                Point pt2(-20, 20)
                Point pt3(20, 20)
                Point pt4(50, -20)
                Point pt5(20, 20)
                Point pt6(50, -20)
                
            } 
            TaskAreaSummit sum1(map, 44.273987027194, 1.730245013810423)
            TaskAreaSummit sum2(map, 44.27606670575096, 1.727561722355053)
            TaskAreaSummit sum3(map, 44.27527283408215, 1.726114653154427)
            TaskAreaSummit sum4(map, 44.27424281023033, 1.726174130539044)
            TaskAreaSummit sum5(map, 44.27246047661149, 1.728857651044453)
            TaskAreaSummit sum6(map, 44.273987027194, 1.730245013810423)
            
            sum1.x =:> area.pt1.x
            sum1.y =:> area.pt1.y
            sum2.x =:> area.pt2.x
            sum2.y =:> area.pt2.y
            sum3.x =:> area.pt3.x
            sum3.y =:> area.pt3.y
            sum4.x =:> area.pt4.x
            sum4.y =:> area.pt4.y
            sum5.x =:> area.pt5.x
            sum5.y =:> area.pt5.y
            sum6.x =:> area.pt6.x
            sum6.y =:> area.pt6.y

        }
        Component roz_all{
            FillColor red (255, 0, 0)
            OutlineColor red_out (255, 0, 0)


            Polygon area {
                Point pt1(-50, -20)
                Point pt2(-20, 20)
                Point pt3(20, 20)
                Point pt4(50, -20)
                Point pt5(20, 20)
                Point pt6(50, -20)
                Point pt7(0,0)
                Point pt8(0,0)
                
            } 
            TaskAreaSummit sum1(map, 44.27525349811598, 1.726118963093772)
            TaskAreaSummit sum2(map, 44.27423777005773, 1.726178157676348)
            TaskAreaSummit sum3(map, 44.27407892648813, 1.726438070983625)
            TaskAreaSummit sum4(map, 44.27448241754246, 1.726920940512346)
            TaskAreaSummit sum5(map, 44.27468108197568, 1.726620377991235)
            TaskAreaSummit sum6(map, 44.27496089055605, 1.726439790427292)
            TaskAreaSummit sum7(map, 44.27522750414526, 1.726479182787166)
            TaskAreaSummit sum8(map, 44.27525349811598, 1.726118963093772)
            
            sum1.x =:> area.pt1.x
            sum1.y =:> area.pt1.y
            sum2.x =:> area.pt2.x
            sum2.y =:> area.pt2.y
            sum3.x =:> area.pt3.x
            sum3.y =:> area.pt3.y
            sum4.x =:> area.pt4.x
            sum4.y =:> area.pt4.y
            sum5.x =:> area.pt5.x
            sum5.y =:> area.pt5.y
            sum6.x =:> area.pt6.x
            sum6.y =:> area.pt6.y
            sum7.x =:> area.pt7.x
            sum7.y =:> area.pt7.y
            sum8.x =:> area.pt8.x
            sum8.y =:> area.pt8.y
        }
        Component roz_ground
        Component ffa
        Component nfz{
            FillColor yellow (180, 180, 20)
            OutlineColor yellow_out (180, 180, 20)
            Polygon area {
                Point pt1(-50, -20)
                Point pt2(-20, 20)
                Point pt3(20, 20)
                Point pt4(50, -20)
                Point pt5(20, 20)

                
            } 
            TaskAreaSummit sum1(map, 44.27246283621425, 1.728853168617424)
            TaskAreaSummit sum2(map, 44.27327264678223, 1.72959625069901)
            TaskAreaSummit sum3(map, 44.27342451002093, 1.729303127363166)
            TaskAreaSummit sum4(map, 44.27269229471858, 1.728522798768126)
            TaskAreaSummit sum5(map, 44.27246283621425, 1.728853168617424)
            
            sum1.x =:> area.pt1.x
            sum1.y =:> area.pt1.y
            sum2.x =:> area.pt2.x
            sum2.y =:> area.pt2.y
            sum3.x =:> area.pt3.x
            sum3.y =:> area.pt3.y
            sum4.x =:> area.pt4.x
            sum4.y =:> area.pt4.y
            sum5.x =:> area.pt5.x
            sum5.y =:> area.pt5.y
        }
        Component nfa
        Component rfa
        Component unknown


    }
    status =:> type.state
    

}