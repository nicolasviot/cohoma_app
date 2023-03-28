use core
use base
use display
use gui

_native_code_
%{
    //#include <math.h>
%}


// Manage pan but not zoom ! 
_define_
PanAndZoom (Process move, Process picking) {
    // input

    // move: e.g., frame.move // unfortunately, we need to know where the cursor is while zooming // FIXME put it in wheel event?..
    // press: e.g., frame.press
    // release: e.g., frame.release

    // output // should be connected e.g. to a Translation and a Scaling

    Double xpan (0)
    Double ypan (0)
    
    left_press_trigger aka picking.left.press
    right_press_trigger aka picking.right.press
    press aka picking.press
    release aka picking.release

    // we need to know where the cursor is // FIXME put it in wheel event?..
    mouseTracking = 1

    Double new_xpan (0)
    Double new_ypan (0)

    Double xlast (0)
    Double ylast (0)

    FSM pan_control {
        State idle
        State pressing {
            press.x =: xlast
            press.y =: ylast
        }
        State panning {
            AssignmentSequence seq (1) {
                xpan + (move.x - xlast) =: new_xpan
                ypan + (move.y - ylast) =: new_ypan
                new_xpan =: xpan
                new_ypan =: ypan

                move.x =: xlast
                move.y =: ylast
            }
            move -> seq
        }
        idle -> pressing (left_press_trigger)
        pressing -> idle (release)
        pressing -> panning (move)
        panning -> idle (release)
    }
}
