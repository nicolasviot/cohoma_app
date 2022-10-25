use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
Edge (int _id_src, int _id_dest, double _length, Process nodelist)
{
    Int id_src(_id_src)
    Int id_dest(_id_dest)
    Double length (_length)

    Double explored(0)
    DoubleFormatter df(0, 2)
    100 * explored =:> df.input 

    DoubleFormatter dm(0, 2)
    length =:> dm.input

    //Rarccos(sin(a)sin(b) + cos(a)cos(b)cos(c-d)) a = lata, b=lona, c =latb, d =lonb
    /*
    Double R(6171000)
    a aka nodelist.[$id_src].wpt.latinput
    b aka nodelist.[$id_src].wpt.lon
    c aka nodelist.[$id_src].wpt.lat
    d aka nodelist.[$id_src].wpt.lon

    arcos
    */

    //print ("Edge edge_" + id_src + "_" + id_dest + " (" + id_src + ", " + id_dest + ", " + length + ", nodes)\n")

    OutlineWidth outline_width (5)
    OutlineColor outline_color (234, 234, 234)
    OutlineCapStyle _ (1)

    Line edge (0, 0, 0, 0)

    x1 aka edge.x1
    x2 aka edge.x2
    y1 aka edge.y1
    y2 aka edge.y2

    Spike leave
    Spike enter

    FSM tooltip{
        State idle

        State entered{
            Timer t (500)
        }
        State display_tooltip{
            Translation t(0, 0)
            (x1 + x2) / 2 + 20 =:> t.tx
            (y1 + y2) / 2 =:> t.ty


            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            
            Rectangle bg (0, 0, 50, 20, 5, 5)
            FillColor _ (0, 0, 0)
            Text legend (0, 0, "length :  ")
            legend.x =:> bg.x
            legend.y - 12 =:> bg.y
            legend.width =:> bg.width
            legend.height =:> bg.height
            "Length : " + dm.output + "m, explored : " + df.output + "%" =:> legend.text
        }
        idle -> entered (enter)
        entered -> display_tooltip(entered.t.end)
        entered -> idle (leave)
        display_tooltip -> idle (leave)
    }


    // Transparent, only to detect enter/leave with 20 as outline width
    OutlineOpacity _ (0.0)
    //OutlineColor _ (255, 0, 0)
    OutlineWidth _ (20)

    Line mask_edge (0, 0, 0, 0)

    mask_edge.enter -> enter
    mask_edge.leave -> leave

    nodelist.[$id_src].wpt.screen_translation.tx =:> x1, mask_edge.x1
    nodelist.[$id_src].wpt.screen_translation.ty =:> y1, mask_edge.y1
    nodelist.[$id_dest].wpt.screen_translation.tx=:> x2, mask_edge.x2
    nodelist.[$id_dest].wpt.screen_translation.ty=:> y2, mask_edge.y2


    FSM fsm_mask {
        State idle {
            10 =: outline_width.width
        }
        State inside {
            20 =: outline_width.width
        }
        idle -> inside (mask_edge.enter)
        inside -> idle (mask_edge.leave)
    }

}