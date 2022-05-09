use core
use gui
use base

_define_
Edge (int _id_src, int _id_dest, double _length, Process nodelist){

Int id_src(_id_src)
Int id_dest(_id_dest)
Double length(_length)
Double explored(0)
DoubleFormatter df(0, 2)
explored =:> df.input 

//Rarccos(sin(a)sin(b) + cos(a)cos(b)cos(c-d)) a = lata, b=lona, c =latb, d =lonb
/*
Double R(6171000)
a aka nodelist.[$id_src].wpt.lat
b aka nodelist.[$id_src].wpt.lon
c aka nodelist.[$id_src].wpt.lat
d aka nodelist.[$id_src].wpt.lon

arcos

*/
OutlineWidth width (5)
OutlineColor color (234, 234, 234)
//OutlineCapStyle _ (0)
OutlineJoinStyle _(0)
Translation pos (0, 0)
nodelist.[$id_src].wpt.pos.tx =:> pos.tx

nodelist.[$id_src].wpt.pos.ty =:> pos.ty

Line edge (0, 0, 0, 0)

x1 aka edge.x1
x2 aka edge.x2
y1 aka edge.y1
y2 aka edge.y2

nodelist.[$id_src].wpt.screen_translation.tx =:> x1
nodelist.[$id_src].wpt.screen_translation.ty =:> y1
nodelist.[$id_dest].wpt.screen_translation.tx=:> x2
nodelist.[$id_dest].wpt.screen_translation.ty=:> y2

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
        Text legend (0, 0, "lenght :  ")
        legend.x =:> bg.x
        legend.y - 12 =:> bg.y
        legend.width =:> bg.width
        legend.height =:> bg.height
        "Length : " + toString(length) + "m, explored : " + df.output + "%" =:> legend.text


    }
    idle -> entered (enter)
    entered -> display_tooltip(entered.t.end)
    entered -> idle (leave)
    display_tooltip -> idle (leave)
}


OutlineOpacity _(0)
//OutlineColor _(255, 0, 0)
OutlineWidth outer_width (20)
Line outerEdge (0, 0, 0, 0)
outerEdge.enter -> enter
outerEdge.leave -> leave
outer_x1 aka outerEdge.x1
outer_x2 aka outerEdge.x2
outer_y1 aka outerEdge.y1
outer_y2 aka outerEdge.y2

nodelist.[$id_src].wpt.screen_translation.tx =:> outer_x1
nodelist.[$id_src].wpt.screen_translation.ty =:> outer_y1
nodelist.[$id_dest].wpt.screen_translation.tx=:> outer_x2
nodelist.[$id_dest].wpt.screen_translation.ty=:> outer_y2


FSM enterLeave {
    State idle {
        10 =: width.width
    }
    State inside {
        20 =: width.width
    }
    idle -> inside (outerEdge.enter)
    inside -> idle (outerEdge.leave)
}



}