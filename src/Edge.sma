use core
use gui
use base

_define_
Edge (int _id_src, int _id_dest, double _length, Process nodelist){

Int id_src(_id_src)
Int id_dest(_id_dest)
Double length(_length)
Double explored(0)

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
OutlineColor color (180, 90, 140)
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

OutlineOpacity _(0)
//OutlineColor _(255, 0, 0)
OutlineWidth outer_width (20)
Line outerEdge (0, 0, 0, 0)
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