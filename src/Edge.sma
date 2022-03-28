use core
use gui
use base

_define_
Edge (int _id_src, int _id_dest, double _length, Process nodelist){

Int id_src(_id_src + 1)
Int id_dest(_id_dest + 1)
Double length(_length)


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

nodelist.[$id_src].wpt.c.cx =:> x1
nodelist.[$id_src].wpt.c.cy =:> y1
nodelist.[$id_dest].wpt.c.cx=:> x2
nodelist.[$id_dest].wpt.c.cy=:> y2




}