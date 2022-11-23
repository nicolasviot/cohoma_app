use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
Edge (Process _context, Process _model)
{
    //context aka _context
    model aka _model

    //Double explored(0)
    //DoubleFormatter df(0, 2)
    //100 * explored =:> df.input 

    DoubleFormatter format (0, 2)
    _model.length =:> format.input

    //print ("View of edge: " + _model.node1.id + " -- " + _model.node2.id + " (" + _model.length + " m)\n")

    OutlineCapStyle _ (1)

    Component bg {
        // "Transparent", only to detect enter/leave with 20 as outline width
        NoOutline _
        PickOutline _
        OutlineWidth _ (20)

        Line mask (0, 0, 0, 0)
    }

    OutlineWidth outline_width (10)
    OutlineColor outline_color ($_context.EDGE_COLOR)

    Line edge (0, 0, 0, 0)


    FSM tooltip {
        State idle {
            10 =: outline_width.width
        }

        State entered {
            Timer t (500)
            20 =: outline_width.width
        }

        State display_tooltip {
            //20 =: outline_width.width

            Translation t (0, 0)
            (edge.x1 + edge.x2) / 2 =:> t.tx
            (edge.y1 + edge.y2) / 2 =:> t.ty

            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            OutlineColor _ ($_context.EDGE_COLOR)
            OutlineWidth _ (1)
            Rectangle bg (0, 0, 50, 20, 5, 5)
            
            FillColor black (#000000)
            Text legend (3, 15, "0 m")
            legend.width + 6 =:> bg.width
            //legend.height + 2 =:> bg.height

            format.output + " m" =:> legend.text
        }
        idle -> entered (bg.mask.enter)
        entered -> display_tooltip (entered.t.end)
        entered -> idle (bg.mask.leave)
        display_tooltip -> idle (bg.mask.leave)
    }


    _model.node1.dx_map =:> edge.x1, bg.mask.x1
    _model.node1.dy_map =:> edge.y1, bg.mask.y1

    _model.node2.dx_map=:> edge.x2, bg.mask.x2
    _model.node2.dy_map=:> edge.y2, bg.mask.y2

}