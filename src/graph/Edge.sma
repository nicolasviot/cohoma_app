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

    Int default_width (8)
	Int inside_width (16)
	Int mask_width (16)

    DoubleFormatter format (0, 2)
    _model.length =:> format.input

    //print ("View of edge: " + _model.node1.id + " -- " + _model.node2.id + " (" + _model.length + " m)\n")

    OutlineCapStyle _ (1)

    Component bg {
        // "Transparent", only to detect enter/leave with 15 as outline width
        NoOutline _
        PickOutline _
        OutlineWidth _ ($mask_width)

        Line mask (0, 0, 0, 0)
    }
    mask_release aka bg.mask.release

    OutlineWidth outline_width ($default_width)
    //OutlineColor outline_color ($_context.EDGE_COLOR)

    Line edge (0, 0, 0, 0)


    FSM fsm {
        State idle {
            default_width =: outline_width.width
        }

        State entered {
            Timer t (500)
            inside_width =: outline_width.width
        }

        State display_tooltip {
            //20 =: outline_width.width

            Translation tr (0, 0)
            (edge.x1 + edge.x2) / 2 =:> tr.tx
            (edge.y1 + edge.y2) / 2 =:> tr.ty

            OutlineWidth _ (1)
            OutlineColor _ (#777777)
            FillOpacity fo (0.8)
            FillColor light_grey (204, 204, 204)
            Rectangle bg (0, 0, 1, 20, 5, 5)
            
            FillColor black (#000000)
            FontSize _ (5, 12)
            FontWeight _ (DJN_NORMAL)
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


    _model.node1.dx_in_map =:> edge.x1, bg.mask.x1
    _model.node1.dy_in_map =:> edge.y1, bg.mask.y1

    _model.node2.dx_in_map=:> edge.x2, bg.mask.x2
    _model.node2.dy_in_map=:> edge.y2, bg.mask.y2

}