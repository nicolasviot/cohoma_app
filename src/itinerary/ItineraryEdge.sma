use core
use gui
use base

import graph.Edge

_native_code_
%{
    #include <iostream>
%}


_define_
ItineraryEdge (Process _context, Process _model, int _width) inherits Edge (_context, _model, _width)
{
    Spike click

    print ("View of itinerary edge: " + _model.node1.id + " -- " + _model.node2.id + " (" + _model.length_meters + ")\n")

    this.mask_release -> click
}