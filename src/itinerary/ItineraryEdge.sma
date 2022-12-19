use core
use gui
use base

import graph.Edge

_native_code_
%{
    #include <iostream>
%}


_define_
ItineraryEdge (Process _context, Process _model) inherits Edge (_context, _model)
{
    Spike click

    print ("View of itinerary edge: " + _model.node1.id + " -- " + _model.node2.id + " (" + _model.length + " m)\n")

    this.mask_release -> click
}