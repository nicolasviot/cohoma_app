use core
use gui
use base

import VehiculeModel
import SafetyPilotModel
import NodeModel
import EdgeModel
import TrapModel
import ItineraryModel

_native_code_
%{
    #include <iostream>

    //#include "core/property/text_property.h"
    //#include "core/property/double_property.h"
    //#include "core/property/int_property.h"
    //#include "core/property/bool_property.h"
    //#include "core/property/ref_property.h"
%}

_define_
ModelManager (Process _context, int _is_debug)
{
    context aka _context

    Bool IS_DEBUG (_is_debug)

    print ("Model Manager\n")

    Spike itineraries_updated

    Component vehicles {
        VehiculeModel vab (_context, "vab", "VAB", $_context.init_lat, $_context.init_lon, $_context.VAB_COL)
        VehiculeModel agilex1 (_context, "agilex1", "AGILEX 1", $_context.init_lat + 0.0005, $_context.init_lon, $_context.AGI_1_COL)
        VehiculeModel agilex2 (_context, "agilex2", "AGILEX 2", $_context.init_lat + 0.001, $_context.init_lon, $_context.AGI_2_COL)
        VehiculeModel lynx (_context, "lynx", "LYNX", $_context.init_lat, $_context.init_lon + 0.001, $_context.LYNX_COL)
        VehiculeModel spot (_context, "spot", "SPOT", $_context.init_lat + 0.001 , $_context.init_lon + 0.001, $_context.SPOT_COL)
        VehiculeModel drone (_context, "drone", "DRONE", $_context.init_lat + 0.0015 , $_context.init_lon + 0.0015, $_context.DRONE_COL)
    }

    Component safety_pilots {
        // Unmanned Aerial Vehicle
        //SafetyPilotModel uav (_context, "uav", "UAV", $_context.init_lat, $_context.init_lon - 0.005, $_context.UAV_COL)
        SafetyPilotModel drone_safety_pilot (_context, "uav", "UAV", $_context.init_lat, $_context.init_lon - 0.005, $_context.UAV_COL)

        // Unmanned Ground Vehicle
        //SafetyPilotModel ugv (_context, "ugv", "UGV", $_context.init_lat, $_context.init_lon + 0.005, $_context.UGV_COL)
        SafetyPilotModel ground_safety_pilot (_context, "ugv", "UGV", $_context.init_lat, $_context.init_lon + 0.005, $_context.UGV_COL)
    }

    List itineraries {
        ItineraryModel itinerary1 (_context, "shortest")
        ItineraryModel itinerary2 (_context, "safest")
        ItineraryModel itinerary3 (_context, "tradeoff")
    }
    // FIXME: use aka
    shortest_itinerary aka itineraries.[1]
    safest_itinerary aka itineraries.[2]
    tradeoff_itinerary aka itineraries.[3]


    // NODES
    List nodes

    // EDGES
    List edges

    // TRAPS
    List traps

}