use core
use gui
use base

import PointModel
import ExclusionZoneModel
import LimaModel
import VehiculeModel
import SafetyPilotModel
import NodeModel
import EdgeModel
import TrapModel
import ItineraryModel

_native_code_
%{
    #include <iostream>
    using namespace std;

    //#include "core/property/text_property.h"
    //#include "core/property/double_property.h"
    //#include "core/property/int_property.h"
    //#include "core/property/bool_property.h"
    //#include "core/property/ref_property.h"

    //Process* get_edge_model (Process* self, int id_1, int id_2)
    Process* get_edge_model (Process* self, int index_1, int index_2)
	{
		Process* edge_model = nullptr;

        Process* nodes = self->find_child("nodes");
        int nodes_size = static_cast<IntProperty*>(nodes->find_child("size"))->get_value();

        Process* edges = self->find_child("edges");
        int edges_size = static_cast<IntProperty*>(edges->find_child("size"))->get_value();

        //if ( (index_1 < nodes_size) && (index_2 << nodes_size) )
        
        Process* node_1 = nodes->find_child(to_string(index_1 + 1));
        Process* node_2 = nodes->find_child(to_string(index_2 + 1));

        //int id_1 = static_cast<IntProperty*>(node_1->find_child("id"))->get_value();
        //int id_2 = static_cast<IntProperty*>(node_2->find_child("id"))->get_value();
        //cout << "Get edge model at " << index_1 << " (id " << id_1 << ") -- at " << index_2 << " (id " << id_2 << ")" << endl;

		for (int i = 1; i <= edges_size; i++)
    	{
			Process *edge = edges->find_child(to_string(i));
			
            if ( ( (node_1 == edge->find_child("node1")) || (node_1 == edge->find_child("node2")) )
              && ( (node_2 == edge->find_child("node1")) || (node_2 == edge->find_child("node2")) ) )
              {
                //cout << "Edge model " << index_1 << " -- " << index_2 << " FOUND !!!" << endl;
                edge_model = edge;
                break;
            }
		}
		return edge_model;
	}
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

    // EXCLUSION ZONES
    List zones

    // LIMAS
    List limas

    // NODES
    List nodes

    // EDGES
    List edges

    List itineraries {
        ItineraryModel itinerary1 (_context, "shortest")
        ItineraryModel itinerary2 (_context, "safest")
        ItineraryModel itinerary3 (_context, "tradeoff")
    }
    // FIXME: use aka
    shortest_itinerary aka itineraries.[1]
    safest_itinerary aka itineraries.[2]
    tradeoff_itinerary aka itineraries.[3]

    // TRAPS
    List traps

}