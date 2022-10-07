use core
use gui
use base

import VehiculeModel
import SafetyPilotModel
import NodeModel
import TrapModel

_native_code_
%{
    #include "cpp/coords-utils.h"
    //#include "core/property/text_property.h"
    #include "core/property/double_property.h"
    #include "core/property/int_property.h"
    //#include "core/property/bool_property.h"
    //#include "core/property/ref_property.h"
%}

_define_
ModelManager (Process _context)
{

    //context aka _context

    Component vehicules {
        VehiculeModel vab (_context, "vab", "VAB", $_context.init_lat, $_context.init_lon, $_context.VAB_COL)
        VehiculeModel agilex1 (_context, "agilex1", "AGILEX 1", $_context.init_lat + 0.0005, $_context.init_lon, $_context.AGI_1_COL)
        VehiculeModel agilex2 (_context, "agilex2", "AGILEX 2", $_context.init_lat + 0.001, $_context.init_lon, $_context.AGI_2_COL)
        VehiculeModel lynx (_context, "lynx", "LYNX", $_context.init_lat, $_context.init_lon + 0.001, $_context.LYNX_COL)
        VehiculeModel spot (_context, "spot", "SPOT", $_context.init_lat + 0.001 , $_context.init_lon + 0.001, $_context.SPOT_COL)
        VehiculeModel drone (_context, "drone", "DRONE", $_context.init_lat + 0.0015 , $_context.init_lon + 0.0015, $_context.DRONE_COL)
    }

    Component safety_pilots {
        // Unmanned Aerial Vehicle
        SafetyPilotModel uav (_context, "uav", "UAV", $_context.init_lat, $_context.init_lon - 0.005, $_context.UAV_COL)
        //SafetyPilotModel drone_safety_pilot (_context, "drone_safety_pilot", "UAV", $_context.init_lat, $_context.init_lon - 0.005, $_context.UAV_COL)

        // Unmanned Ground Vehicle
        SafetyPilotModel ugv (_context, "ugv", "UGV", $_context.init_lat, $_context.init_lon + 0.005, $_context.UGV_COL)
        //SafetyPilotModel ground_safety_pilot (_context, "ground_safety_pilot", "UGV", $_context.init_lat, $_context.init_lon + 0.005, $_context.UGV_COL)
    }

    List nodes {
        NodeModel node0 (_context, 1, "n_01", 43.316021818382886, 1.4041900634765625, 0.0, 0)
        NodeModel node1 (_context, 2, "n_02", 43.316006206187375, 1.4047694206237793, 0.0, 0)
	}

    Component traps {
        TrapModel trap (_context, 23, 43.315893, 1.403865)
    }
}