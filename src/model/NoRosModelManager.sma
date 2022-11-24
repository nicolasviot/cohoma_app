use core
use gui
use base

import VehiculeModel
import SafetyPilotModel
import NodeModel
import EdgeModel
import TrapModel
import ItineraryModel
import ModelManager

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
NoRosModelManager (Process _context, int _is_debug) inherits ModelManager (_context, _is_debug)
{
    print ("No ROS Model Manager\n")

    // **************************************************************************************************
    //
    //  ITINERARIES
    //
    // **************************************************************************************************
    
    "e5bd15ed-d7c3-4ddd-b80a-a1a8121f9e7e" =: this.shortest_itinerary.uid
    "Planning shortest OK: a path including PPOs was found with cost 341.001" =: this.shortest_itinerary.description_input

    "b5ac3bb1-593c-4dff-b1bd-9dcca5c39ab2" =: this.safest_itinerary.uid
    "Planning safest OK: a path including PPOs was found with cost 501.248" =: this.safest_itinerary.description_input

    "4e5aa5e6-0ed4-430f-9094-d0e8295594e7" =: this.tradeoff_itinerary.uid
    "Planning tradeoff OK: a path including PPOs was found with cost 450.050" =: this.tradeoff_itinerary.description_input


    // **************************************************************************************************
    //
    //  NODES
    //
    // **************************************************************************************************

    addChildrenTo this.nodes {
        //NodeModel (Process _context, int _id, int _phase, string _label, double _lat, double _lon, double _alt, int _is_mandatory)
        NodeModel node106 (_context, 0, 4, "", 48.8664, 1.88506, 0, 0)
        NodeModel node105 (_context, 1, 4, "", 48.8662, 1.88534, 0, 0)
        NodeModel node104 (_context, 2, 4, "", 48.8667, 1.88602, 0, 0)
        NodeModel node103 (_context, 3, 4, "", 48.8654, 1.88662, 0, 0)
        NodeModel node102 (_context, 4, 4, "", 48.8645, 1.88433, 0, 0)
        NodeModel node101 (_context, 5, 4, "", 48.8653, 1.88536, 0, 0)
        NodeModel node100 (_context, 6, 4, "", 48.8653, 1.8851, 0, 0)
        NodeModel node99 (_context, 7, 4, "", 48.8654, 1.88504, 0, 0)
        NodeModel node98 (_context, 8, 3, "", 48.8631, 1.88716, 0, 0)
        NodeModel node97 (_context, 9, 3, "", 48.8646, 1.8866, 0, 0)
        NodeModel node96 (_context, 10, 2, "", 48.863, 1.88879, 0, 0)
        NodeModel node95 (_context, 11, 2, "", 48.8628, 1.88937, 0, 0)
        NodeModel node94 (_context, 12, 2, "", 48.8626, 1.88896, 0, 0)
        NodeModel node93 (_context, 13, 2, "", 48.8626, 1.88836, 0, 0)
        NodeModel node92 (_context, 14, 2, "", 48.8625, 1.88815, 0, 0)
        NodeModel node91 (_context, 15, 2, "", 48.8628, 1.88843, 0, 0)
        NodeModel node90 (_context, 16, 3, "", 48.863, 1.88838, 0, 0)
        NodeModel node89 (_context, 17, 2, "", 48.8627, 1.8881, 0, 0)
        NodeModel node88 (_context, 18, 2, "", 48.8608, 1.88694, 0, 0)
        NodeModel node87 (_context, 19, 1, "", 48.8597, 1.88709, 0, 0)
        NodeModel node86 (_context, 20, 1, "", 48.8603, 1.88699, 0, 0)
        NodeModel node85 (_context, 21, 1, "", 48.8603, 1.88722, 0, 0)
        NodeModel node84 (_context, 22, 1, "", 48.861, 1.89203, 0, 0)
        NodeModel node83 (_context, 23, 1, "PPO10", 48.8607, 1.88702, 0, 1)
        NodeModel node82 (_context, 24, 2, "PPO20", 48.8632, 1.88971, 0, 1)
        NodeModel node81 (_context, 25, 3, "PPO30", 48.8647, 1.88677, 0, 1)
        NodeModel node80 (_context, 26, 4, "PPO40", 48.8662, 1.88387, 0, 1)
        NodeModel node79 (_context, 27, 0, "depart", 48.8611, 1.89331, 0, 1)
        NodeModel node78 (_context, 28, 1, "T101", 48.861, 1.89266, 0, 0)
        NodeModel node77 (_context, 29, 1, "T102", 48.8608, 1.8904, 0, 0)
        NodeModel node76 (_context, 30, 1, "T103", 48.8603, 1.88755, 0, 0)
        NodeModel node75 (_context, 31, 2, "T201", 48.8619, 1.88468, 0, 0)
        NodeModel node74 (_context, 32, 2, "T202", 48.8622, 1.88563, 0, 0)
        NodeModel node73 (_context, 33, 2, "T203", 48.8614, 1.8866, 0, 0)
        NodeModel node72 (_context, 34, 2, "T204", 48.8609, 1.88735, 0, 0)
        NodeModel node71 (_context, 35, 2, "T205", 48.8622, 1.88666, 0, 0)
        NodeModel node70 (_context, 36, 2, "T206", 48.8617, 1.88715, 0, 0)
        NodeModel node69 (_context, 37, 2, "T207", 48.8624, 1.88741, 0, 0)
        NodeModel node68 (_context, 38, 2, "T208", 48.862, 1.88769, 0, 0)
        NodeModel node67 (_context, 39, 2, "T209", 48.8625, 1.88788, 0, 0)
        NodeModel node66 (_context, 40, 2, "T210", 48.8623, 1.88832, 0, 0)
        NodeModel node65 (_context, 41, 2, "T211", 48.8622, 1.88925, 0, 0)
        NodeModel node64 (_context, 42, 2, "T212", 48.8627, 1.88883, 0, 0)
        NodeModel node63 (_context, 43, 2, "T213", 48.863, 1.8891, 0, 0)
        NodeModel node62 (_context, 44, 2, "T214", 48.8627, 1.88946, 0, 0)
        NodeModel node61 (_context, 45, 2, "T215", 48.8624, 1.88974, 0, 0)
        NodeModel node60 (_context, 46, 2, "T216", 48.8628, 1.88996, 0, 0)
        NodeModel node59 (_context, 47, 3, "T301", 48.8631, 1.88509, 0, 0)
        NodeModel node58 (_context, 48, 3, "T302", 48.863, 1.88569, 0, 0)
        NodeModel node57 (_context, 49, 3, "T303", 48.8637, 1.88599, 0, 0)
        NodeModel node56 (_context, 50, 3, "T304", 48.8645, 1.88698, 0, 0)
        NodeModel node55 (_context, 51, 3, "T305", 48.8643, 1.88743, 0, 0)
        NodeModel node54 (_context, 52, 3, "T306", 48.8639, 1.88761, 0, 0)
        NodeModel node53 (_context, 53, 3, "T307", 48.8636, 1.88802, 0, 0)
        NodeModel node52 (_context, 54, 3, "T308", 48.8629, 1.88813, 0, 0)
        NodeModel node51 (_context, 55, 3, "T309", 48.8639, 1.88811, 0, 0)
        NodeModel node50 (_context, 56, 3, "T310", 48.8648, 1.88741, 0, 0)
        NodeModel node49 (_context, 57, 3, "T311", 48.8644, 1.88859, 0, 0)
        NodeModel node48 (_context, 58, 3, "T312", 48.8635, 1.88897, 0, 0)
        NodeModel node47 (_context, 59, 3, "T313", 48.8638, 1.88922, 0, 0)
        NodeModel node46 (_context, 60, 3, "T314", 48.864, 1.88964, 0, 0)
        NodeModel node45 (_context, 61, 4, "T401", 48.8651, 1.88414, 0, 0)
        NodeModel node44 (_context, 62, 4, "T402", 48.8644, 1.88512, 0, 0)
        NodeModel node43 (_context, 63, 4, "T403", 48.8653, 1.88463, 0, 0)
        NodeModel node42 (_context, 64, 4, "T404", 48.8658, 1.88435, 0, 0)
        NodeModel node41 (_context, 65, 4, "T405", 48.8663, 1.88454, 0, 0)
        NodeModel node40 (_context, 66, 4, "T406", 48.8657, 1.88487, 0, 0)
        NodeModel node39 (_context, 67, 4, "T407", 48.865, 1.88521, 0, 0)
        NodeModel node38 (_context, 68, 4, "T408", 48.8651, 1.88559, 0, 0)
        NodeModel node37 (_context, 69, 4, "T409", 48.8653, 1.88556, 0, 0)
        NodeModel node36 (_context, 70, 4, "T410", 48.865, 1.88608, 0, 0)
        NodeModel node35 (_context, 71, 4, "T411", 48.8655, 1.88677, 0, 0)
        NodeModel node34 (_context, 72, 4, "T412", 48.8655, 1.88622, 0, 0)
        NodeModel node33 (_context, 73, 4, "T413", 48.8661, 1.88517, 0, 0)
        NodeModel node32 (_context, 74, 4, "T414", 48.8665, 1.88496, 0, 0)
        NodeModel node31 (_context, 75, 4, "T415", 48.8668, 1.88585, 0, 0)
        NodeModel node30 (_context, 76, 2, "", 48.8624, 1.88598, 0, 0)
        NodeModel node29 (_context, 77, 2, "", 48.8621, 1.88525, 0, 0)
        NodeModel node28 (_context, 78, 2, "", 48.861, 1.88718, 0, 0)
        NodeModel node27 (_context, 79, 2, "", 48.8614, 1.88727, 0, 0)
        NodeModel node26 (_context, 80, 2, "", 48.8616, 1.88737, 0, 0)
        NodeModel node25 (_context, 81, 2, "", 48.8613, 1.88707, 0, 0)
        NodeModel node24 (_context, 82, 2, "", 48.8625, 1.88765, 0, 0)
        NodeModel node23 (_context, 83, 2, "", 48.8624, 1.88804, 0, 0)
        NodeModel node22 (_context, 84, 2, "", 48.8624, 1.88718, 0, 0)
        NodeModel node21 (_context, 85, 2, "", 48.8621, 1.88731, 0, 0)
        NodeModel node20 (_context, 86, 3, "", 48.8627, 1.88742, 0, 0)
        NodeModel node19 (_context, 87, 3, "", 48.8627, 1.88789, 0, 0)
        NodeModel node18 (_context, 88, 3, "", 48.8632, 1.88823, 0, 0)
        NodeModel node17 (_context, 89, 3, "", 48.8634, 1.88913, 0, 0)
        NodeModel node16 (_context, 90, 3, "", 48.8636, 1.88958, 0, 0)
        NodeModel node15 (_context, 91, 3, "", 48.8636, 1.89008, 0, 0)
        NodeModel node14 (_context, 92, 3, "", 48.8638, 1.88941, 0, 0)
        NodeModel node13 (_context, 93, 3, "", 48.8641, 1.88911, 0, 0)
        NodeModel node12 (_context, 94, 3, "", 48.8639, 1.88853, 0, 0)
        NodeModel node11 (_context, 95, 3, "", 48.8641, 1.88778, 0, 0)
        NodeModel node10 (_context, 96, 3, "", 48.8643, 1.88677, 0, 0)
        NodeModel node9 (_context, 97, 3, "", 48.865, 1.88763, 0, 0)
        NodeModel node8 (_context, 98, 3, "", 48.8642, 1.88519, 0, 0)
        NodeModel node7 (_context, 99, 3, "", 48.8643, 1.88555, 0, 0)
        NodeModel node6 (_context, 100, 3, "", 48.8641, 1.8848, 0, 0)
        NodeModel node5 (_context, 101, 3, "", 48.865, 1.88793, 0, 0)
        NodeModel node4 (_context, 102, 3, "", 48.8643, 1.88843, 0, 0)
        NodeModel node3 (_context, 103, 4, "", 48.8659, 1.88456, 0, 0)
        NodeModel node2 (_context, 104, 4, "", 48.8653, 1.88409, 0, 0)
        NodeModel node1 (_context, 105, 4, "", 48.8662, 1.88452, 0, 0)
        NodeModel node0 (_context, 106, 4, "", 48.8661, 1.88491, 0, 0)
    }


    // **************************************************************************************************
    //
    //  EDGES
    //
    // **************************************************************************************************

    addChildrenTo this.edges {
        EdgeModel edge_28_22 (this.nodes.[29], this.nodes.[23], 46.2805)
        EdgeModel edge_27_28 (this.nodes.[28], this.nodes.[29], 48.5232)
        EdgeModel edge_22_29 (this.nodes[23], this.nodes[30], 122.646)
    }


    // **************************************************************************************************
    //
    //  TRAPS
    //
    // **************************************************************************************************

    Spike add_trap1
    Spike add_trap2
    Spike set_trap1
    Spike set_trap2

    add_trap1 -> (this) {
        addChildrenTo this.traps {
            TrapModel debug_trap1 (this.context, 199, $this.context.init_lat, $this.context.init_lon - 0.0015, null)
        }
    }
    add_trap2 -> (this) {
        addChildrenTo this.traps {
            TrapModel debug_trap2 (this.context, 223, $this.context.init_lat + 0.0005, $this.context.init_lon - 0.003, null)
        }
    }

    set_trap1 -> (this) {
        if (this.traps.size > 0) {
            this.traps.[1].description = "Ceci est le trap 1"
            this.traps.[1].contact_mode = 1
            this.traps.[1].remotely_deactivate = 1
        }
    }
    set_trap2 -> (this) {
        if (this.traps.size > 1) {
            this.traps.[2].description = "Ceci est le trap 2"
            this.traps.[2].contact_mode = 2
            this.traps.[2].contact_deactivate = 1
        }
    }

}