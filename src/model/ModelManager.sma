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
ModelManager (Process _context, int _is_debug)
{

    //context aka _context
    Bool IS_DEBUG (_is_debug)

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

    /*if (_is_debug) {
        addChildrenTo nodes {
            NodeModel node1 (_context, 48.866366, 1.885056, 0.0, 0, "", 1)
            NodeModel node2 (_context, 48.866246, 1.885335, 0.0, 0, "", 2)
            NodeModel node3 (_context, 48.866733, 1.886022, 0.0, 0, "", 3)
            NodeModel node4 (_context, 48.865385, 1.886623, 0.0, 0, "", 4)
            NodeModel node5 (_context, 48.864482, 1.884327, 0.0, 0, "", 5)
            NodeModel node6 (_context, 48.865308, 1.885357, 0.0, 0, "", 6)
            NodeModel node7 (_context, 48.865294, 1.885099, 0.0, 0, "", 7)
            NodeModel node8 (_context, 48.865421, 1.885035, 0.0, 0, "", 8)
            NodeModel node9 (_context, 48.863127, 1.887159, 0.0, 0, "", 9)
            NodeModel node10 (_context, 48.864644, 1.886601, 0.0, 0, "", 10)
            NodeModel node11 (_context, 48.862964, 1.888790, 0.0, 0, "", 11)
            NodeModel node12 (_context, 48.862774, 1.889369, 0.0, 0, "", 12)
            NodeModel node13 (_context, 48.862633, 1.888962, 0.0, 0, "", 13)
            NodeModel node14 (_context, 48.862626, 1.888361, 0.0, 0, "", 14)
            NodeModel node15 (_context, 48.862527, 1.888146, 0.0, 0, "", 15)
            NodeModel node16 (_context, 48.862823, 1.888425, 0.0, 0, "", 16)
            NodeModel node17 (_context, 48.862964, 1.888382, 0.0, 0, "", 17)
            NodeModel node18 (_context, 48.862668, 1.888103, 0.0, 0, "", 18)
            NodeModel node19 (_context, 48.860833, 1.886945, 0.0, 0, "", 19)
            NodeModel node20 (_context, 48.859703, 1.887095, 0.0, 0, "", 20)
            NodeModel node21 (_context, 48.860268, 1.886988, 0.0, 0, "", 21)
            NodeModel node22 (_context, 48.860303, 1.887224, 0.0, 0, "", 22)
            NodeModel node23 (_context, 48.861023, 1.892030, 0.0, 0, "", 23)
            NodeModel node24 (_context, 48.860684, 1.887024, 0.0, 1, "PPO10", 24)
            NodeModel node25 (_context, 48.863247, 1.889708, 0.0, 1, "PPO20", 25)
            NodeModel node26 (_context, 48.864712, 1.886771, 0.0, 1, "PPO30", 26)
            NodeModel node27 (_context, 48.866231, 1.883874, 0.0, 1, "PPO40", 27)
            NodeModel node28 (_context, 48.861095, 1.893314, 0.0, 1, "depart", 28)
            NodeModel node29 (_context, 48.861026, 1.892661, 0.0, 0, "T101", 29)
            NodeModel node30 (_context, 48.860770, 1.890403, 0.0, 0, "T102", 30)
            NodeModel node31 (_context, 48.860347, 1.887549, 0.0, 0, "T103", 31)
            NodeModel node32 (_context, 48.861903, 1.884679, 0.0, 0, "T201", 32)
            NodeModel node33 (_context, 48.862245, 1.885626, 0.0, 0, "T202", 33)
            NodeModel node34 (_context, 48.861444, 1.886598, 0.0, 0, "T203", 34)
            NodeModel node35 (_context, 48.860894, 1.887346, 0.0, 0, "T204", 35)
            NodeModel node36 (_context, 48.862156, 1.886664, 0.0, 0, "T205", 36)
            NodeModel node37 (_context, 48.861729, 1.887151, 0.0, 0, "T206", 37)
            NodeModel node38 (_context, 48.862424, 1.887408, 0.0, 0, "T207", 38)
            NodeModel node39 (_context, 48.861977, 1.887691, 0.0, 0, "T208", 39)
            NodeModel node40 (_context, 48.862518, 1.887883, 0.0, 0, "T209", 40)
            NodeModel node41 (_context, 48.862289, 1.888325, 0.0, 0, "T210", 41)
            NodeModel node42 (_context, 48.862163, 1.889254, 0.0, 0, "T211", 42)
            NodeModel node43 (_context, 48.862725, 1.888833, 0.0, 0, "T212", 43)
            NodeModel node44 (_context, 48.863043, 1.889099, 0.0, 0, "T213", 44)
            NodeModel node45 (_context, 48.862678, 1.889461, 0.0, 0, "T214", 45)
            NodeModel node46 (_context, 48.862383, 1.889740, 0.0, 0, "T215", 46)
            NodeModel node47 (_context, 48.862763, 1.889964, 0.0, 0, "T216", 47)
            NodeModel node48 (_context, 48.863058, 1.885090, 0.0, 0, "T301", 48)
            NodeModel node49 (_context, 48.863046, 1.885690, 0.0, 0, "T302", 49)
            NodeModel node50 (_context, 48.863652, 1.885990, 0.0, 0, "T303", 50)
            NodeModel node51 (_context, 48.864453, 1.886981, 0.0, 0, "T304", 51)
            NodeModel node52 (_context, 48.864295, 1.887435, 0.0, 0, "T305", 52)
            NodeModel node53 (_context, 48.863883, 1.887607, 0.0, 0, "T306", 53)
            NodeModel node54 (_context, 48.863599, 1.888023, 0.0, 0, "T307", 54)
            NodeModel node55 (_context, 48.862863, 1.888135, 0.0, 0, "T308", 55)
            NodeModel node56 (_context, 48.863924, 1.888111, 0.0, 0, "T309", 56)
            NodeModel node57 (_context, 48.864835, 1.887409, 0.0, 0, "T310", 57)
            NodeModel node58 (_context, 48.864414, 1.888591, 0.0, 0, "T311", 58)
            NodeModel node59 (_context, 48.863518, 1.888965, 0.0, 0, "T312", 59)
            NodeModel node60 (_context, 48.863836, 1.889217, 0.0, 0, "T313", 60)
            NodeModel node61 (_context, 48.863975, 1.889637, 0.0, 0, "T314", 61)
            NodeModel node62 (_context, 48.865145, 1.884143, 0.0, 0, "T401", 62)
            NodeModel node63 (_context, 48.864363, 1.885115, 0.0, 0, "T402", 63)
            NodeModel node64 (_context, 48.865267, 1.884632, 0.0, 0, "T403", 64)
            NodeModel node65 (_context, 48.865840, 1.884346, 0.0, 0, "T404", 65)
            NodeModel node66 (_context, 48.866336, 1.884540, 0.0, 0, "T405", 66)
            NodeModel node67 (_context, 48.865674, 1.884868, 0.0, 0, "T406", 67)
            NodeModel node68 (_context, 48.865029, 1.885209, 0.0, 0, "T407", 68)
            NodeModel node69 (_context, 48.865087, 1.885590, 0.0, 0, "T408", 69)
            NodeModel node70 (_context, 48.865330, 1.885557, 0.0, 0, "T409", 70)
            NodeModel node71 (_context, 48.865020, 1.886082, 0.0, 0, "T410", 71)
            NodeModel node72 (_context, 48.865494, 1.886767, 0.0, 0, "T411", 72)
            NodeModel node73 (_context, 48.865489, 1.886222, 0.0, 0, "T412", 73)
            NodeModel node74 (_context, 48.866127, 1.885171, 0.0, 0, "T413", 74)
            NodeModel node75 (_context, 48.866529, 1.884958, 0.0, 0, "T414", 75)
            NodeModel node76 (_context, 48.866835, 1.885851, 0.0, 0, "T415", 76)
            NodeModel node77 (_context, 48.862350, 1.885979, 0.0, 0, "", 77)
            NodeModel node78 (_context, 48.862139, 1.885250, 0.0, 0, "", 78)
            NodeModel node79 (_context, 48.861045, 1.887181, 0.0, 0, "", 79)
            NodeModel node80 (_context, 48.861440, 1.887267, 0.0, 0, "", 80)
            NodeModel node81 (_context, 48.861616, 1.887374, 0.0, 0, "", 81)
            NodeModel node82 (_context, 48.861334, 1.887074, 0.0, 0, "", 82)
            NodeModel node83 (_context, 48.862484, 1.887653, 0.0, 0, "", 83)
            NodeModel node84 (_context, 48.862393, 1.888039, 0.0, 0, "", 84)
            NodeModel node85 (_context, 48.862407, 1.887181, 0.0, 0, "", 85)
            NodeModel node86 (_context, 48.862068, 1.887310, 0.0, 0, "", 86)
            NodeModel node87 (_context, 48.862689, 1.887417, 0.0, 0, "", 87)
            NodeModel node88 (_context, 48.862746, 1.887889, 0.0, 0, "", 88)
            NodeModel node89 (_context, 48.863183, 1.888232, 0.0, 0, "", 89)
            NodeModel node90 (_context, 48.863395, 1.889133, 0.0, 0, "", 90)
            NodeModel node91 (_context, 48.863564, 1.889584, 0.0, 0, "", 91)
            NodeModel node92 (_context, 48.863642, 1.890078, 0.0, 0, "", 92)
            NodeModel node93 (_context, 48.863811, 1.889412, 0.0, 0, "", 93)
            NodeModel node94 (_context, 48.864143, 1.889112, 0.0, 0, "", 94)
            NodeModel node95 (_context, 48.863931, 1.888533, 0.0, 0, "", 95)
            NodeModel node96 (_context, 48.864065, 1.887782, 0.0, 0, "", 96)
            NodeModel node97 (_context, 48.864284, 1.886773, 0.0, 0, "", 97)
            NodeModel node98 (_context, 48.864955, 1.887631, 0.0, 0, "", 98)
            NodeModel node99 (_context, 48.864228, 1.885185, 0.0, 0, "", 99)
            NodeModel node100 (_context, 48.864334, 1.885550, 0.0, 0, "", 100)
            NodeModel node101 (_context, 48.864108, 1.884799, 0.0, 0, "", 101)
            NodeModel node102 (_context, 48.865025, 1.887932, 0.0, 0, "", 102)
            NodeModel node103 (_context, 48.864305, 1.888425, 0.0, 0, "", 103)
            NodeModel node104 (_context, 48.865943, 1.884563, 0.0, 0, "", 104)
            NodeModel node105 (_context, 48.865329, 1.884091, 0.0, 0, "", 105)
            NodeModel node106 (_context, 48.866211, 1.884520, 0.0, 0, "", 106)
            NodeModel node107 (_context, 48.866119, 1.884906, 0.0, 0, "", 107)
        }
    }*/

    List edges {
		
	}

    List traps {
        TrapModel trap (_context, 23, $_context.init_lat, $_context.init_lon - 0.0015)
    }
}