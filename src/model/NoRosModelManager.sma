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

    Spike set_shortest_itinerary
    Spike set_safest_itinerary
    Spike set_tradeoff_itinerary

    set_shortest_itinerary -> {
        "e5bd15ed-d7c3-4ddd-b80a-a1a8121f9e7e" =: this.shortest_itinerary.uid
        "Planning shortest OK: a path including PPOs was found with cost 341.001" =: this.shortest_itinerary.description_input
    }
    set_shortest_itinerary -> na_set_shortest:(this) {
        addChildrenTo this.shortest_itinerary.node_ids {
            Int _ (27)
            Int _ (28)
            Int _ (22)
            Int _ (29)
            Int _ (30)
            Int _ (21)
            Int _ (23)
        }
    }

    set_safest_itinerary -> {
        "b5ac3bb1-593c-4dff-b1bd-9dcca5c39ab2" =: this.safest_itinerary.uid
        "Planning safest OK: a path including PPOs was found with cost 501.248" =: this.safest_itinerary.description_input
    }
    set_safest_itinerary -> na_set_safest:(this) {
        addChildrenTo this.safest_itinerary.node_ids {
            Int _ (27)
            Int _ (28)
            Int _ (22)
            Int _ (29)
        }
    }

    set_tradeoff_itinerary -> {
        "4e5aa5e6-0ed4-430f-9094-d0e8295594e7" =: this.tradeoff_itinerary.uid
        "Planning tradeoff OK: a path including PPOs was found with cost 450.050" =: this.tradeoff_itinerary.description_input
    }
    set_tradeoff_itinerary -> na_set_tradeoff:(this) {
        addChildrenTo this.tradeoff_itinerary.node_ids {
            Int _ (29)
            Int _ (30)
            Int _ (21)
            Int _ (23)
        }
    }


    // **************************************************************************************************
    //
    //  LIMITS
    //
    // **************************************************************************************************

    addChildrenTo this.limits {
        PointModel _ (48.860842645824064, 1.8932376517446958, 0.0)
        PointModel _ (48.860767378809605, 1.891957685955128, 0.0)
        PointModel _ (48.8600332177389, 1.8913876263562024, 0.0)
        PointModel _ (48.859540513596414, 1.8878673072636698, 0.0)
        PointModel _ (48.8597485227418, 1.8861175411994577, 0.0)
        PointModel _ (48.8601892353074, 1.886107760769553, 0.0)
        PointModel _ (48.86053727266625, 1.8848865880934869, 0.0)
        PointModel _ (48.86635888720907, 1.883148198410929, 0.0)
        PointModel _ (48.86767699534763, 1.885491559200939, 0.0)
        PointModel _ (48.86694948853563, 1.8865440531753028, 0.0)
        PointModel _ (48.86640378293678, 1.8859289126837873, 0.0)
        PointModel _ (48.86571303786785, 1.8870623718795294, 0.0)
        PointModel _ (48.865088920648745, 1.8885761172587332, 0.0)
        PointModel _ (48.863756345748016, 1.8903236551257345, 0.0)
        PointModel _ (48.861250966497174, 1.8917288587814383, 0.0)
        PointModel _ (48.86131047269054, 1.8932409700211832, 0.0)
    }


    // **************************************************************************************************
    //
    //  EXCLUSION ZONES
    //
    // **************************************************************************************************

    addChildrenTo this.zones {
        ExclusionZoneModel za (5, "ZA")
        ExclusionZoneModel zb (3, "ZB")
        ExclusionZoneModel zc (2, "ZC")
        ExclusionZoneModel zd (5, "ZD") // FIXME: 6 ROZ_GROUND instead of 5 ROZ_ALL ?
    }

    addChildrenTo this.zones.[1].points {
        PointModel _ (48.86069712046944, 1.8865327868841357, 0.0)
        PointModel _ (48.860405947808886, 1.8843305027102233, 0.0)
        PointModel _ (48.8582831729342, 1.8852911394666538, 0.0)
        PointModel _ (48.85851523079417, 1.8869629318702104, 0.0)
    }
    addChildrenTo this.zones.[2].points {
        PointModel _ (48.86356725631027, 1.8875053752135988, 0.0)
        PointModel _ (48.86317427647312, 1.887800420838259, 0.0)
        PointModel _ (48.86278463517772, 1.887509088628924, 0.0)
        PointModel _ (48.862777134716374, 1.8867320676825863, 0.0)
        PointModel _ (48.863170111801466, 1.8864370161260662, 0.0)
        PointModel _ (48.86355062996941, 1.8867149095144675, 0.0)
    }
    addChildrenTo this.zones.[3].points {
        PointModel _ (48.86699109672875, 1.8843340963798363, 0.0)
        PointModel _ (48.866331640733726, 1.883121533155845, 0.0)
        PointModel _ (48.8653626531049, 1.8833885394899847, 0.0)
        PointModel _ (48.86577870856293, 1.8854791766182561, 0.0)
        PointModel _ (48.86677840361167, 1.8846660880707284, 0.0)
    }
    addChildrenTo this.zones.[4].points {
        PointModel _ (48.86770397770758, 1.8854909599023824, 0.0)
        PointModel _ (48.86700035469283, 1.8843611625703838, 0.0)
        PointModel _ (48.86653819975485, 1.8849441446593271, 0.0)
        PointModel _ (48.867295781830215, 1.886072742014048, 0.0)
    }


    // **************************************************************************************************
    //
    //  LIMAS
    //
    // **************************************************************************************************

    addChildrenTo this.limas {
        LimaModel ld (1, "LD", null)
        LimaModel l1 (2, "L1", null)
        LimaModel l2 (3, "L2", null)
        LimaModel l3 (4, "L3", null)
        LimaModel l4 (5, "L4", null)
    }

    addChildrenTo this.limas.[1].points {
        PointModel _ (48.8608, 1.89324, 0.0)
        PointModel _ (48.8613, 1.89324, 0.0)
    }
    addChildrenTo this.limas.[2].points {
        PointModel _ (48.8605, 1.88479, 0.0)
        PointModel _ (48.8609, 1.8883, 0.0)
        PointModel _ (48.8609, 1.8891, 0.0)
        PointModel _ (48.8614, 1.89173, 0.0)
    }
    addChildrenTo this.limas.[3].points {
        PointModel _ (48.8619, 1.88443, 0.0)
        PointModel _ (48.8624, 1.88604, 0.0)
        PointModel _ (48.8626, 1.88776, 0.0)
        PointModel _ (48.8628, 1.88824, 0.0)
        PointModel _ (48.8636, 1.8905, 0.0)
    }
    addChildrenTo this.limas.[4].points {
        PointModel _ (48.8639, 1.88383, 0.0)
        PointModel _ (48.8653, 1.8885, 0.0)
    }
    addChildrenTo this.limas.[5].points {
        PointModel _ (48.8663, 1.8831, 0.0)
        PointModel _ (48.867, 1.88442, 0.0)
        PointModel _ (48.8677, 1.88549, 0.0)
    }


    // **************************************************************************************************
    //
    //  NODES
    //
    // **************************************************************************************************

    addChildrenTo this.nodes {
        //NodeModel (int _id, int _phase, string _label, double _lat, double _lon, double _alt, int _is_mandatory)
        NodeModel node_0 (0, 4, "", 48.8664, 1.88506, 0, 0)
        NodeModel node_1 (1, 4, "", 48.8662, 1.88534, 0, 0)
        NodeModel node_2 (2, 4, "", 48.8667, 1.88602, 0, 0)
        NodeModel node_3 (3, 4, "", 48.8654, 1.88662, 0, 0)
        NodeModel node_4 (4, 4, "", 48.8645, 1.88433, 0, 0)
        NodeModel node_5 (5, 4, "", 48.8653, 1.88536, 0, 0)
        NodeModel node_6 (6, 4, "", 48.8653, 1.8851, 0, 0)
        NodeModel node_7 (7, 4, "", 48.8654, 1.88504, 0, 0)
        NodeModel node_8 (8, 3, "", 48.8631, 1.88716, 0, 0)
        NodeModel node_9 (9, 3, "", 48.8646, 1.8866, 0, 0)
        NodeModel node_10 (10, 2, "", 48.863, 1.88879, 0, 0)
        NodeModel node_11 (11, 2, "", 48.8628, 1.88937, 0, 0)
        NodeModel node_12 (12, 2, "", 48.8626, 1.88896, 0, 0)
        NodeModel node_13 (13, 2, "", 48.8626, 1.88836, 0, 0)
        NodeModel node_14 (14, 2, "", 48.8625, 1.88815, 0, 0)
        NodeModel node_15 (15, 2, "", 48.8628, 1.88843, 0, 0)
        NodeModel node_16 (16, 3, "", 48.863, 1.88838, 0, 0)
        NodeModel node_17 (17, 2, "", 48.8627, 1.8881, 0, 0)
        NodeModel node_18 (18, 2, "", 48.8608, 1.88694, 0, 0)
        NodeModel node_19 (19, 1, "", 48.8597, 1.88709, 0, 0)
        NodeModel node_20 (20, 1, "", 48.8603, 1.88699, 0, 0)
        NodeModel node_21 (21, 1, "", 48.8603, 1.88722, 0, 0)
        NodeModel node_22 (22, 1, "", 48.861, 1.89203, 0, 0)
        NodeModel node_23 (23, 1, "PPO10", 48.8607, 1.88702, 0, 1)
        NodeModel node_24 (24, 2, "PPO20", 48.8632, 1.88971, 0, 1)
        NodeModel node_25 (25, 3, "PPO30", 48.8647, 1.88677, 0, 1)
        NodeModel node_26 (26, 4, "PPO40", 48.8662, 1.88387, 0, 1)
        NodeModel node_27 (27, 0, "depart", 48.8611, 1.89331, 0, 1)
        NodeModel node_28 (28, 1, "T101", 48.861, 1.89266, 0, 0)
        NodeModel node_29 (29, 1, "T102", 48.8608, 1.8904, 0, 0)
        NodeModel node_30 (30, 1, "T103", 48.8603, 1.88755, 0, 0)
        NodeModel node_31 (31, 2, "T201", 48.8619, 1.88468, 0, 0)
        NodeModel node_32 (32, 2, "T202", 48.8622, 1.88563, 0, 0)
        NodeModel node_33 (33, 2, "T203", 48.8614, 1.8866, 0, 0)
        NodeModel node_34 (34, 2, "T204", 48.8609, 1.88735, 0, 0)
        NodeModel node_35 (35, 2, "T205", 48.8622, 1.88666, 0, 0)
        NodeModel node_36 (36, 2, "T206", 48.8617, 1.88715, 0, 0)
        NodeModel node_37 (37, 2, "T207", 48.8624, 1.88741, 0, 0)
        NodeModel node_38 (38, 2, "T208", 48.862, 1.88769, 0, 0)
        NodeModel node_39 (39, 2, "T209", 48.8625, 1.88788, 0, 0)
        NodeModel node_40 (40, 2, "T210", 48.8623, 1.88832, 0, 0)
        NodeModel node_41 (41, 2, "T211", 48.8622, 1.88925, 0, 0)
        NodeModel node_42 (42, 2, "T212", 48.8627, 1.88883, 0, 0)
        NodeModel node_43 (43, 2, "T213", 48.863, 1.8891, 0, 0)
        NodeModel node_44 (44, 2, "T214", 48.8627, 1.88946, 0, 0)
        NodeModel node_45 (45, 2, "T215", 48.8624, 1.88974, 0, 0)
        NodeModel node_46 (46, 2, "T216", 48.8628, 1.88996, 0, 0)
        NodeModel node_47 (47, 3, "T301", 48.8631, 1.88509, 0, 0)
        NodeModel node_48 (48, 3, "T302", 48.863, 1.88569, 0, 0)
        NodeModel node_49 (49, 3, "T303", 48.8637, 1.88599, 0, 0)
        NodeModel node_50 (50, 3, "T304", 48.8645, 1.88698, 0, 0)
        NodeModel node_51 (51, 3, "T305", 48.8643, 1.88743, 0, 0)
        NodeModel node_52 (52, 3, "T306", 48.8639, 1.88761, 0, 0)
        NodeModel node_53 (53, 3, "T307", 48.8636, 1.88802, 0, 0)
        NodeModel node_54 (54, 3, "T308", 48.8629, 1.88813, 0, 0)
        NodeModel node_55 (55, 3, "T309", 48.8639, 1.88811, 0, 0)
        NodeModel node_56 (56, 3, "T310", 48.8648, 1.88741, 0, 0)
        NodeModel node_57 (57, 3, "T311", 48.8644, 1.88859, 0, 0)
        NodeModel node_58 (58, 3, "T312", 48.8635, 1.88897, 0, 0)
        NodeModel node_59 (59, 3, "T313", 48.8638, 1.88922, 0, 0)
        NodeModel node_60 (60, 3, "T314", 48.864, 1.88964, 0, 0)
        NodeModel node_61 (61, 4, "T401", 48.8651, 1.88414, 0, 0)
        NodeModel node_62 (62, 4, "T402", 48.8644, 1.88512, 0, 0)
        NodeModel node_63 (63, 4, "T403", 48.8653, 1.88463, 0, 0)
        NodeModel node_64 (64, 4, "T404", 48.8658, 1.88435, 0, 0)
        NodeModel node_65 (65, 4, "T405", 48.8663, 1.88454, 0, 0)
        NodeModel node_66 (66, 4, "T406", 48.8657, 1.88487, 0, 0)
        NodeModel node_67 (67, 4, "T407", 48.865, 1.88521, 0, 0)
        NodeModel node_68 (68, 4, "T408", 48.8651, 1.88559, 0, 0)
        NodeModel node_69 (69, 4, "T409", 48.8653, 1.88556, 0, 0)
        NodeModel node_70 (70, 4, "T410", 48.865, 1.88608, 0, 0)
        NodeModel node_71 (71, 4, "T411", 48.8655, 1.88677, 0, 0)
        NodeModel node_72 (72, 4, "T412", 48.8655, 1.88622, 0, 0)
        NodeModel node_73 (73, 4, "T413", 48.8661, 1.88517, 0, 0)
        NodeModel node_74 (74, 4, "T414", 48.8665, 1.88496, 0, 0)
        NodeModel node_75 (75, 4, "T415", 48.8668, 1.88585, 0, 0)
        NodeModel node_76 (76, 2, "", 48.8624, 1.88598, 0, 0)
        NodeModel node_77 (77, 2, "", 48.8621, 1.88525, 0, 0)
        NodeModel node_78 (78, 2, "", 48.861, 1.88718, 0, 0)
        NodeModel node_79 (79, 2, "", 48.8614, 1.88727, 0, 0)
        NodeModel node_80 (80, 2, "", 48.8616, 1.88737, 0, 0)
        NodeModel node_81 (81, 2, "", 48.8613, 1.88707, 0, 0)
        NodeModel node_82 (82, 2, "", 48.8625, 1.88765, 0, 0)
        NodeModel node_83 (83, 2, "", 48.8624, 1.88804, 0, 0)
        NodeModel node_84 (84, 2, "", 48.8624, 1.88718, 0, 0)
        NodeModel node_85 (85, 2, "", 48.8621, 1.88731, 0, 0)
        NodeModel node_86 (86, 3, "", 48.8627, 1.88742, 0, 0)
        NodeModel node_87 (87, 3, "", 48.8627, 1.88789, 0, 0)
        NodeModel node_88 (88, 3, "", 48.8632, 1.88823, 0, 0)
        NodeModel node_89 (89, 3, "", 48.8634, 1.88913, 0, 0)
        NodeModel node_90 (90, 3, "", 48.8636, 1.88958, 0, 0)
        NodeModel node_91 (91, 3, "", 48.8636, 1.89008, 0, 0)
        NodeModel node_92 (92, 3, "", 48.8638, 1.88941, 0, 0)
        NodeModel node_93 (93, 3, "", 48.8641, 1.88911, 0, 0)
        NodeModel node_94 (94, 3, "", 48.8639, 1.88853, 0, 0)
        NodeModel node_95 (95, 3, "", 48.8641, 1.88778, 0, 0)
        NodeModel node_96 (96, 3, "", 48.8643, 1.88677, 0, 0)
        NodeModel node_97 (97, 3, "", 48.865, 1.88763, 0, 0)
        NodeModel node_98 (98, 3, "", 48.8642, 1.88519, 0, 0)
        NodeModel node_99 (99, 3, "", 48.8643, 1.88555, 0, 0)
        NodeModel node_100 (100, 3, "", 48.8641, 1.8848, 0, 0)
        NodeModel node_101 (101, 3, "", 48.865, 1.88793, 0, 0)
        NodeModel node_102 (102, 3, "", 48.8643, 1.88843, 0, 0)
        NodeModel node_103 (103, 4, "", 48.8659, 1.88456, 0, 0)
        NodeModel node_104 (104, 4, "", 48.8653, 1.88409, 0, 0)
        NodeModel node_105 (105, 4, "", 48.8662, 1.88452, 0, 0)
        NodeModel node_106 (106, 4, "", 48.8661, 1.88491, 0, 0)
    }


    // **************************************************************************************************
    //
    //  EDGES
    //
    // **************************************************************************************************

    addChildrenTo this.edges {
        //EdgeModel (Process _node1, Process _node2, double _length)
        EdgeModel edge_106_65 (this.nodes[107], this.nodes[66], 36.1518)
        EdgeModel edge_106_0 (this.nodes[107], this.nodes[1], 29.5981)
        EdgeModel edge_106_73 (this.nodes[107], this.nodes[74], 19.4721)
        EdgeModel edge_105_26 (this.nodes[106], this.nodes[27], 47.4702)
        EdgeModel edge_105_64 (this.nodes[106], this.nodes[65], 43.218)
        EdgeModel edge_105_74 (this.nodes[106], this.nodes[75], 47.8072)
        EdgeModel edge_105_106 (this.nodes[106], this.nodes[107], 30.1183)
        EdgeModel edge_104_63 (this.nodes[105], this.nodes[64], 40.2597)
        EdgeModel edge_103_106 (this.nodes[104], this.nodes[107], 31.9294)
        EdgeModel edge_103_66 (this.nodes[104], this.nodes[67], 37.3526)
        EdgeModel edge_102_56 (this.nodes[103], this.nodes[57], 95.0007)
        EdgeModel edge_100_4 (this.nodes[101], this.nodes[5], 54.1307)
        EdgeModel edge_99_49 (this.nodes[100], this.nodes[50], 82.4335)
        EdgeModel edge_99_67 (this.nodes[100], this.nodes[68], 81.3112)
        EdgeModel edge_98_48 (this.nodes[99], this.nodes[49], 136.543)
        EdgeModel edge_98_62 (this.nodes[99], this.nodes[63], 15.8725)
        EdgeModel edge_97_57 (this.nodes[98], this.nodes[58], 92.5662)
        EdgeModel edge_97_3 (this.nodes[98], this.nodes[4], 88.1322)
        EdgeModel edge_96_52 (this.nodes[97], this.nodes[53], 75.7454)
        EdgeModel edge_95_55 (this.nodes[96], this.nodes[56], 28.8409)
        EdgeModel edge_95_51 (this.nodes[96], this.nodes[52], 36.0707)
        EdgeModel edge_94_59 (this.nodes[95], this.nodes[60], 51.3599)
        EdgeModel edge_93_57 (this.nodes[94], this.nodes[58], 48.6935)
        EdgeModel edge_92_93 (this.nodes[93], this.nodes[94], 42.9731)
        EdgeModel edge_92_90 (this.nodes[93], this.nodes[91], 30.2211)
        EdgeModel edge_91_60 (this.nodes[92], this.nodes[61], 49.1265)
        EdgeModel edge_91_92 (this.nodes[92], this.nodes[93], 52.3149)
        EdgeModel edge_90_59 (this.nodes[91], this.nodes[60], 40.4231)
        EdgeModel edge_90_91 (this.nodes[91], this.nodes[92], 37.2258)
        EdgeModel edge_89_58 (this.nodes[90], this.nodes[59], 18.4513)
        EdgeModel edge_89_24 (this.nodes[90], this.nodes[25], 45.2469)
        EdgeModel edge_88_54 (this.nodes[89], this.nodes[55], 36.3603)
        EdgeModel edge_87_54 (this.nodes[88], this.nodes[55], 22.2335)
        EdgeModel edge_86_84 (this.nodes[87], this.nodes[85], 35.8564)
        EdgeModel edge_86_82 (this.nodes[87], this.nodes[83], 28.6018)
        EdgeModel edge_85_37 (this.nodes[86], this.nodes[38], 40.2236)
        EdgeModel edge_84_35 (this.nodes[85], this.nodes[36], 47.0857)
        EdgeModel edge_83_40 (this.nodes[84], this.nodes[41], 23.921)
        EdgeModel edge_83_39 (this.nodes[84], this.nodes[40], 18.0651)
        EdgeModel edge_82_38 (this.nodes[83], this.nodes[39], 56.5266)
        EdgeModel edge_82_83 (this.nodes[83], this.nodes[84], 30.1205)
        EdgeModel edge_81_36 (this.nodes[82], this.nodes[37], 44.2575)
        EdgeModel edge_80_79 (this.nodes[81], this.nodes[80], 21.1435)
        EdgeModel edge_79_78 (this.nodes[80], this.nodes[79], 44.4053)
        EdgeModel edge_78_81 (this.nodes[79], this.nodes[82], 33.1313)
        EdgeModel edge_77_32 (this.nodes[78], this.nodes[33], 30.0201)
        EdgeModel edge_77_47 (this.nodes[78], this.nodes[48], 102.932)
        EdgeModel edge_76_48 (this.nodes[77], this.nodes[49], 80.2175)
        EdgeModel edge_76_33 (this.nodes[77], this.nodes[34], 110.503)
        EdgeModel edge_75_2 (this.nodes[76], this.nodes[3], 16.8779)
        EdgeModel edge_74_106 (this.nodes[75], this.nodes[107], 45.7616)
        EdgeModel edge_74_0 (this.nodes[75], this.nodes[1], 19.5216)
        EdgeModel edge_72_3 (this.nodes[73], this.nodes[4], 31.601)
        EdgeModel edge_72_73 (this.nodes[73], this.nodes[74], 104.723)
        EdgeModel edge_71_101 (this.nodes[72], this.nodes[102], 100.104)
        EdgeModel edge_70_9 (this.nodes[71], this.nodes[10], 56.5335)
        EdgeModel edge_70_25 (this.nodes[71], this.nodes[26], 61.0522)
        EdgeModel edge_70_69 (this.nodes[71], this.nodes[70], 51.6788)
        EdgeModel edge_69_66 (this.nodes[70], this.nodes[67], 63.4315)
        EdgeModel edge_69_5 (this.nodes[70], this.nodes[6], 14.9078)
        EdgeModel edge_68_70 (this.nodes[69], this.nodes[71], 36.8899)
        EdgeModel edge_67_68 (this.nodes[68], this.nodes[69], 28.6447)
        EdgeModel edge_67_6 (this.nodes[68], this.nodes[7], 30.4569)
        EdgeModel edge_65_105 (this.nodes[66], this.nodes[106], 14.0152)
        EdgeModel edge_64_26 (this.nodes[65], this.nodes[27], 55.6234)
        EdgeModel edge_64_103 (this.nodes[65], this.nodes[104], 19.614)
        EdgeModel edge_63_64 (this.nodes[64], this.nodes[65], 67.0735)
        EdgeModel edge_62_63 (this.nodes[63], this.nodes[64], 106.603)
        EdgeModel edge_61_104 (this.nodes[62], this.nodes[105], 20.7912)
        EdgeModel edge_60_101 (this.nodes[61], this.nodes[102], 171.189)
        EdgeModel edge_59_92 (this.nodes[60], this.nodes[93], 14.5567)
        EdgeModel edge_59_102 (this.nodes[60], this.nodes[103], 78.1447)
        EdgeModel edge_58_94 (this.nodes[59], this.nodes[95], 55.8336)
        EdgeModel edge_58_59 (this.nodes[59], this.nodes[60], 39.8373)
        EdgeModel edge_56_97 (this.nodes[57], this.nodes[98], 21.0717)
        EdgeModel edge_56_72 (this.nodes[57], this.nodes[73], 113.491)
        EdgeModel edge_55_94 (this.nodes[56], this.nodes[95], 30.9432)
        EdgeModel edge_54_17 (this.nodes[55], this.nodes[18], 21.7711)
        EdgeModel edge_54_16 (this.nodes[55], this.nodes[17], 21.4194)
        EdgeModel edge_53_52 (this.nodes[54], this.nodes[53], 43.8774)
        EdgeModel edge_53_88 (this.nodes[54], this.nodes[89], 48.7528)
        EdgeModel edge_52_95 (this.nodes[53], this.nodes[96], 23.9674)
        EdgeModel edge_51_50 (this.nodes[52], this.nodes[51], 37.6019)
        EdgeModel edge_50_96 (this.nodes[51], this.nodes[97], 24.1791)
        EdgeModel edge_49_96 (this.nodes[50], this.nodes[97], 90.8357)
        EdgeModel edge_48_47 (this.nodes[49], this.nodes[48], 44.0594)
        EdgeModel edge_47_100 (this.nodes[48], this.nodes[101], 118.66)
        EdgeModel edge_46_24 (this.nodes[47], this.nodes[25], 56.9391)
        EdgeModel edge_45_44 (this.nodes[46], this.nodes[45], 38.5999)
        EdgeModel edge_44_46 (this.nodes[45], this.nodes[47], 38.091)
        EdgeModel edge_44_12 (this.nodes[45], this.nodes[13], 36.9842)
        EdgeModel edge_43_55 (this.nodes[44], this.nodes[56], 121.867)
        EdgeModel edge_43_89 (this.nodes[44], this.nodes[90], 39.2359)
        EdgeModel edge_43_11 (this.nodes[44], this.nodes[12], 35.9173)
        EdgeModel edge_42_15 (this.nodes[43], this.nodes[16], 31.8246)
        EdgeModel edge_42_13 (this.nodes[43], this.nodes[14], 36.3684)
        EdgeModel edge_41_45 (this.nodes[42], this.nodes[46], 43.2804)
        EdgeModel edge_40_41 (this.nodes[41], this.nodes[42], 69.6569)
        EdgeModel edge_40_44 (this.nodes[41], this.nodes[45], 93.9448)
        EdgeModel edge_40_14 (this.nodes[41], this.nodes[15], 29.5244)
        EdgeModel edge_39_82 (this.nodes[40], this.nodes[83], 17.3108)
        EdgeModel edge_39_17 (this.nodes[40], this.nodes[18], 23.1973)
        EdgeModel edge_39_87 (this.nodes[40], this.nodes[88], 25.2721)
        EdgeModel edge_38_40 (this.nodes[39], this.nodes[41], 58.0271)
        EdgeModel edge_37_82 (this.nodes[38], this.nodes[83], 19.1894)
        EdgeModel edge_37_84 (this.nodes[38], this.nodes[85], 16.7778)
        EdgeModel edge_36_85 (this.nodes[37], this.nodes[86], 39.4976)
        EdgeModel edge_36_38 (this.nodes[37], this.nodes[39], 48.2751)
        EdgeModel edge_36_80 (this.nodes[37], this.nodes[81], 20.5998)
        EdgeModel edge_34_78 (this.nodes[35], this.nodes[79], 20.6994)
        EdgeModel edge_32_76 (this.nodes[33], this.nodes[77], 28.464)
        EdgeModel edge_31_77 (this.nodes[32], this.nodes[78], 49.4087)
        EdgeModel edge_30_21 (this.nodes[31], this.nodes[22], 24.3918)
        EdgeModel edge_29_30 (this.nodes[30], this.nodes[31], 214.628)
        EdgeModel edge_28_22 (this.nodes[29], this.nodes[23], 46.2805)
        EdgeModel edge_27_28 (this.nodes[28], this.nodes[29], 48.5232)
        EdgeModel edge_26_104 (this.nodes[27], this.nodes[105], 101.586)
        EdgeModel edge_26_103 (this.nodes[27], this.nodes[104], 59.8659)
        EdgeModel edge_25_51 (this.nodes[26], this.nodes[52], 67.2002)
        EdgeModel edge_24_90 (this.nodes[25], this.nodes[91], 36.4749)
        EdgeModel edge_24_43 (this.nodes[25], this.nodes[44], 50.1059)
        EdgeModel edge_23_18 (this.nodes[24], this.nodes[19], 17.55)
        EdgeModel edge_22_29 (this.nodes[23], this.nodes[30], 122.646)
        EdgeModel edge_21_20 (this.nodes[22], this.nodes[21], 17.7584)
        EdgeModel edge_21_23 (this.nodes[22], this.nodes[24], 44.7778)
        EdgeModel edge_20_19 (this.nodes[21], this.nodes[20], 63.2881)
        EdgeModel edge_18_34 (this.nodes[19], this.nodes[35], 30.2488)
        EdgeModel edge_18_33 (this.nodes[19], this.nodes[34], 72.6205)
        EdgeModel edge_17_14 (this.nodes[18], this.nodes[15], 16.011)
        EdgeModel edge_16_15 (this.nodes[17], this.nodes[16], 16.0109)
        EdgeModel edge_16_10 (this.nodes[17], this.nodes[11], 29.9136)
        EdgeModel edge_16_88 (this.nodes[17], this.nodes[89], 26.7116)
        EdgeModel edge_15_10 (this.nodes[16], this.nodes[11], 31.0289)
        EdgeModel edge_15_17 (this.nodes[16], this.nodes[18], 29.2559)
        EdgeModel edge_14_13 (this.nodes[15], this.nodes[14], 19.1998)
        EdgeModel edge_14_39 (this.nodes[15], this.nodes[40], 19.3396)
        EdgeModel edge_14_12 (this.nodes[15], this.nodes[13], 60.9751)
        EdgeModel edge_14_83 (this.nodes[15], this.nodes[84], 16.8636)
        EdgeModel edge_13_15 (this.nodes[14], this.nodes[16], 22.4794)
        EdgeModel edge_12_42 (this.nodes[13], this.nodes[43], 13.9923)
        EdgeModel edge_11_46 (this.nodes[12], this.nodes[47], 43.6245)
        EdgeModel edge_11_44 (this.nodes[12], this.nodes[45], 12.6501)
        EdgeModel edge_10_43 (this.nodes[11], this.nodes[44], 24.2607)
        EdgeModel edge_9_50 (this.nodes[10], this.nodes[51], 35.0629)
        EdgeModel edge_8_86 (this.nodes[9], this.nodes[87], 52.2032)
        EdgeModel edge_7_66 (this.nodes[8], this.nodes[67], 30.7221)
        EdgeModel edge_7_5 (this.nodes[8], this.nodes[6], 26.7463)
        EdgeModel edge_6_7 (this.nodes[7], this.nodes[8], 14.8963)
        EdgeModel edge_5_6 (this.nodes[6], this.nodes[7], 18.957)
        EdgeModel edge_5_68 (this.nodes[6], this.nodes[69], 29.8985)
        EdgeModel edge_4_61 (this.nodes[5], this.nodes[62], 74.9735)
        EdgeModel edge_3_71 (this.nodes[4], this.nodes[72], 16.0983)
        EdgeModel edge_2_1 (this.nodes[3], this.nodes[2], 73.9637)
        EdgeModel edge_1_73 (this.nodes[2], this.nodes[74], 17.9437)
        EdgeModel edge_1_0 (this.nodes[2], this.nodes[1], 24.431)
        EdgeModel edge_0_75 (this.nodes[1], this.nodes[76], 78.195)
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