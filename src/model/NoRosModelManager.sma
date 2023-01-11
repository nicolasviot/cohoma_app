/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2022)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *	Contributors:
 *      Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use gui
use base

import ModelManager
import PointModel
import ExclusionZoneModel
import LimaModel
import VehiculeModel
import SafetyPilotModel
import NodeModel
import EdgeModel
import ItineraryModel
import TrapModel
import task.TaskTrapModel

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
        NodeModel _106 (106, 4, "", 48.8661, 1.88491, 0, 0)
        NodeModel _105 (105, 4, "", 48.8662, 1.88452, 0, 0)
        NodeModel _104 (104, 4, "", 48.8653, 1.88409, 0, 0)
        NodeModel _103 (103, 4, "", 48.8659, 1.88456, 0, 0)
        NodeModel _102 (102, 3, "", 48.8643, 1.88843, 0, 0)
        NodeModel _101 (101, 3, "", 48.865, 1.88793, 0, 0)
        NodeModel _100 (100, 3, "", 48.8641, 1.8848, 0, 0)
        NodeModel _99 (99, 3, "", 48.8643, 1.88555, 0, 0)
        NodeModel _98 (98, 3, "", 48.8642, 1.88519, 0, 0)
        NodeModel _97 (97, 3, "", 48.865, 1.88763, 0, 0)
        NodeModel _96 (96, 3, "", 48.8643, 1.88677, 0, 0)
        NodeModel _95 (95, 3, "", 48.8641, 1.88778, 0, 0)
        NodeModel _94 (94, 3, "", 48.8639, 1.88853, 0, 0)
        NodeModel _93 (93, 3, "", 48.8641, 1.88911, 0, 0)
        NodeModel _92 (92, 3, "", 48.8638, 1.88941, 0, 0)
        NodeModel _91 (91, 3, "", 48.8636, 1.89008, 0, 0)
        NodeModel _90 (90, 3, "", 48.8636, 1.88958, 0, 0)
        NodeModel _89 (89, 3, "", 48.8634, 1.88913, 0, 0)
        NodeModel _88 (88, 3, "", 48.8632, 1.88823, 0, 0)
        NodeModel _87 (87, 3, "", 48.8627, 1.88789, 0, 0)
        NodeModel _86 (86, 3, "", 48.8627, 1.88742, 0, 0)
        NodeModel _85 (85, 2, "", 48.8621, 1.88731, 0, 0)
        NodeModel _84 (84, 2, "", 48.8624, 1.88718, 0, 0)
        NodeModel _83 (83, 2, "", 48.8624, 1.88804, 0, 0)
        NodeModel _82 (82, 2, "", 48.8625, 1.88765, 0, 0)
        NodeModel _81 (81, 2, "", 48.8613, 1.88707, 0, 0)
        NodeModel _80 (80, 2, "", 48.8616, 1.88737, 0, 0)
        NodeModel _79 (79, 2, "", 48.8614, 1.88727, 0, 0)
        NodeModel _78 (78, 2, "", 48.861, 1.88718, 0, 0)
        NodeModel _77 (77, 2, "", 48.8621, 1.88525, 0, 0)
        NodeModel _76 (76, 2, "", 48.8624, 1.88598, 0, 0)
        NodeModel _75 (75, 4, "T415", 48.8668, 1.88585, 0, 0)
        NodeModel _74 (74, 4, "T414", 48.8665, 1.88496, 0, 0)
        NodeModel _73 (73, 4, "T413", 48.8661, 1.88517, 0, 0)
        NodeModel _72 (72, 4, "T412", 48.8655, 1.88622, 0, 0)
        NodeModel _71 (71, 4, "T411", 48.8655, 1.88677, 0, 0)
        NodeModel _70 (70, 4, "T410", 48.865, 1.88608, 0, 0)
        NodeModel _69 (69, 4, "T409", 48.8653, 1.88556, 0, 0)
        NodeModel _68 (68, 4, "T408", 48.8651, 1.88559, 0, 0)
        NodeModel _67 (67, 4, "T407", 48.865, 1.88521, 0, 0)
        NodeModel _66 (66, 4, "T406", 48.8657, 1.88487, 0, 0)
        NodeModel _65 (65, 4, "T405", 48.8663, 1.88454, 0, 0)
        NodeModel _64 (64, 4, "T404", 48.8658, 1.88435, 0, 0)
        NodeModel _63 (63, 4, "T403", 48.8653, 1.88463, 0, 0)
        NodeModel _62 (62, 4, "T402", 48.8644, 1.88512, 0, 0)
        NodeModel _61 (61, 4, "T401", 48.8651, 1.88414, 0, 0)
        NodeModel _60 (60, 3, "T314", 48.864, 1.88964, 0, 0)
        NodeModel _59 (59, 3, "T313", 48.8638, 1.88922, 0, 0)
        NodeModel _58 (58, 3, "T312", 48.8635, 1.88897, 0, 0)
        NodeModel _57 (57, 3, "T311", 48.8644, 1.88859, 0, 0)
        NodeModel _56 (56, 3, "T310", 48.8648, 1.88741, 0, 0)
        NodeModel _55 (55, 3, "T309", 48.8639, 1.88811, 0, 0)
        NodeModel _54 (54, 3, "T308", 48.8629, 1.88813, 0, 0)
        NodeModel _53 (53, 3, "T307", 48.8636, 1.88802, 0, 0)
        NodeModel _52 (52, 3, "T306", 48.8639, 1.88761, 0, 0)
        NodeModel _51 (51, 3, "T305", 48.8643, 1.88743, 0, 0)
        NodeModel _50 (50, 3, "T304", 48.8645, 1.88698, 0, 0)
        NodeModel _49 (49, 3, "T303", 48.8637, 1.88599, 0, 0)
        NodeModel _48 (48, 3, "T302", 48.863, 1.88569, 0, 0)
        NodeModel _47 (47, 3, "T301", 48.8631, 1.88509, 0, 0)
        NodeModel _46 (46, 2, "T216", 48.8628, 1.88996, 0, 0)
        NodeModel _45 (45, 2, "T215", 48.8624, 1.88974, 0, 0)
        NodeModel _44 (44, 2, "T214", 48.8627, 1.88946, 0, 0)
        NodeModel _43 (43, 2, "T213", 48.863, 1.8891, 0, 0)
        NodeModel _42 (42, 2, "T212", 48.8627, 1.88883, 0, 0)
        NodeModel _41 (41, 2, "T211", 48.8622, 1.88925, 0, 0)
        NodeModel _40 (40, 2, "T210", 48.8623, 1.88832, 0, 0)
        NodeModel _39 (39, 2, "T209", 48.8625, 1.88788, 0, 0)
        NodeModel _38 (38, 2, "T208", 48.862, 1.88769, 0, 0)
        NodeModel _37 (37, 2, "T207", 48.8624, 1.88741, 0, 0)
        NodeModel _36 (36, 2, "T206", 48.8617, 1.88715, 0, 0)
        NodeModel _35 (35, 2, "T205", 48.8622, 1.88666, 0, 0)
        NodeModel _34 (34, 2, "T204", 48.8609, 1.88735, 0, 0)
        NodeModel _33 (33, 2, "T203", 48.8614, 1.8866, 0, 0)
        NodeModel _32 (32, 2, "T202", 48.8622, 1.88563, 0, 0)
        NodeModel _31 (31, 2, "T201", 48.8619, 1.88468, 0, 0)
        NodeModel _30 (30, 1, "T103", 48.8603, 1.88755, 0, 0)
        NodeModel _29 (29, 1, "T102", 48.8608, 1.8904, 0, 0)
        NodeModel _28 (28, 1, "T101", 48.861, 1.89266, 0, 0)
        NodeModel _27 (27, 0, "depart", 48.8611, 1.89331, 0, 1)
        NodeModel _26 (26, 4, "PPO40", 48.8662, 1.88387, 0, 1)
        NodeModel _25 (25, 3, "PPO30", 48.8647, 1.88677, 0, 1)
        NodeModel _24 (24, 2, "PPO20", 48.8632, 1.88971, 0, 1)
        NodeModel _23 (23, 1, "PPO10", 48.8607, 1.88702, 0, 1)
        NodeModel _22 (22, 1, "", 48.861, 1.89203, 0, 0)
        NodeModel _21 (21, 1, "", 48.8603, 1.88722, 0, 0)
        NodeModel _20 (20, 1, "", 48.8603, 1.88699, 0, 0)
        NodeModel _19 (19, 1, "", 48.8597, 1.88709, 0, 0)
        NodeModel _18 (18, 2, "", 48.8608, 1.88694, 0, 0)
        NodeModel _17 (17, 2, "", 48.8627, 1.8881, 0, 0)
        NodeModel _16 (16, 3, "", 48.863, 1.88838, 0, 0)
        NodeModel _15 (15, 2, "", 48.8628, 1.88843, 0, 0)
        NodeModel _14 (14, 2, "", 48.8625, 1.88815, 0, 0)
        NodeModel _13 (13, 2, "", 48.8626, 1.88836, 0, 0)
        NodeModel _12 (12, 2, "", 48.8626, 1.88896, 0, 0)
        NodeModel _11 (11, 2, "", 48.8628, 1.88937, 0, 0)
        NodeModel _10 (10, 2, "", 48.863, 1.88879, 0, 0)
        NodeModel _9 (9, 3, "", 48.8646, 1.8866, 0, 0)
        NodeModel _8 (8, 3, "", 48.8631, 1.88716, 0, 0)
        NodeModel _7 (7, 4, "", 48.8654, 1.88504, 0, 0)
        NodeModel _6 (6, 4, "", 48.8653, 1.8851, 0, 0)
        NodeModel _5 (5, 4, "", 48.8653, 1.88536, 0, 0)
        NodeModel _4 (4, 4, "", 48.8645, 1.88433, 0, 0)
        NodeModel _3 (3, 4, "", 48.8654, 1.88662, 0, 0)
        NodeModel _2 (2, 4, "", 48.8667, 1.88602, 0, 0)
        NodeModel _1 (1, 4, "", 48.8662, 1.88534, 0, 0)
        NodeModel _0 (0, 4, "", 48.8664, 1.88506, 0, 0)
    }

    addChildrenTo this.node_ids {
        String _ ("_106")
        String _ ("_105")
        String _ ("_104")
        String _ ("_103")
        String _ ("_102")
        String _ ("_101")
        String _ ("_100")
        String _ ("_99")
        String _ ("_98")
        String _ ("_97")
        String _ ("_96")
        String _ ("_95")
        String _ ("_94")
        String _ ("_93")
        String _ ("_92")
        String _ ("_91")
        String _ ("_90")
        String _ ("_89")
        String _ ("_88")
        String _ ("_87")
        String _ ("_86")
        String _ ("_85")
        String _ ("_84")
        String _ ("_83")
        String _ ("_82")
        String _ ("_81")
        String _ ("_80")
        String _ ("_79")
        String _ ("_78")
        String _ ("_77")
        String _ ("_76")
        String _ ("_75")
        String _ ("_74")
        String _ ("_73")
        String _ ("_72")
        String _ ("_71")
        String _ ("_70")
        String _ ("_69")
        String _ ("_68")
        String _ ("_67")
        String _ ("_66")
        String _ ("_65")
        String _ ("_64")
        String _ ("_63")
        String _ ("_62")
        String _ ("_61")
        String _ ("_60")
        String _ ("_59")
        String _ ("_58")
        String _ ("_57")
        String _ ("_56")
        String _ ("_55")
        String _ ("_54")
        String _ ("_53")
        String _ ("_52")
        String _ ("_51")
        String _ ("_50")
        String _ ("_49")
        String _ ("_48")
        String _ ("_47")
        String _ ("_46")
        String _ ("_45")
        String _ ("_44")
        String _ ("_43")
        String _ ("_42")
        String _ ("_41")
        String _ ("_40")
        String _ ("_39")
        String _ ("_38")
        String _ ("_37")
        String _ ("_36")
        String _ ("_35")
        String _ ("_34")
        String _ ("_33")
        String _ ("_32")
        String _ ("_31")
        String _ ("_30")
        String _ ("_29")
        String _ ("_28")
        String _ ("_27")
        String _ ("_26")
        String _ ("_25")
        String _ ("_24")
        String _ ("_23")
        String _ ("_22")
        String _ ("_21")
        String _ ("_20")
        String _ ("_19")
        String _ ("_18")
        String _ ("_17")
        String _ ("_16")
        String _ ("_15")
        String _ ("_14")
        String _ ("_13")
        String _ ("_12")
        String _ ("_11")
        String _ ("_10")
        String _ ("_9")
        String _ ("_8")
        String _ ("_7")
        String _ ("_6")
        String _ ("_5")
        String _ ("_4")
        String _ ("_3")
        String _ ("_2")
        String _ ("_1")
        String _ ("_0")
    }


    // **************************************************************************************************
    //
    //  EDGES
    //
    // **************************************************************************************************

    addChildrenTo this.edges {
        EdgeModel _106_65 (find(this.nodes, "_106"), find(this.nodes, "_65"), 36.1518)
        EdgeModel _106_0 (find(this.nodes, "_106"), find(this.nodes, "_0"), 29.5981)
        EdgeModel _106_73 (find(this.nodes, "_106"), find(this.nodes, "_73"), 19.4721)
        EdgeModel _105_26 (find(this.nodes, "_105"), find(this.nodes, "_26"), 47.4702)
        EdgeModel _105_64 (find(this.nodes, "_105"), find(this.nodes, "_64"), 43.218)
        EdgeModel _105_74 (find(this.nodes, "_105"), find(this.nodes, "_74"), 47.8072)
        EdgeModel _105_106 (find(this.nodes, "_105"), find(this.nodes, "_106"), 30.1183)
        EdgeModel _104_63 (find(this.nodes, "_104"), find(this.nodes, "_63"), 40.2597)
        EdgeModel _103_106 (find(this.nodes, "_103"), find(this.nodes, "_106"), 31.9294)
        EdgeModel _103_66 (find(this.nodes, "_103"), find(this.nodes, "_66"), 37.3526)
        EdgeModel _102_56 (find(this.nodes, "_102"), find(this.nodes, "_56"), 95.0007)
        EdgeModel _100_4 (find(this.nodes, "_100"), find(this.nodes, "_4"), 54.1307)
        EdgeModel _99_49 (find(this.nodes, "_99"), find(this.nodes, "_49"), 82.4335)
        EdgeModel _99_67 (find(this.nodes, "_99"), find(this.nodes, "_67"), 81.3112)
        EdgeModel _98_48 (find(this.nodes, "_98"), find(this.nodes, "_48"), 136.543)
        EdgeModel _98_62 (find(this.nodes, "_98"), find(this.nodes, "_62"), 15.8725)
        EdgeModel _97_57 (find(this.nodes, "_97"), find(this.nodes, "_57"), 92.5662)
        EdgeModel _97_3 (find(this.nodes, "_97"), find(this.nodes, "_3"), 88.1322)
        EdgeModel _96_52 (find(this.nodes, "_96"), find(this.nodes, "_52"), 75.7454)
        EdgeModel _95_55 (find(this.nodes, "_95"), find(this.nodes, "_55"), 28.8409)
        EdgeModel _95_51 (find(this.nodes, "_95"), find(this.nodes, "_51"), 36.0707)
        EdgeModel _94_59 (find(this.nodes, "_94"), find(this.nodes, "_59"), 51.3599)
        EdgeModel _93_57 (find(this.nodes, "_93"), find(this.nodes, "_57"), 48.6935)
        EdgeModel _92_93 (find(this.nodes, "_92"), find(this.nodes, "_93"), 42.9731)
        EdgeModel _92_90 (find(this.nodes, "_92"), find(this.nodes, "_90"), 30.2211)
        EdgeModel _91_60 (find(this.nodes, "_91"), find(this.nodes, "_60"), 49.1265)
        EdgeModel _91_92 (find(this.nodes, "_91"), find(this.nodes, "_92"), 52.3149)
        EdgeModel _90_59 (find(this.nodes, "_90"), find(this.nodes, "_59"), 40.4231)
        EdgeModel _90_91 (find(this.nodes, "_90"), find(this.nodes, "_91"), 37.2258)
        EdgeModel _89_58 (find(this.nodes, "_89"), find(this.nodes, "_58"), 18.4513)
        EdgeModel _89_24 (find(this.nodes, "_89"), find(this.nodes, "_24"), 45.2469)
        EdgeModel _88_54 (find(this.nodes, "_88"), find(this.nodes, "_54"), 36.3603)
        EdgeModel _87_54 (find(this.nodes, "_87"), find(this.nodes, "_54"), 22.2335)
        EdgeModel _86_84 (find(this.nodes, "_86"), find(this.nodes, "_84"), 35.8564)
        EdgeModel _86_82 (find(this.nodes, "_86"), find(this.nodes, "_82"), 28.6018)
        EdgeModel _85_37 (find(this.nodes, "_85"), find(this.nodes, "_37"), 40.2236)
        EdgeModel _84_35 (find(this.nodes, "_84"), find(this.nodes, "_35"), 47.0857)
        EdgeModel _83_40 (find(this.nodes, "_83"), find(this.nodes, "_40"), 23.921)
        EdgeModel _83_39 (find(this.nodes, "_83"), find(this.nodes, "_39"), 18.0651)
        EdgeModel _82_38 (find(this.nodes, "_82"), find(this.nodes, "_38"), 56.5266)
        EdgeModel _82_83 (find(this.nodes, "_82"), find(this.nodes, "_83"), 30.1205)
        EdgeModel _81_36 (find(this.nodes, "_81"), find(this.nodes, "_36"), 44.2575)
        EdgeModel _80_79 (find(this.nodes, "_80"), find(this.nodes, "_79"), 21.1435)
        EdgeModel _79_78 (find(this.nodes, "_79"), find(this.nodes, "_78"), 44.4053)
        EdgeModel _78_81 (find(this.nodes, "_78"), find(this.nodes, "_81"), 33.1313)
        EdgeModel _77_32 (find(this.nodes, "_77"), find(this.nodes, "_32"), 30.0201)
        EdgeModel _77_47 (find(this.nodes, "_77"), find(this.nodes, "_47"), 102.932)
        EdgeModel _76_48 (find(this.nodes, "_76"), find(this.nodes, "_48"), 80.2175)
        EdgeModel _76_33 (find(this.nodes, "_76"), find(this.nodes, "_33"), 110.503)
        EdgeModel _75_2 (find(this.nodes, "_75"), find(this.nodes, "_2"), 16.8779)
        EdgeModel _74_106 (find(this.nodes, "_74"), find(this.nodes, "_106"), 45.7616)
        EdgeModel _74_0 (find(this.nodes, "_74"), find(this.nodes, "_0"), 19.5216)
        EdgeModel _72_3 (find(this.nodes, "_72"), find(this.nodes, "_3"), 31.601)
        EdgeModel _72_73 (find(this.nodes, "_72"), find(this.nodes, "_73"), 104.723)
        EdgeModel _71_101 (find(this.nodes, "_71"), find(this.nodes, "_101"), 100.104)
        EdgeModel _70_9 (find(this.nodes, "_70"), find(this.nodes, "_9"), 56.5335)
        EdgeModel _70_25 (find(this.nodes, "_70"), find(this.nodes, "_25"), 61.0522)
        EdgeModel _70_69 (find(this.nodes, "_70"), find(this.nodes, "_69"), 51.6788)
        EdgeModel _69_66 (find(this.nodes, "_69"), find(this.nodes, "_66"), 63.4315)
        EdgeModel _69_5 (find(this.nodes, "_69"), find(this.nodes, "_5"), 14.9078)
        EdgeModel _68_70 (find(this.nodes, "_68"), find(this.nodes, "_70"), 36.8899)
        EdgeModel _67_68 (find(this.nodes, "_67"), find(this.nodes, "_68"), 28.6447)
        EdgeModel _67_6 (find(this.nodes, "_67"), find(this.nodes, "_6"), 30.4569)
        EdgeModel _65_105 (find(this.nodes, "_65"), find(this.nodes, "_105"), 14.0152)
        EdgeModel _64_26 (find(this.nodes, "_64"), find(this.nodes, "_26"), 55.6234)
        EdgeModel _64_103 (find(this.nodes, "_64"), find(this.nodes, "_103"), 19.614)
        EdgeModel _63_64 (find(this.nodes, "_63"), find(this.nodes, "_64"), 67.0735)
        EdgeModel _62_63 (find(this.nodes, "_62"), find(this.nodes, "_63"), 106.603)
        EdgeModel _61_104 (find(this.nodes, "_61"), find(this.nodes, "_104"), 20.7912)
        EdgeModel _60_101 (find(this.nodes, "_60"), find(this.nodes, "_101"), 171.189)
        EdgeModel _59_92 (find(this.nodes, "_59"), find(this.nodes, "_92"), 14.5567)
        EdgeModel _59_102 (find(this.nodes, "_59"), find(this.nodes, "_102"), 78.1447)
        EdgeModel _58_94 (find(this.nodes, "_58"), find(this.nodes, "_94"), 55.8336)
        EdgeModel _58_59 (find(this.nodes, "_58"), find(this.nodes, "_59"), 39.8373)
        EdgeModel _56_97 (find(this.nodes, "_56"), find(this.nodes, "_97"), 21.0717)
        EdgeModel _56_72 (find(this.nodes, "_56"), find(this.nodes, "_72"), 113.491)
        EdgeModel _55_94 (find(this.nodes, "_55"), find(this.nodes, "_94"), 30.9432)
        EdgeModel _54_17 (find(this.nodes, "_54"), find(this.nodes, "_17"), 21.7711)
        EdgeModel _54_16 (find(this.nodes, "_54"), find(this.nodes, "_16"), 21.4194)
        EdgeModel _53_52 (find(this.nodes, "_53"), find(this.nodes, "_52"), 43.8774)
        EdgeModel _53_88 (find(this.nodes, "_53"), find(this.nodes, "_88"), 48.7528)
        EdgeModel _52_95 (find(this.nodes, "_52"), find(this.nodes, "_95"), 23.9674)
        EdgeModel _51_50 (find(this.nodes, "_51"), find(this.nodes, "_50"), 37.6019)
        EdgeModel _50_96 (find(this.nodes, "_50"), find(this.nodes, "_96"), 24.1791)
        EdgeModel _49_96 (find(this.nodes, "_49"), find(this.nodes, "_96"), 90.8357)
        EdgeModel _48_47 (find(this.nodes, "_48"), find(this.nodes, "_47"), 44.0594)
        EdgeModel _47_100 (find(this.nodes, "_47"), find(this.nodes, "_100"), 118.66)
        EdgeModel _46_24 (find(this.nodes, "_46"), find(this.nodes, "_24"), 56.9391)
        EdgeModel _45_44 (find(this.nodes, "_45"), find(this.nodes, "_44"), 38.5999)
        EdgeModel _44_46 (find(this.nodes, "_44"), find(this.nodes, "_46"), 38.091)
        EdgeModel _44_12 (find(this.nodes, "_44"), find(this.nodes, "_12"), 36.9842)
        EdgeModel _43_55 (find(this.nodes, "_43"), find(this.nodes, "_55"), 121.867)
        EdgeModel _43_89 (find(this.nodes, "_43"), find(this.nodes, "_89"), 39.2359)
        EdgeModel _43_11 (find(this.nodes, "_43"), find(this.nodes, "_11"), 35.9173)
        EdgeModel _42_15 (find(this.nodes, "_42"), find(this.nodes, "_15"), 31.8246)
        EdgeModel _42_13 (find(this.nodes, "_42"), find(this.nodes, "_13"), 36.3684)
        EdgeModel _41_45 (find(this.nodes, "_41"), find(this.nodes, "_45"), 43.2804)
        EdgeModel _40_41 (find(this.nodes, "_40"), find(this.nodes, "_41"), 69.6569)
        EdgeModel _40_44 (find(this.nodes, "_40"), find(this.nodes, "_44"), 93.9448)
        EdgeModel _40_14 (find(this.nodes, "_40"), find(this.nodes, "_14"), 29.5244)
        EdgeModel _39_82 (find(this.nodes, "_39"), find(this.nodes, "_82"), 17.3108)
        EdgeModel _39_17 (find(this.nodes, "_39"), find(this.nodes, "_17"), 23.1973)
        EdgeModel _39_87 (find(this.nodes, "_39"), find(this.nodes, "_87"), 25.2721)
        EdgeModel _38_40 (find(this.nodes, "_38"), find(this.nodes, "_40"), 58.0271)
        EdgeModel _37_82 (find(this.nodes, "_37"), find(this.nodes, "_82"), 19.1894)
        EdgeModel _37_84 (find(this.nodes, "_37"), find(this.nodes, "_84"), 16.7778)
        EdgeModel _36_85 (find(this.nodes, "_36"), find(this.nodes, "_85"), 39.4976)
        EdgeModel _36_38 (find(this.nodes, "_36"), find(this.nodes, "_38"), 48.2751)
        EdgeModel _36_80 (find(this.nodes, "_36"), find(this.nodes, "_80"), 20.5998)
        EdgeModel _34_78 (find(this.nodes, "_34"), find(this.nodes, "_78"), 20.6994)
        EdgeModel _32_76 (find(this.nodes, "_32"), find(this.nodes, "_76"), 28.464)
        EdgeModel _31_77 (find(this.nodes, "_31"), find(this.nodes, "_77"), 49.4087)
        EdgeModel _30_21 (find(this.nodes, "_30"), find(this.nodes, "_21"), 24.3918)
        EdgeModel _29_30 (find(this.nodes, "_29"), find(this.nodes, "_30"), 214.628)
        EdgeModel _28_22 (find(this.nodes, "_28"), find(this.nodes, "_22"), 46.2805)
        EdgeModel _27_28 (find(this.nodes, "_27"), find(this.nodes, "_28"), 48.5232)
        EdgeModel _26_104 (find(this.nodes, "_26"), find(this.nodes, "_104"), 101.586)
        EdgeModel _26_103 (find(this.nodes, "_26"), find(this.nodes, "_103"), 59.8659)
        EdgeModel _25_51 (find(this.nodes, "_25"), find(this.nodes, "_51"), 67.2002)
        EdgeModel _24_90 (find(this.nodes, "_24"), find(this.nodes, "_90"), 36.4749)
        EdgeModel _24_43 (find(this.nodes, "_24"), find(this.nodes, "_43"), 50.1059)
        EdgeModel _23_18 (find(this.nodes, "_23"), find(this.nodes, "_18"), 17.55)
        EdgeModel _22_29 (find(this.nodes, "_22"), find(this.nodes, "_29"), 122.646)
        EdgeModel _21_20 (find(this.nodes, "_21"), find(this.nodes, "_20"), 17.7584)
        EdgeModel _21_23 (find(this.nodes, "_21"), find(this.nodes, "_23"), 44.7778)
        EdgeModel _20_19 (find(this.nodes, "_20"), find(this.nodes, "_19"), 63.2881)
        EdgeModel _18_34 (find(this.nodes, "_18"), find(this.nodes, "_34"), 30.2488)
        EdgeModel _18_33 (find(this.nodes, "_18"), find(this.nodes, "_33"), 72.6205)
        EdgeModel _17_14 (find(this.nodes, "_17"), find(this.nodes, "_14"), 16.011)
        EdgeModel _16_15 (find(this.nodes, "_16"), find(this.nodes, "_15"), 16.0109)
        EdgeModel _16_10 (find(this.nodes, "_16"), find(this.nodes, "_10"), 29.9136)
        EdgeModel _16_88 (find(this.nodes, "_16"), find(this.nodes, "_88"), 26.7116)
        EdgeModel _15_10 (find(this.nodes, "_15"), find(this.nodes, "_10"), 31.0289)
        EdgeModel _15_17 (find(this.nodes, "_15"), find(this.nodes, "_17"), 29.2559)
        EdgeModel _14_13 (find(this.nodes, "_14"), find(this.nodes, "_13"), 19.1998)
        EdgeModel _14_39 (find(this.nodes, "_14"), find(this.nodes, "_39"), 19.3396)
        EdgeModel _14_12 (find(this.nodes, "_14"), find(this.nodes, "_12"), 60.9751)
        EdgeModel _14_83 (find(this.nodes, "_14"), find(this.nodes, "_83"), 16.8636)
        EdgeModel _13_15 (find(this.nodes, "_13"), find(this.nodes, "_15"), 22.4794)
        EdgeModel _12_42 (find(this.nodes, "_12"), find(this.nodes, "_42"), 13.9923)
        EdgeModel _11_46 (find(this.nodes, "_11"), find(this.nodes, "_46"), 43.6245)
        EdgeModel _11_44 (find(this.nodes, "_11"), find(this.nodes, "_44"), 12.6501)
        EdgeModel _10_43 (find(this.nodes, "_10"), find(this.nodes, "_43"), 24.2607)
        EdgeModel _9_50 (find(this.nodes, "_9"), find(this.nodes, "_50"), 35.0629)
        EdgeModel _8_86 (find(this.nodes, "_8"), find(this.nodes, "_86"), 52.2032)
        EdgeModel _7_66 (find(this.nodes, "_7"), find(this.nodes, "_66"), 30.7221)
        EdgeModel _7_5 (find(this.nodes, "_7"), find(this.nodes, "_5"), 26.7463)
        EdgeModel _6_7 (find(this.nodes, "_6"), find(this.nodes, "_7"), 14.8963)
        EdgeModel _5_6 (find(this.nodes, "_5"), find(this.nodes, "_6"), 18.957)
        EdgeModel _5_68 (find(this.nodes, "_5"), find(this.nodes, "_68"), 29.8985)
        EdgeModel _4_61 (find(this.nodes, "_4"), find(this.nodes, "_61"), 74.9735)
        EdgeModel _3_71 (find(this.nodes, "_3"), find(this.nodes, "_71"), 16.0983)
        EdgeModel _2_1 (find(this.nodes, "_2"), find(this.nodes, "_1"), 73.9637)
        EdgeModel _1_73 (find(this.nodes, "_1"), find(this.nodes, "_73"), 17.9437)
        EdgeModel _1_0 (find(this.nodes, "_1"), find(this.nodes, "_0"), 24.431)
        EdgeModel _0_75 (find(this.nodes, "_0"), find(this.nodes, "_75"), 78.195)
    }

    addChildrenTo this.edge_ids {
        String _ ("_106_65")
        String _ ("_106_0")
        String _ ("_106_73")
        String _ ("_105_26")
        String _ ("_105_64")
        String _ ("_105_74")
        String _ ("_105_106")
        String _ ("_104_63")
        String _ ("_103_106")
        String _ ("_103_66")
        String _ ("_102_56")
        String _ ("_100_4")
        String _ ("_99_49")
        String _ ("_99_67")
        String _ ("_98_48")
        String _ ("_98_62")
        String _ ("_97_57")
        String _ ("_97_3")
        String _ ("_96_52")
        String _ ("_95_55")
        String _ ("_95_51")
        String _ ("_94_59")
        String _ ("_93_57")
        String _ ("_92_93")
        String _ ("_92_90")
        String _ ("_91_60")
        String _ ("_91_92")
        String _ ("_90_59")
        String _ ("_90_91")
        String _ ("_89_58")
        String _ ("_89_24")
        String _ ("_88_54")
        String _ ("_87_54")
        String _ ("_86_84")
        String _ ("_86_82")
        String _ ("_85_37")
        String _ ("_84_35")
        String _ ("_83_40")
        String _ ("_83_39")
        String _ ("_82_38")
        String _ ("_82_83")
        String _ ("_81_36")
        String _ ("_80_79")
        String _ ("_79_78")
        String _ ("_78_81")
        String _ ("_77_32")
        String _ ("_77_47")
        String _ ("_76_48")
        String _ ("_76_33")
        String _ ("_75_2")
        String _ ("_74_106")
        String _ ("_74_0")
        String _ ("_72_3")
        String _ ("_72_73")
        String _ ("_71_101")
        String _ ("_70_9")
        String _ ("_70_25")
        String _ ("_70_69")
        String _ ("_69_66")
        String _ ("_69_5")
        String _ ("_68_70")
        String _ ("_67_68")
        String _ ("_67_6")
        String _ ("_65_105")
        String _ ("_64_26")
        String _ ("_64_103")
        String _ ("_63_64")
        String _ ("_62_63")
        String _ ("_61_104")
        String _ ("_60_101")
        String _ ("_59_92")
        String _ ("_59_102")
        String _ ("_58_94")
        String _ ("_58_59")
        String _ ("_56_97")
        String _ ("_56_72")
        String _ ("_55_94")
        String _ ("_54_17")
        String _ ("_54_16")
        String _ ("_53_52")
        String _ ("_53_88")
        String _ ("_52_95")
        String _ ("_51_50")
        String _ ("_50_96")
        String _ ("_49_96")
        String _ ("_48_47")
        String _ ("_47_100")
        String _ ("_46_24")
        String _ ("_45_44")
        String _ ("_44_46")
        String _ ("_44_12")
        String _ ("_43_55")
        String _ ("_43_89")
        String _ ("_43_11")
        String _ ("_42_15")
        String _ ("_42_13")
        String _ ("_41_45")
        String _ ("_40_41")
        String _ ("_40_44")
        String _ ("_40_14")
        String _ ("_39_82")
        String _ ("_39_17")
        String _ ("_39_87")
        String _ ("_38_40")
        String _ ("_37_82")
        String _ ("_37_84")
        String _ ("_36_85")
        String _ ("_36_38")
        String _ ("_36_80")
        String _ ("_34_78")
        String _ ("_32_76")
        String _ ("_31_77")
        String _ ("_30_21")
        String _ ("_29_30")
        String _ ("_28_22")
        String _ ("_27_28")
        String _ ("_26_104")
        String _ ("_26_103")
        String _ ("_25_51")
        String _ ("_24_90")
        String _ ("_24_43")
        String _ ("_23_18")
        String _ ("_22_29")
        String _ ("_21_20")
        String _ ("_21_23")
        String _ ("_20_19")
        String _ ("_18_34")
        String _ ("_18_33")
        String _ ("_17_14")
        String _ ("_16_15")
        String _ ("_16_10")
        String _ ("_16_88")
        String _ ("_15_10")
        String _ ("_15_17")
        String _ ("_14_13")
        String _ ("_14_39")
        String _ ("_14_12")
        String _ ("_14_83")
        String _ ("_13_15")
        String _ ("_12_42")
        String _ ("_11_46")
        String _ ("_11_44")
        String _ ("_10_43")
        String _ ("_9_50")
        String _ ("_8_86")
        String _ ("_7_66")
        String _ ("_7_5")
        String _ ("_6_7")
        String _ ("_5_6")
        String _ ("_5_68")
        String _ ("_4_61")
        String _ ("_3_71")
        String _ ("_2_1")
        String _ ("_1_73")
        String _ ("_1_0")
        String _ ("_0_75")
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

            addChildrenTo this.task_traps {
                TaskTrapModel debug_task_trap1 (this.traps.[1])
            }
        }
    }
    set_trap2 -> (this) {
        if (this.traps.size > 1) {
            this.traps.[2].description = "Ceci est le trap 2"
            this.traps.[2].contact_mode = 2
            this.traps.[2].contact_deactivate = 1

            addChildrenTo this.task_traps {
                TaskTrapModel debug_task_trap2 (this.traps.[2])
            }
        }
    }

}