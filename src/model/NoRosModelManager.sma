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
import task.TaskEdgeModel

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
            String _ ("_27")
            String _ ("_28")
            String _ ("_22")
            String _ ("_29")
            String _ ("_30")
            String _ ("_21")
            String _ ("_23")
        }
    }

    set_safest_itinerary -> {
        "b5ac3bb1-593c-4dff-b1bd-9dcca5c39ab2" =: this.safest_itinerary.uid
        "Planning safest OK: a path including PPOs was found with cost 501.248" =: this.safest_itinerary.description_input
    }
    set_safest_itinerary -> na_set_safest:(this) {
        addChildrenTo this.safest_itinerary.node_ids {
            String _ ("_27")
            String _ ("_28")
            String _ ("_22")
            String _ ("_29")
        }
    }

    set_tradeoff_itinerary -> {
        "4e5aa5e6-0ed4-430f-9094-d0e8295594e7" =: this.tradeoff_itinerary.uid
        "Planning tradeoff OK: a path including PPOs was found with cost 450.050" =: this.tradeoff_itinerary.description_input
    }
    set_tradeoff_itinerary -> na_set_tradeoff:(this) {
        addChildrenTo this.tradeoff_itinerary.node_ids {
            String _ ("_29")
            String _ ("_30")
            String _ ("_21")
            String _ ("_23")
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
        NodeModel _106 ("106", 4, "", 48.8661, 1.88491, 0, 0)
        NodeModel _105 ("105", 4, "", 48.8662, 1.88452, 0, 0)
        NodeModel _104 ("104", 4, "", 48.8653, 1.88409, 0, 0)
        NodeModel _103 ("103", 4, "", 48.8659, 1.88456, 0, 0)
        NodeModel _102 ("102", 3, "", 48.8643, 1.88843, 0, 0)
        NodeModel _101 ("101", 3, "", 48.865, 1.88793, 0, 0)
        NodeModel _100 ("100", 3, "", 48.8641, 1.8848, 0, 0)
        NodeModel _99 ("99", 3, "", 48.8643, 1.88555, 0, 0)
        NodeModel _98 ("98", 3, "", 48.8642, 1.88519, 0, 0)
        NodeModel _97 ("97", 3, "", 48.865, 1.88763, 0, 0)
        NodeModel _96 ("96", 3, "", 48.8643, 1.88677, 0, 0)
        NodeModel _95 ("95", 3, "", 48.8641, 1.88778, 0, 0)
        NodeModel _94 ("94", 3, "", 48.8639, 1.88853, 0, 0)
        NodeModel _93 ("93", 3, "", 48.8641, 1.88911, 0, 0)
        NodeModel _92 ("92", 3, "", 48.8638, 1.88941, 0, 0)
        NodeModel _91 ("91", 3, "", 48.8636, 1.89008, 0, 0)
        NodeModel _90 ("90", 3, "", 48.8636, 1.88958, 0, 0)
        NodeModel _89 ("89", 3, "", 48.8634, 1.88913, 0, 0)
        NodeModel _88 ("88", 3, "", 48.8632, 1.88823, 0, 0)
        NodeModel _87 ("87", 3, "", 48.8627, 1.88789, 0, 0)
        NodeModel _86 ("86", 3, "", 48.8627, 1.88742, 0, 0)
        NodeModel _85 ("85", 2, "", 48.8621, 1.88731, 0, 0)
        NodeModel _84 ("84", 2, "", 48.8624, 1.88718, 0, 0)
        NodeModel _83 ("83", 2, "", 48.8624, 1.88804, 0, 0)
        NodeModel _82 ("82", 2, "", 48.8625, 1.88765, 0, 0)
        NodeModel _81 ("81", 2, "", 48.8613, 1.88707, 0, 0)
        NodeModel _80 ("80", 2, "", 48.8616, 1.88737, 0, 0)
        NodeModel _79 ("79", 2, "", 48.8614, 1.88727, 0, 0)
        NodeModel _78 ("78", 2, "", 48.861, 1.88718, 0, 0)
        NodeModel _77 ("77", 2, "", 48.8621, 1.88525, 0, 0)
        NodeModel _76 ("76", 2, "", 48.8624, 1.88598, 0, 0)
        NodeModel _75 ("75", 4, "T415", 48.8668, 1.88585, 0, 0)
        NodeModel _74 ("74", 4, "T414", 48.8665, 1.88496, 0, 0)
        NodeModel _73 ("73", 4, "T413", 48.8661, 1.88517, 0, 0)
        NodeModel _72 ("72", 4, "T412", 48.8655, 1.88622, 0, 0)
        NodeModel _71 ("71", 4, "T411", 48.8655, 1.88677, 0, 0)
        NodeModel _70 ("70", 4, "T410", 48.865, 1.88608, 0, 0)
        NodeModel _69 ("69", 4, "T409", 48.8653, 1.88556, 0, 0)
        NodeModel _68 ("68", 4, "T408", 48.8651, 1.88559, 0, 0)
        NodeModel _67 ("67", 4, "T407", 48.865, 1.88521, 0, 0)
        NodeModel _66 ("66", 4, "T406", 48.8657, 1.88487, 0, 0)
        NodeModel _65 ("65", 4, "T405", 48.8663, 1.88454, 0, 0)
        NodeModel _64 ("64", 4, "T404", 48.8658, 1.88435, 0, 0)
        NodeModel _63 ("63", 4, "T403", 48.8653, 1.88463, 0, 0)
        NodeModel _62 ("62", 4, "T402", 48.8644, 1.88512, 0, 0)
        NodeModel _61 ("61", 4, "T401", 48.8651, 1.88414, 0, 0)
        NodeModel _60 ("60", 3, "T314", 48.864, 1.88964, 0, 0)
        NodeModel _59 ("59", 3, "T313", 48.8638, 1.88922, 0, 0)
        NodeModel _58 ("58", 3, "T312", 48.8635, 1.88897, 0, 0)
        NodeModel _57 ("57", 3, "T311", 48.8644, 1.88859, 0, 0)
        NodeModel _56 ("56", 3, "T310", 48.8648, 1.88741, 0, 0)
        NodeModel _55 ("55", 3, "T309", 48.8639, 1.88811, 0, 0)
        NodeModel _54 ("54", 3, "T308", 48.8629, 1.88813, 0, 0)
        NodeModel _53 ("53", 3, "T307", 48.8636, 1.88802, 0, 0)
        NodeModel _52 ("52", 3, "T306", 48.8639, 1.88761, 0, 0)
        NodeModel _51 ("51", 3, "T305", 48.8643, 1.88743, 0, 0)
        NodeModel _50 ("50", 3, "T304", 48.8645, 1.88698, 0, 0)
        NodeModel _49 ("49", 3, "T303", 48.8637, 1.88599, 0, 0)
        NodeModel _48 ("48", 3, "T302", 48.863, 1.88569, 0, 0)
        NodeModel _47 ("47", 3, "T301", 48.8631, 1.88509, 0, 0)
        NodeModel _46 ("46", 2, "T216", 48.8628, 1.88996, 0, 0)
        NodeModel _45 ("45", 2, "T215", 48.8624, 1.88974, 0, 0)
        NodeModel _44 ("44", 2, "T214", 48.8627, 1.88946, 0, 0)
        NodeModel _43 ("43", 2, "T213", 48.863, 1.8891, 0, 0)
        NodeModel _42 ("42", 2, "T212", 48.8627, 1.88883, 0, 0)
        NodeModel _41 ("41", 2, "T211", 48.8622, 1.88925, 0, 0)
        NodeModel _40 ("40", 2, "T210", 48.8623, 1.88832, 0, 0)
        NodeModel _39 ("39", 2, "T209", 48.8625, 1.88788, 0, 0)
        NodeModel _38 ("38", 2, "T208", 48.862, 1.88769, 0, 0)
        NodeModel _37 ("37", 2, "T207", 48.8624, 1.88741, 0, 0)
        NodeModel _36 ("36", 2, "T206", 48.8617, 1.88715, 0, 0)
        NodeModel _35 ("35", 2, "T205", 48.8622, 1.88666, 0, 0)
        NodeModel _34 ("34", 2, "T204", 48.8609, 1.88735, 0, 0)
        NodeModel _33 ("33", 2, "T203", 48.8614, 1.8866, 0, 0)
        NodeModel _32 ("32", 2, "T202", 48.8622, 1.88563, 0, 0)
        NodeModel _31 ("31", 2, "T201", 48.8619, 1.88468, 0, 0)
        NodeModel _30 ("30", 1, "T103", 48.8603, 1.88755, 0, 0)
        NodeModel _29 ("29", 1, "T102", 48.8608, 1.8904, 0, 0)
        NodeModel _28 ("28", 1, "T101", 48.861, 1.89266, 0, 0)
        NodeModel _27 ("27", 0, "depart", 48.8611, 1.89331, 0, 1)
        NodeModel _26 ("26", 4, "PPO40", 48.8662, 1.88387, 0, 1)
        NodeModel _25 ("25", 3, "PPO30", 48.8647, 1.88677, 0, 1)
        NodeModel _24 ("24", 2, "PPO20", 48.8632, 1.88971, 0, 1)
        NodeModel _23 ("23", 1, "PPO10", 48.8607, 1.88702, 0, 1)
        NodeModel _22 ("22", 1, "", 48.861, 1.89203, 0, 0)
        NodeModel _21 ("21", 1, "", 48.8603, 1.88722, 0, 0)
        NodeModel _20 ("20", 1, "", 48.8603, 1.88699, 0, 0)
        NodeModel _19 ("19", 1, "", 48.8597, 1.88709, 0, 0)
        NodeModel _18 ("18", 2, "", 48.8608, 1.88694, 0, 0)
        NodeModel _17 ("17", 2, "", 48.8627, 1.8881, 0, 0)
        NodeModel _16 ("16", 3, "", 48.863, 1.88838, 0, 0)
        NodeModel _15 ("15", 2, "", 48.8628, 1.88843, 0, 0)
        NodeModel _14 ("14", 2, "", 48.8625, 1.88815, 0, 0)
        NodeModel _13 ("13", 2, "", 48.8626, 1.88836, 0, 0)
        NodeModel _12 ("12", 2, "", 48.8626, 1.88896, 0, 0)
        NodeModel _11 ("11", 2, "", 48.8628, 1.88937, 0, 0)
        NodeModel _10 ("10", 2, "", 48.863, 1.88879, 0, 0)
        NodeModel _9 ("9", 3, "", 48.8646, 1.8866, 0, 0)
        NodeModel _8 ("8", 3, "", 48.8631, 1.88716, 0, 0)
        NodeModel _7 ("7", 4, "", 48.8654, 1.88504, 0, 0)
        NodeModel _6 ("6", 4, "", 48.8653, 1.8851, 0, 0)
        NodeModel _5 ("5", 4, "", 48.8653, 1.88536, 0, 0)
        NodeModel _4 ("4", 4, "", 48.8645, 1.88433, 0, 0)
        NodeModel _3 ("3", 4, "", 48.8654, 1.88662, 0, 0)
        NodeModel _2 ("2", 4, "", 48.8667, 1.88602, 0, 0)
        NodeModel _1 ("1", 4, "", 48.8662, 1.88534, 0, 0)
        NodeModel _0 ("0", 4, "", 48.8664, 1.88506, 0, 0)
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
        EdgeModel _106__65 (find(this.nodes, "_106"), find(this.nodes, "_65"), 36.1518)
        EdgeModel _106__0 (find(this.nodes, "_106"), find(this.nodes, "_0"), 29.5981)
        EdgeModel _106__73 (find(this.nodes, "_106"), find(this.nodes, "_73"), 19.4721)
        EdgeModel _105__26 (find(this.nodes, "_105"), find(this.nodes, "_26"), 47.4702)
        EdgeModel _105__64 (find(this.nodes, "_105"), find(this.nodes, "_64"), 43.218)
        EdgeModel _105__74 (find(this.nodes, "_105"), find(this.nodes, "_74"), 47.8072)
        EdgeModel _105__106 (find(this.nodes, "_105"), find(this.nodes, "_106"), 30.1183)
        EdgeModel _104__63 (find(this.nodes, "_104"), find(this.nodes, "_63"), 40.2597)
        EdgeModel _103__106 (find(this.nodes, "_103"), find(this.nodes, "_106"), 31.9294)
        EdgeModel _103__66 (find(this.nodes, "_103"), find(this.nodes, "_66"), 37.3526)
        EdgeModel _102__56 (find(this.nodes, "_102"), find(this.nodes, "_56"), 95.0007)
        EdgeModel _100__4 (find(this.nodes, "_100"), find(this.nodes, "_4"), 54.1307)
        EdgeModel _99__49 (find(this.nodes, "_99"), find(this.nodes, "_49"), 82.4335)
        EdgeModel _99__67 (find(this.nodes, "_99"), find(this.nodes, "_67"), 81.3112)
        EdgeModel _98__48 (find(this.nodes, "_98"), find(this.nodes, "_48"), 136.543)
        EdgeModel _98__62 (find(this.nodes, "_98"), find(this.nodes, "_62"), 15.8725)
        EdgeModel _97__57 (find(this.nodes, "_97"), find(this.nodes, "_57"), 92.5662)
        EdgeModel _97__3 (find(this.nodes, "_97"), find(this.nodes, "_3"), 88.1322)
        EdgeModel _96__52 (find(this.nodes, "_96"), find(this.nodes, "_52"), 75.7454)
        EdgeModel _95__55 (find(this.nodes, "_95"), find(this.nodes, "_55"), 28.8409)
        EdgeModel _95__51 (find(this.nodes, "_95"), find(this.nodes, "_51"), 36.0707)
        EdgeModel _94__59 (find(this.nodes, "_94"), find(this.nodes, "_59"), 51.3599)
        EdgeModel _93__57 (find(this.nodes, "_93"), find(this.nodes, "_57"), 48.6935)
        EdgeModel _92__93 (find(this.nodes, "_92"), find(this.nodes, "_93"), 42.9731)
        EdgeModel _92__90 (find(this.nodes, "_92"), find(this.nodes, "_90"), 30.2211)
        EdgeModel _91__60 (find(this.nodes, "_91"), find(this.nodes, "_60"), 49.1265)
        EdgeModel _91__92 (find(this.nodes, "_91"), find(this.nodes, "_92"), 52.3149)
        EdgeModel _90__59 (find(this.nodes, "_90"), find(this.nodes, "_59"), 40.4231)
        EdgeModel _90__91 (find(this.nodes, "_90"), find(this.nodes, "_91"), 37.2258)
        EdgeModel _89__58 (find(this.nodes, "_89"), find(this.nodes, "_58"), 18.4513)
        EdgeModel _89__24 (find(this.nodes, "_89"), find(this.nodes, "_24"), 45.2469)
        EdgeModel _88__54 (find(this.nodes, "_88"), find(this.nodes, "_54"), 36.3603)
        EdgeModel _87__54 (find(this.nodes, "_87"), find(this.nodes, "_54"), 22.2335)
        EdgeModel _86__84 (find(this.nodes, "_86"), find(this.nodes, "_84"), 35.8564)
        EdgeModel _86__82 (find(this.nodes, "_86"), find(this.nodes, "_82"), 28.6018)
        EdgeModel _85__37 (find(this.nodes, "_85"), find(this.nodes, "_37"), 40.2236)
        EdgeModel _84__35 (find(this.nodes, "_84"), find(this.nodes, "_35"), 47.0857)
        EdgeModel _83__40 (find(this.nodes, "_83"), find(this.nodes, "_40"), 23.921)
        EdgeModel _83__39 (find(this.nodes, "_83"), find(this.nodes, "_39"), 18.0651)
        EdgeModel _82__38 (find(this.nodes, "_82"), find(this.nodes, "_38"), 56.5266)
        EdgeModel _82__83 (find(this.nodes, "_82"), find(this.nodes, "_83"), 30.1205)
        EdgeModel _81__36 (find(this.nodes, "_81"), find(this.nodes, "_36"), 44.2575)
        EdgeModel _80__79 (find(this.nodes, "_80"), find(this.nodes, "_79"), 21.1435)
        EdgeModel _79__78 (find(this.nodes, "_79"), find(this.nodes, "_78"), 44.4053)
        EdgeModel _78__81 (find(this.nodes, "_78"), find(this.nodes, "_81"), 33.1313)
        EdgeModel _77__32 (find(this.nodes, "_77"), find(this.nodes, "_32"), 30.0201)
        EdgeModel _77__47 (find(this.nodes, "_77"), find(this.nodes, "_47"), 102.932)
        EdgeModel _76__48 (find(this.nodes, "_76"), find(this.nodes, "_48"), 80.2175)
        EdgeModel _76__33 (find(this.nodes, "_76"), find(this.nodes, "_33"), 110.503)
        EdgeModel _75__2 (find(this.nodes, "_75"), find(this.nodes, "_2"), 16.8779)
        EdgeModel _74__106 (find(this.nodes, "_74"), find(this.nodes, "_106"), 45.7616)
        EdgeModel _74__0 (find(this.nodes, "_74"), find(this.nodes, "_0"), 19.5216)
        EdgeModel _72__3 (find(this.nodes, "_72"), find(this.nodes, "_3"), 31.601)
        EdgeModel _72__73 (find(this.nodes, "_72"), find(this.nodes, "_73"), 104.723)
        EdgeModel _71__101 (find(this.nodes, "_71"), find(this.nodes, "_101"), 100.104)
        EdgeModel _70__9 (find(this.nodes, "_70"), find(this.nodes, "_9"), 56.5335)
        EdgeModel _70__25 (find(this.nodes, "_70"), find(this.nodes, "_25"), 61.0522)
        EdgeModel _70__69 (find(this.nodes, "_70"), find(this.nodes, "_69"), 51.6788)
        EdgeModel _69__66 (find(this.nodes, "_69"), find(this.nodes, "_66"), 63.4315)
        EdgeModel _69__5 (find(this.nodes, "_69"), find(this.nodes, "_5"), 14.9078)
        EdgeModel _68__70 (find(this.nodes, "_68"), find(this.nodes, "_70"), 36.8899)
        EdgeModel _67__68 (find(this.nodes, "_67"), find(this.nodes, "_68"), 28.6447)
        EdgeModel _67__6 (find(this.nodes, "_67"), find(this.nodes, "_6"), 30.4569)
        EdgeModel _65__105 (find(this.nodes, "_65"), find(this.nodes, "_105"), 14.0152)
        EdgeModel _64__26 (find(this.nodes, "_64"), find(this.nodes, "_26"), 55.6234)
        EdgeModel _64__103 (find(this.nodes, "_64"), find(this.nodes, "_103"), 19.614)
        EdgeModel _63__64 (find(this.nodes, "_63"), find(this.nodes, "_64"), 67.0735)
        EdgeModel _62__63 (find(this.nodes, "_62"), find(this.nodes, "_63"), 106.603)
        EdgeModel _61__104 (find(this.nodes, "_61"), find(this.nodes, "_104"), 20.7912)
        EdgeModel _60__101 (find(this.nodes, "_60"), find(this.nodes, "_101"), 171.189)
        EdgeModel _59__92 (find(this.nodes, "_59"), find(this.nodes, "_92"), 14.5567)
        EdgeModel _59__102 (find(this.nodes, "_59"), find(this.nodes, "_102"), 78.1447)
        EdgeModel _58__94 (find(this.nodes, "_58"), find(this.nodes, "_94"), 55.8336)
        EdgeModel _58__59 (find(this.nodes, "_58"), find(this.nodes, "_59"), 39.8373)
        EdgeModel _56__97 (find(this.nodes, "_56"), find(this.nodes, "_97"), 21.0717)
        EdgeModel _56__72 (find(this.nodes, "_56"), find(this.nodes, "_72"), 113.491)
        EdgeModel _55__94 (find(this.nodes, "_55"), find(this.nodes, "_94"), 30.9432)
        EdgeModel _54__17 (find(this.nodes, "_54"), find(this.nodes, "_17"), 21.7711)
        EdgeModel _54__16 (find(this.nodes, "_54"), find(this.nodes, "_16"), 21.4194)
        EdgeModel _53__52 (find(this.nodes, "_53"), find(this.nodes, "_52"), 43.8774)
        EdgeModel _53__88 (find(this.nodes, "_53"), find(this.nodes, "_88"), 48.7528)
        EdgeModel _52__95 (find(this.nodes, "_52"), find(this.nodes, "_95"), 23.9674)
        EdgeModel _51__50 (find(this.nodes, "_51"), find(this.nodes, "_50"), 37.6019)
        EdgeModel _50__96 (find(this.nodes, "_50"), find(this.nodes, "_96"), 24.1791)
        EdgeModel _49__96 (find(this.nodes, "_49"), find(this.nodes, "_96"), 90.8357)
        EdgeModel _48__47 (find(this.nodes, "_48"), find(this.nodes, "_47"), 44.0594)
        EdgeModel _47__100 (find(this.nodes, "_47"), find(this.nodes, "_100"), 118.66)
        EdgeModel _46__24 (find(this.nodes, "_46"), find(this.nodes, "_24"), 56.9391)
        EdgeModel _45__44 (find(this.nodes, "_45"), find(this.nodes, "_44"), 38.5999)
        EdgeModel _44__46 (find(this.nodes, "_44"), find(this.nodes, "_46"), 38.091)
        EdgeModel _44__12 (find(this.nodes, "_44"), find(this.nodes, "_12"), 36.9842)
        EdgeModel _43__55 (find(this.nodes, "_43"), find(this.nodes, "_55"), 121.867)
        EdgeModel _43__89 (find(this.nodes, "_43"), find(this.nodes, "_89"), 39.2359)
        EdgeModel _43__11 (find(this.nodes, "_43"), find(this.nodes, "_11"), 35.9173)
        EdgeModel _42__15 (find(this.nodes, "_42"), find(this.nodes, "_15"), 31.8246)
        EdgeModel _42__13 (find(this.nodes, "_42"), find(this.nodes, "_13"), 36.3684)
        EdgeModel _41__45 (find(this.nodes, "_41"), find(this.nodes, "_45"), 43.2804)
        EdgeModel _40__41 (find(this.nodes, "_40"), find(this.nodes, "_41"), 69.6569)
        EdgeModel _40__44 (find(this.nodes, "_40"), find(this.nodes, "_44"), 93.9448)
        EdgeModel _40__14 (find(this.nodes, "_40"), find(this.nodes, "_14"), 29.5244)
        EdgeModel _39__82 (find(this.nodes, "_39"), find(this.nodes, "_82"), 17.3108)
        EdgeModel _39__17 (find(this.nodes, "_39"), find(this.nodes, "_17"), 23.1973)
        EdgeModel _39__87 (find(this.nodes, "_39"), find(this.nodes, "_87"), 25.2721)
        EdgeModel _38__40 (find(this.nodes, "_38"), find(this.nodes, "_40"), 58.0271)
        EdgeModel _37__82 (find(this.nodes, "_37"), find(this.nodes, "_82"), 19.1894)
        EdgeModel _37__84 (find(this.nodes, "_37"), find(this.nodes, "_84"), 16.7778)
        EdgeModel _36__85 (find(this.nodes, "_36"), find(this.nodes, "_85"), 39.4976)
        EdgeModel _36__38 (find(this.nodes, "_36"), find(this.nodes, "_38"), 48.2751)
        EdgeModel _36__80 (find(this.nodes, "_36"), find(this.nodes, "_80"), 20.5998)
        EdgeModel _34__78 (find(this.nodes, "_34"), find(this.nodes, "_78"), 20.6994)
        EdgeModel _32__76 (find(this.nodes, "_32"), find(this.nodes, "_76"), 28.464)
        EdgeModel _31__77 (find(this.nodes, "_31"), find(this.nodes, "_77"), 49.4087)
        EdgeModel _30__21 (find(this.nodes, "_30"), find(this.nodes, "_21"), 24.3918)
        EdgeModel _29__30 (find(this.nodes, "_29"), find(this.nodes, "_30"), 214.628)
        EdgeModel _28__22 (find(this.nodes, "_28"), find(this.nodes, "_22"), 46.2805)
        EdgeModel _27__28 (find(this.nodes, "_27"), find(this.nodes, "_28"), 48.5232)
        EdgeModel _26__104 (find(this.nodes, "_26"), find(this.nodes, "_104"), 101.586)
        EdgeModel _26__103 (find(this.nodes, "_26"), find(this.nodes, "_103"), 59.8659)
        EdgeModel _25__51 (find(this.nodes, "_25"), find(this.nodes, "_51"), 67.2002)
        EdgeModel _24__90 (find(this.nodes, "_24"), find(this.nodes, "_90"), 36.4749)
        EdgeModel _24__43 (find(this.nodes, "_24"), find(this.nodes, "_43"), 50.1059)
        EdgeModel _23__18 (find(this.nodes, "_23"), find(this.nodes, "_18"), 17.55)
        EdgeModel _22__29 (find(this.nodes, "_22"), find(this.nodes, "_29"), 122.646)
        EdgeModel _21__20 (find(this.nodes, "_21"), find(this.nodes, "_20"), 17.7584)
        EdgeModel _21__23 (find(this.nodes, "_21"), find(this.nodes, "_23"), 44.7778)
        EdgeModel _20__19 (find(this.nodes, "_20"), find(this.nodes, "_19"), 63.2881)
        EdgeModel _18__34 (find(this.nodes, "_18"), find(this.nodes, "_34"), 30.2488)
        EdgeModel _18__33 (find(this.nodes, "_18"), find(this.nodes, "_33"), 72.6205)
        EdgeModel _17__14 (find(this.nodes, "_17"), find(this.nodes, "_14"), 16.011)
        EdgeModel _16__15 (find(this.nodes, "_16"), find(this.nodes, "_15"), 16.0109)
        EdgeModel _16__10 (find(this.nodes, "_16"), find(this.nodes, "_10"), 29.9136)
        EdgeModel _16__88 (find(this.nodes, "_16"), find(this.nodes, "_88"), 26.7116)
        EdgeModel _15__10 (find(this.nodes, "_15"), find(this.nodes, "_10"), 31.0289)
        EdgeModel _15__17 (find(this.nodes, "_15"), find(this.nodes, "_17"), 29.2559)
        EdgeModel _14__13 (find(this.nodes, "_14"), find(this.nodes, "_13"), 19.1998)
        EdgeModel _14__39 (find(this.nodes, "_14"), find(this.nodes, "_39"), 19.3396)
        EdgeModel _14__12 (find(this.nodes, "_14"), find(this.nodes, "_12"), 60.9751)
        EdgeModel _14__83 (find(this.nodes, "_14"), find(this.nodes, "_83"), 16.8636)
        EdgeModel _13__15 (find(this.nodes, "_13"), find(this.nodes, "_15"), 22.4794)
        EdgeModel _12__42 (find(this.nodes, "_12"), find(this.nodes, "_42"), 13.9923)
        EdgeModel _11__46 (find(this.nodes, "_11"), find(this.nodes, "_46"), 43.6245)
        EdgeModel _11__44 (find(this.nodes, "_11"), find(this.nodes, "_44"), 12.6501)
        EdgeModel _10__43 (find(this.nodes, "_10"), find(this.nodes, "_43"), 24.2607)
        EdgeModel _9__50 (find(this.nodes, "_9"), find(this.nodes, "_50"), 35.0629)
        EdgeModel _8__86 (find(this.nodes, "_8"), find(this.nodes, "_86"), 52.2032)
        EdgeModel _7__66 (find(this.nodes, "_7"), find(this.nodes, "_66"), 30.7221)
        EdgeModel _7__5 (find(this.nodes, "_7"), find(this.nodes, "_5"), 26.7463)
        EdgeModel _6__7 (find(this.nodes, "_6"), find(this.nodes, "_7"), 14.8963)
        EdgeModel _5__6 (find(this.nodes, "_5"), find(this.nodes, "_6"), 18.957)
        EdgeModel _5__68 (find(this.nodes, "_5"), find(this.nodes, "_68"), 29.8985)
        EdgeModel _4__61 (find(this.nodes, "_4"), find(this.nodes, "_61"), 74.9735)
        EdgeModel _3__71 (find(this.nodes, "_3"), find(this.nodes, "_71"), 16.0983)
        EdgeModel _2__1 (find(this.nodes, "_2"), find(this.nodes, "_1"), 73.9637)
        EdgeModel _1__73 (find(this.nodes, "_1"), find(this.nodes, "_73"), 17.9437)
        EdgeModel _1__0 (find(this.nodes, "_1"), find(this.nodes, "_0"), 24.431)
        EdgeModel _0__75 (find(this.nodes, "_0"), find(this.nodes, "_75"), 78.195)
    }

    addChildrenTo this.edge_ids {
        String _ ("_106__65")
        String _ ("_106__0")
        String _ ("_106__73")
        String _ ("_105__26")
        String _ ("_105__64")
        String _ ("_105__74")
        String _ ("_105__106")
        String _ ("_104__63")
        String _ ("_103__106")
        String _ ("_103__66")
        String _ ("_102__56")
        String _ ("_100__4")
        String _ ("_99__49")
        String _ ("_99__67")
        String _ ("_98__48")
        String _ ("_98__62")
        String _ ("_97__57")
        String _ ("_97__3")
        String _ ("_96__52")
        String _ ("_95__55")
        String _ ("_95__51")
        String _ ("_94__59")
        String _ ("_93__57")
        String _ ("_92__93")
        String _ ("_92__90")
        String _ ("_91__60")
        String _ ("_91__92")
        String _ ("_90__59")
        String _ ("_90__91")
        String _ ("_89__58")
        String _ ("_89__24")
        String _ ("_88__54")
        String _ ("_87__54")
        String _ ("_86__84")
        String _ ("_86__82")
        String _ ("_85__37")
        String _ ("_84__35")
        String _ ("_83__40")
        String _ ("_83__39")
        String _ ("_82__38")
        String _ ("_82__83")
        String _ ("_81__36")
        String _ ("_80__79")
        String _ ("_79__78")
        String _ ("_78__81")
        String _ ("_77__32")
        String _ ("_77__47")
        String _ ("_76__48")
        String _ ("_76__33")
        String _ ("_75__2")
        String _ ("_74__106")
        String _ ("_74__0")
        String _ ("_72__3")
        String _ ("_72__73")
        String _ ("_71__101")
        String _ ("_70__9")
        String _ ("_70__25")
        String _ ("_70__69")
        String _ ("_69__66")
        String _ ("_69__5")
        String _ ("_68__70")
        String _ ("_67__68")
        String _ ("_67__6")
        String _ ("_65__105")
        String _ ("_64__26")
        String _ ("_64__103")
        String _ ("_63__64")
        String _ ("_62__63")
        String _ ("_61__104")
        String _ ("_60__101")
        String _ ("_59__92")
        String _ ("_59__102")
        String _ ("_58__94")
        String _ ("_58__59")
        String _ ("_56__97")
        String _ ("_56__72")
        String _ ("_55__94")
        String _ ("_54__17")
        String _ ("_54__16")
        String _ ("_53__52")
        String _ ("_53__88")
        String _ ("_52__95")
        String _ ("_51__50")
        String _ ("_50__96")
        String _ ("_49__96")
        String _ ("_48__47")
        String _ ("_47__100")
        String _ ("_46__24")
        String _ ("_45__44")
        String _ ("_44__46")
        String _ ("_44__12")
        String _ ("_43__55")
        String _ ("_43__89")
        String _ ("_43__11")
        String _ ("_42__15")
        String _ ("_42__13")
        String _ ("_41__45")
        String _ ("_40__41")
        String _ ("_40__44")
        String _ ("_40__14")
        String _ ("_39__82")
        String _ ("_39__17")
        String _ ("_39__87")
        String _ ("_38__40")
        String _ ("_37__82")
        String _ ("_37__84")
        String _ ("_36__85")
        String _ ("_36__38")
        String _ ("_36__80")
        String _ ("_34__78")
        String _ ("_32__76")
        String _ ("_31__77")
        String _ ("_30__21")
        String _ ("_29__30")
        String _ ("_28__22")
        String _ ("_27__28")
        String _ ("_26__104")
        String _ ("_26__103")
        String _ ("_25__51")
        String _ ("_24__90")
        String _ ("_24__43")
        String _ ("_23__18")
        String _ ("_22__29")
        String _ ("_21__20")
        String _ ("_21__23")
        String _ ("_20__19")
        String _ ("_18__34")
        String _ ("_18__33")
        String _ ("_17__14")
        String _ ("_16__15")
        String _ ("_16__10")
        String _ ("_16__88")
        String _ ("_15__10")
        String _ ("_15__17")
        String _ ("_14__13")
        String _ ("_14__39")
        String _ ("_14__12")
        String _ ("_14__83")
        String _ ("_13__15")
        String _ ("_12__42")
        String _ ("_11__46")
        String _ ("_11__44")
        String _ ("_10__43")
        String _ ("_9__50")
        String _ ("_8__86")
        String _ ("_7__66")
        String _ ("_7__5")
        String _ ("_6__7")
        String _ ("_5__6")
        String _ ("_5__68")
        String _ ("_4__61")
        String _ ("_3__71")
        String _ ("_2__1")
        String _ ("_1__73")
        String _ ("_1__0")
        String _ ("_0__75")
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


    // **************************************************************************************************
    //
    //  TASKS
    //
    // **************************************************************************************************

    Spike add_task_edge1
    Spike add_task_edge2

    add_task_edge1 -> (this) {
        addChildrenTo this.task_edges {
            TaskEdgeModel debug_task_edge1 (find(this.edges, "_22__29"), 0.51)
        }
    }
    add_task_edge2 -> (this) {
        addChildrenTo this.task_edges {
            TaskEdgeModel debug_task_edge2 (find(this.edges, "_29__30"), 0.12)
        }
    }
}