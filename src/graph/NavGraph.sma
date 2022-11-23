use core
use gui
use base

import Node
import Edge

_native_code_
%{
    #include <iostream>
%}


_define_
NavGraph (Process _map, Process _context, Process _model_manager)
{
	map aka _map
	context aka _context
	model_manager aka _model_manager

	Int id (0)

	//Spike create_bindings
	//Spike clear

	Scaling sc (1, 1, 0, 0)
	_context.map_scale =:> sc.sx, sc.sy

	Translation pos (0, 0)
	_context.map_translation_x =:> pos.tx
	_context.map_translation_y =:> pos.ty

	List edges {
		
	}

	List nodes {

	}

	_model_manager.nodes.$added -> na_node_added:(this) {
		print ("New model of node added to list " + this.model_manager.nodes.size + "\n")
		/*model = getRef (&this.model_manager.nodes.$added)
    	addChildrenTo this.nodes {
			Node node (this.map, this.context, model)
		}*/
	}

	_model_manager.nodes.$removed -> na_node_removed:(this) {
		print ("Model of node removed from list " + this.model_manager.nodes.size + "\n")
		//model = getRef (&this.model_manager.nodes.$removed)
	}

	for model : _model_manager.nodes {
		addChildrenTo this.nodes {
			Node node (this.map, this.context, model)
		}
	}

	for model : _model_manager.edges {
		addChildrenTo this.edges {
			Edge node (this.context, model)
		}
	}

	//if (model_manager.IS_DEBUG)
	//{
	//	addChildrenTo nodes {
			
			//Esperces data
			/*Node node0(_map, _context, 43.316021818382886, 1.4041900634765625, 0.0, 0, "n_011", 1)
			Node node1(_map, _context, 43.316006206187375, 1.4047694206237793, 0.0, 0, "n_010", 2)
			Node node2(_map, _context, 43.3159281451497, 1.4054131507873535, 0.0, 0, "n_009", 3) //end
			Node node3(_map, _context, 43.31569396143501, 1.4050912857055664, 0.0, 0, "n_008", 4)
			Node node4(_map, _context, 43.31580324728085, 1.4047479629516602, 0.0, 0, "n_007", 5)
			Node node5(_map, _context, 43.31575641051384, 1.4043188095092773, 0.0, 0, "n_006", 6)
			Node node6(_map, _context, 43.31556906308483, 1.404104232788086, 0.0, 0, "n_005", 7) // start
			Node node7(_map, _context, 43.31543635830649, 1.4042115211486816, 0.0, 0, "n_004", 8)
			Node node8(_map, _context, 43.315420745960566, 1.404426097869873, 0.0, 0, "n_003", 9)
			Node node9(_map, _context, 43.31560028769644, 1.4044904708862305, 0.0, 0, "n_002", 10)
			Node node10(_map, _context, 43.3155144199759, 1.4047908782958984, 0.0, 0, "n_001", 11)
			Node node11(_map, _context, 43.315327071800844, 1.4045333862304688, 0.0, 0, "n_000", 12)*/

			//Baynes data
			/*Node node1 (_map, _context, 48.866366, 1.885056, 0.0, 0, "FIRST", 1)
			Node node2 (_map, _context, 48.866246, 1.885335, 0.0, 0, "", 2)
			Node node3 (_map, _context, 48.866733, 1.886022, 0.0, 0, "", 3)
			Node node4 (_map, _context, 48.865385, 1.886623, 0.0, 0, "", 4)
			Node node5 (_map, _context, 48.864482, 1.884327, 0.0, 0, "", 5)
			Node node6 (_map, _context, 48.865308, 1.885357, 0.0, 0, "", 6)
			Node node7 (_map, _context, 48.865294, 1.885099, 0.0, 0, "", 7)
			Node node8 (_map, _context, 48.865421, 1.885035, 0.0, 0, "", 8)
			Node node9 (_map, _context, 48.863127, 1.887159, 0.0, 0, "", 9)
			Node node10 (_map, _context, 48.864644, 1.886601, 0.0, 0, "", 10)
			Node node11 (_map, _context, 48.862964, 1.888790, 0.0, 0, "", 11)
			Node node12 (_map, _context, 48.862774, 1.889369, 0.0, 0, "", 12)
			Node node13 (_map, _context, 48.862633, 1.888962, 0.0, 0, "", 13)
			Node node14 (_map, _context, 48.862626, 1.888361, 0.0, 0, "", 14)
			Node node15 (_map, _context, 48.862527, 1.888146, 0.0, 0, "", 15)
			Node node16 (_map, _context, 48.862823, 1.888425, 0.0, 0, "", 16)
			Node node17 (_map, _context, 48.862964, 1.888382, 0.0, 0, "", 17)
			Node node18 (_map, _context, 48.862668, 1.888103, 0.0, 0, "", 18)
			Node node19 (_map, _context, 48.860833, 1.886945, 0.0, 0, "", 19)
			Node node20 (_map, _context, 48.859703, 1.887095, 0.0, 0, "", 20)
			Node node21 (_map, _context, 48.860268, 1.886988, 0.0, 0, "", 21)
			Node node22 (_map, _context, 48.860303, 1.887224, 0.0, 0, "", 22)
			Node node23 (_map, _context, 48.861023, 1.892030, 0.0, 0, "", 23)
			Node node24 (_map, _context, 48.860684, 1.887024, 0.0, 1, "PPO10", 24)
			Node node25 (_map, _context, 48.863247, 1.889708, 0.0, 1, "PPO20", 25)
			Node node26 (_map, _context, 48.864712, 1.886771, 0.0, 1, "PPO30", 26)
			Node node27 (_map, _context, 48.866231, 1.883874, 0.0, 1, "PPO40", 27)
			Node node28 (_map, _context, 48.861095, 1.893314, 0.0, 1, "depart", 28)
			Node node29 (_map, _context, 48.861026, 1.892661, 0.0, 0, "T101", 29)
			Node node30 (_map, _context, 48.860770, 1.890403, 0.0, 0, "T102", 30)
			Node node31 (_map, _context, 48.860347, 1.887549, 0.0, 0, "T103", 31)
			Node node32 (_map, _context, 48.861903, 1.884679, 0.0, 0, "T201", 32)
			Node node33 (_map, _context, 48.862245, 1.885626, 0.0, 0, "T202", 33)
			Node node34 (_map, _context, 48.861444, 1.886598, 0.0, 0, "T203", 34)
			Node node35 (_map, _context, 48.860894, 1.887346, 0.0, 0, "T204", 35)
			Node node36 (_map, _context, 48.862156, 1.886664, 0.0, 0, "T205", 36)
			Node node37 (_map, _context, 48.861729, 1.887151, 0.0, 0, "T206", 37)
			Node node38 (_map, _context, 48.862424, 1.887408, 0.0, 0, "T207", 38)
			Node node39 (_map, _context, 48.861977, 1.887691, 0.0, 0, "T208", 39)
			Node node40 (_map, _context, 48.862518, 1.887883, 0.0, 0, "T209", 40)
			Node node41 (_map, _context, 48.862289, 1.888325, 0.0, 0, "T210", 41)
			Node node42 (_map, _context, 48.862163, 1.889254, 0.0, 0, "T211", 42)
			Node node43 (_map, _context, 48.862725, 1.888833, 0.0, 0, "T212", 43)
			Node node44 (_map, _context, 48.863043, 1.889099, 0.0, 0, "T213", 44)
			Node node45 (_map, _context, 48.862678, 1.889461, 0.0, 0, "T214", 45)
			Node node46 (_map, _context, 48.862383, 1.889740, 0.0, 0, "T215", 46)
			Node node47 (_map, _context, 48.862763, 1.889964, 0.0, 0, "T216", 47)
			Node node48 (_map, _context, 48.863058, 1.885090, 0.0, 0, "T301", 48)
			Node node49 (_map, _context, 48.863046, 1.885690, 0.0, 0, "T302", 49)
			Node node50 (_map, _context, 48.863652, 1.885990, 0.0, 0, "T303", 50)
			Node node51 (_map, _context, 48.864453, 1.886981, 0.0, 0, "T304", 51)
			Node node52 (_map, _context, 48.864295, 1.887435, 0.0, 0, "T305", 52)
			Node node53 (_map, _context, 48.863883, 1.887607, 0.0, 0, "T306", 53)
			Node node54 (_map, _context, 48.863599, 1.888023, 0.0, 0, "T307", 54)
			Node node55 (_map, _context, 48.862863, 1.888135, 0.0, 0, "T308", 55)
			Node node56 (_map, _context, 48.863924, 1.888111, 0.0, 0, "T309", 56)
			Node node57 (_map, _context, 48.864835, 1.887409, 0.0, 0, "T310", 57)
			Node node58 (_map, _context, 48.864414, 1.888591, 0.0, 0, "T311", 58)
			Node node59 (_map, _context, 48.863518, 1.888965, 0.0, 0, "T312", 59)
			Node node60 (_map, _context, 48.863836, 1.889217, 0.0, 0, "T313", 60)
			Node node61 (_map, _context, 48.863975, 1.889637, 0.0, 0, "T314", 61)
			Node node62 (_map, _context, 48.865145, 1.884143, 0.0, 0, "T401", 62)
			Node node63 (_map, _context, 48.864363, 1.885115, 0.0, 0, "T402", 63)
			Node node64 (_map, _context, 48.865267, 1.884632, 0.0, 0, "T403", 64)
			Node node65 (_map, _context, 48.865840, 1.884346, 0.0, 0, "T404", 65)
			Node node66 (_map, _context, 48.866336, 1.884540, 0.0, 0, "T405", 66)
			Node node67 (_map, _context, 48.865674, 1.884868, 0.0, 0, "T406", 67)
			Node node68 (_map, _context, 48.865029, 1.885209, 0.0, 0, "T407", 68)
			Node node69 (_map, _context, 48.865087, 1.885590, 0.0, 0, "T408", 69)
			Node node70 (_map, _context, 48.865330, 1.885557, 0.0, 0, "T409", 70)
			Node node71 (_map, _context, 48.865020, 1.886082, 0.0, 0, "T410", 71)
			Node node72 (_map, _context, 48.865494, 1.886767, 0.0, 0, "T411", 72)
			Node node73 (_map, _context, 48.865489, 1.886222, 0.0, 0, "T412", 73)
			Node node74 (_map, _context, 48.866127, 1.885171, 0.0, 0, "T413", 74)
			Node node75 (_map, _context, 48.866529, 1.884958, 0.0, 0, "T414", 75)
			Node node76 (_map, _context, 48.866835, 1.885851, 0.0, 0, "T415", 76)
			Node node77 (_map, _context, 48.862350, 1.885979, 0.0, 0, "", 77)
			Node node78 (_map, _context, 48.862139, 1.885250, 0.0, 0, "", 78)
			Node node79 (_map, _context, 48.861045, 1.887181, 0.0, 0, "", 79)
			Node node80 (_map, _context, 48.861440, 1.887267, 0.0, 0, "", 80)
			Node node81 (_map, _context, 48.861616, 1.887374, 0.0, 0, "", 81)
			Node node82 (_map, _context, 48.861334, 1.887074, 0.0, 0, "", 82)
			Node node83 (_map, _context, 48.862484, 1.887653, 0.0, 0, "", 83)
			Node node84 (_map, _context, 48.862393, 1.888039, 0.0, 0, "", 84)
			Node node85 (_map, _context, 48.862407, 1.887181, 0.0, 0, "", 85)
			Node node86 (_map, _context, 48.862068, 1.887310, 0.0, 0, "", 86)
			Node node87 (_map, _context, 48.862689, 1.887417, 0.0, 0, "", 87)
			Node node88 (_map, _context, 48.862746, 1.887889, 0.0, 0, "", 88)
			Node node89 (_map, _context, 48.863183, 1.888232, 0.0, 0, "", 89)
			Node node90 (_map, _context, 48.863395, 1.889133, 0.0, 0, "", 90)
			Node node91 (_map, _context, 48.863564, 1.889584, 0.0, 0, "", 91)
			Node node92 (_map, _context, 48.863642, 1.890078, 0.0, 0, "", 92)
			Node node93 (_map, _context, 48.863811, 1.889412, 0.0, 0, "", 93)
			Node node94 (_map, _context, 48.864143, 1.889112, 0.0, 0, "", 94)
			Node node95 (_map, _context, 48.863931, 1.888533, 0.0, 0, "", 95)
			Node node96 (_map, _context, 48.864065, 1.887782, 0.0, 0, "", 96)
			Node node97 (_map, _context, 48.864284, 1.886773, 0.0, 0, "", 97)
			Node node98 (_map, _context, 48.864955, 1.887631, 0.0, 0, "", 98)
			Node node99 (_map, _context, 48.864228, 1.885185, 0.0, 0, "", 99)
			Node node100 (_map, _context, 48.864334, 1.885550, 0.0, 0, "", 100)
			Node node101 (_map, _context, 48.864108, 1.884799, 0.0, 0, "", 101)
			Node node102 (_map, _context, 48.865025, 1.887932, 0.0, 0, "", 102)
			Node node103 (_map, _context, 48.864305, 1.888425, 0.0, 0, "", 103)
			Node node104 (_map, _context, 48.865943, 1.884563, 0.0, 0, "", 104)
			Node node105 (_map, _context, 48.865329, 1.884091, 0.0, 0, "", 105)
			Node node106 (_map, _context, 48.866211, 1.884520, 0.0, 0, "", 106)
			Node node107 (_map, _context, 48.866119, 1.884906, 0.0, 0, "", 107)
			
		}*/

		//addChildrenTo edges {

			//Esperces data
			/*Edge edge1(12, 11, 22.11618714809018, nodes)
			Edge edge2(12, 10, 22.11618714809018, nodes)
			Edge edge3(12, 9, 22.11618714809018, nodes)
			Edge edge4(11, 10, 22.11618714809018, nodes)
			Edge edge5(11, 4, 22.11618714809018, nodes)
			Edge edge6(10, 9, 22.11618714809018, nodes)
			Edge edge7(10, 6, 22.11618714809018, nodes)
			Edge edge8(10, 5, 22.11618714809018, nodes)
			Edge edge9(9, 8, 22.11618714809018, nodes)
			Edge edge10(8, 7, 22.11618714809018, nodes)
			Edge edge11(7, 6, 22.11618714809018, nodes)
			Edge edge12(6, 5, 22.11618714809018, nodes)
			Edge edge13(6, 1, 22.11618714809018, nodes)
			Edge edge14(5, 4, 22.11618714809018, nodes)
			Edge edge15(5, 2, 22.11618714809018, nodes)
			Edge edge16(4, 3, 22.11618714809018, nodes)
			Edge edge17(3, 2, 22.11618714809018, nodes)
			Edge edge18(2, 1, 22.11618714809018, nodes)*/
		
			//Baynes data
			/*Edge edge_107_66 (107, 66, 36.151847, nodes)
			Edge edge_107_1 (107, 1, 29.598092, nodes)
			Edge edge_107_74 (107, 74, 19.472073, nodes)
			Edge edge_106_27 (106, 27, 47.470229, nodes)
			Edge edge_106_65 (106, 65, 43.217995, nodes)
			Edge edge_106_75 (106, 75, 47.807154, nodes)
			Edge edge_106_107 (106, 107, 30.118301, nodes)
			Edge edge_105_64 (105, 64, 40.259731, nodes)
			Edge edge_104_107 (104, 107, 31.929357, nodes)
			Edge edge_104_67 (104, 67, 37.352638, nodes)
			Edge edge_103_57 (103, 57, 95.000664, nodes)
			Edge edge_101_5 (101, 5, 54.130695, nodes)
			Edge edge_100_50 (100, 50, 82.433525, nodes)
			Edge edge_100_68 (100, 68, 81.311173, nodes)
			Edge edge_99_49 (99, 49, 136.542556, nodes)
			Edge edge_99_63 (99, 63, 15.872494, nodes)
			Edge edge_98_58 (98, 58, 92.566236, nodes)
			Edge edge_98_4 (98, 4, 88.132221, nodes)
			Edge edge_97_53 (97, 53, 75.745353, nodes)
			Edge edge_96_56 (96, 56, 28.840949, nodes)
			Edge edge_96_52 (96, 52, 36.070685, nodes)
			Edge edge_95_60 (95, 60, 51.359895, nodes)
			Edge edge_94_58 (94, 58, 48.693536, nodes)
			Edge edge_93_94 (93, 94, 42.973098, nodes)
			Edge edge_93_91 (93, 91, 30.221114, nodes)
			Edge edge_92_61 (92, 61, 49.126481, nodes)
			Edge edge_92_93 (92, 93, 52.314852, nodes)
			Edge edge_91_60 (91, 60, 40.423100, nodes)
			Edge edge_91_92 (91, 92, 37.225806, nodes)
			Edge edge_90_59 (90, 59, 18.451257, nodes)
			Edge edge_90_25 (90, 25, 45.246885, nodes)
			Edge edge_89_55 (89, 55, 36.360323, nodes)
			Edge edge_88_55 (88, 55, 22.233493, nodes)
			Edge edge_87_85 (87, 85, 35.856414, nodes)
			Edge edge_87_83 (87, 83, 28.601847, nodes)
			Edge edge_86_38 (86, 38, 40.223629, nodes)
			Edge edge_85_36 (85, 36, 47.085686, nodes)
			Edge edge_84_41 (84, 41, 23.920976, nodes)
			Edge edge_84_40 (84, 40, 18.065095, nodes)
			Edge edge_83_39 (83, 39, 56.526627, nodes)
			Edge edge_83_84 (83, 84, 30.120536, nodes)
			Edge edge_82_37 (82, 37, 44.257500, nodes)
			Edge edge_81_80 (81, 80, 21.143470, nodes)
			Edge edge_80_79 (80, 79, 44.405337, nodes)
			Edge edge_79_82 (79, 82, 33.131297, nodes)
			Edge edge_78_33 (78, 33, 30.020075, nodes)
			Edge edge_78_48 (78, 48, 102.932196, nodes)
			Edge edge_77_49 (77, 49, 80.217509, nodes)
			Edge edge_77_34 (77, 34, 110.502852, nodes)
			Edge edge_76_3 (76, 3, 16.877866, nodes)
			Edge edge_75_107 (75, 107, 45.761595, nodes)
			Edge edge_75_1 (75, 1, 19.521553, nodes)
			Edge edge_73_4 (73, 4, 31.601004, nodes)
			Edge edge_73_74 (73, 74, 104.723316, nodes)
			Edge edge_72_102 (72, 102, 100.103569, nodes)
			Edge edge_71_10 (71, 10, 56.533485, nodes)
			Edge edge_71_26 (71, 26, 61.052196, nodes)
			Edge edge_71_70 (71, 70, 51.678798, nodes)
			Edge edge_70_67 (70, 67, 63.431519, nodes)
			Edge edge_70_6 (70, 6, 14.907842, nodes)
			Edge edge_69_71 (69, 71, 36.889908, nodes)
			Edge edge_68_69 (68, 69, 28.644749, nodes)
			Edge edge_68_7 (68, 7, 30.456854, nodes)
			Edge edge_66_106 (66, 106, 14.015195, nodes)
			Edge edge_65_27 (65, 27, 55.623428, nodes)
			Edge edge_65_104 (65, 104, 19.613978, nodes)
			Edge edge_64_65 (64, 65, 67.073533, nodes)
			Edge edge_63_64 (63, 64, 106.603108, nodes)
			Edge edge_62_105 (62, 105, 20.791244, nodes)
			Edge edge_61_102 (61, 102, 171.188617, nodes)
			Edge edge_60_93 (60, 93, 14.556726, nodes)
			Edge edge_60_103 (60, 103, 78.144713, nodes)
			Edge edge_59_95 (59, 95, 55.833599, nodes)
			Edge edge_59_60 (59, 60, 39.837302, nodes)
			Edge edge_57_98 (57, 98, 21.071659, nodes)
			Edge edge_57_73 (57, 73, 113.490941, nodes)
			Edge edge_56_95 (56, 95, 30.943167, nodes)
			Edge edge_55_18 (55, 18, 21.771071, nodes)
			Edge edge_55_17 (55, 17, 21.419409, nodes)
			Edge edge_54_53 (54, 53, 43.877389, nodes)
			Edge edge_54_89 (54, 89, 48.752808, nodes)
			Edge edge_53_96 (53, 96, 23.967389, nodes)
			Edge edge_52_51 (52, 51, 37.601856, nodes)
			Edge edge_51_97 (51, 97, 24.179112, nodes)
			Edge edge_50_97 (50, 97, 90.835744, nodes)
			Edge edge_49_48 (49, 48, 44.059438, nodes)
			Edge edge_48_101 (48, 101, 118.660350, nodes)
			Edge edge_47_25 (47, 25, 56.939136, nodes)
			Edge edge_46_45 (46, 45, 38.599858, nodes)
			Edge edge_45_47 (45, 47, 38.090999, nodes)
			Edge edge_45_13 (45, 13, 36.984206, nodes)
			Edge edge_44_56 (44, 56, 121.866547, nodes)
			Edge edge_44_90 (44, 90, 39.235906, nodes)
			Edge edge_44_12 (44, 12, 35.917267, nodes)
			Edge edge_43_16 (43, 16, 31.824615, nodes)
			Edge edge_43_14 (43, 14, 36.368447, nodes)
			Edge edge_42_46 (42, 46, 43.280400, nodes)
			Edge edge_41_42 (41, 42, 69.656940, nodes)
			Edge edge_41_45 (41, 45, 93.944769, nodes)
			Edge edge_41_15 (41, 15, 29.524378, nodes)
			Edge edge_40_83 (40, 83, 17.310842, nodes)
			Edge edge_40_18 (40, 18, 23.197280, nodes)
			Edge edge_40_88 (40, 88, 25.272147, nodes)
			Edge edge_39_41 (39, 41, 58.027089, nodes)
			Edge edge_38_83 (38, 83, 19.189421, nodes)
			Edge edge_38_85 (38, 85, 16.777792, nodes)
			Edge edge_37_86 (37, 86, 39.497586, nodes)
			Edge edge_37_39 (37, 39, 48.275075, nodes)
			Edge edge_37_81 (37, 81, 20.599842, nodes)
			Edge edge_35_79 (35, 79, 20.699424, nodes)
			Edge edge_33_77 (33, 77, 28.463999, nodes)
			Edge edge_32_78 (32, 78, 49.408749, nodes)
			Edge edge_31_22 (31, 22, 24.391799, nodes)
			Edge edge_30_31 (30, 31, 214.628357, nodes)
			Edge edge_29_23 (29, 23, 46.280472, nodes)
			Edge edge_28_29 (28, 29, 48.523211, nodes)
			Edge edge_27_105 (27, 105, 101.586218, nodes)
			Edge edge_27_104 (27, 104, 59.865948, nodes)
			Edge edge_26_52 (26, 52, 67.200246, nodes)
			Edge edge_25_91 (25, 91, 36.474892, nodes)
			Edge edge_25_44 (25, 44, 50.105878, nodes)
			Edge edge_24_19 (24, 19, 17.550023, nodes)
			Edge edge_23_30 (23, 30, 122.646170, nodes)
			Edge edge_22_21 (22, 21, 17.758447, nodes)
			Edge edge_22_24 (22, 24, 44.777808, nodes)
			Edge edge_21_20 (21, 20, 63.288064, nodes)
			Edge edge_19_35 (19, 35, 30.248770, nodes)
			Edge edge_19_34 (19, 34, 72.620469, nodes)
			Edge edge_18_15 (18, 15, 16.011005, nodes)
			Edge edge_17_16 (17, 16, 16.010911, nodes)
			Edge edge_17_11 (17, 11, 29.913585, nodes)
			Edge edge_17_89 (17, 89, 26.711638, nodes)
			Edge edge_16_11 (16, 11, 31.028860, nodes)
			Edge edge_16_18 (16, 18, 29.255889, nodes)
			Edge edge_15_14 (15, 14, 19.199778, nodes)
			Edge edge_15_40 (15, 40, 19.339581, nodes)
			Edge edge_15_13 (15, 13, 60.975128, nodes)
			Edge edge_15_84 (15, 84, 16.863581, nodes)
			Edge edge_14_16 (14, 16, 22.479398, nodes)
			Edge edge_13_43 (13, 43, 13.992287, nodes)
			Edge edge_12_47 (12, 47, 43.624491, nodes)
			Edge edge_12_45 (12, 45, 12.650085, nodes)
			Edge edge_11_44 (11, 44, 24.260737, nodes)
			Edge edge_10_51 (10, 51, 35.062912, nodes)
			Edge edge_9_87 (9, 87, 52.203169, nodes)
			Edge edge_8_67 (8, 67, 30.722089, nodes)
			Edge edge_8_6 (8, 6, 26.746292, nodes)
			Edge edge_7_8 (7, 8, 14.896268, nodes)
			Edge edge_6_7 (6, 7, 18.957015, nodes)
			Edge edge_6_69 (6, 69, 29.898480, nodes)
			Edge edge_5_62 (5, 62, 74.973459, nodes)
			Edge edge_4_72 (4, 72, 16.098275, nodes)
			Edge edge_3_2 (3, 2, 73.963725, nodes)
			Edge edge_2_74 (2, 74, 17.943674, nodes)
			Edge edge_2_1 (2, 1, 24.431021, nodes)
			Edge edge_1_76 (1, 76, 78.194969, nodes)

		}*/
	//}
	
	OutlineOpacity _(0.5)
	
	List shadow_edges{
		
	}

}
