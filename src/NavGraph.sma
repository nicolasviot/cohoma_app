use core
use gui
use base

import Node
import Edge
import ManagerId
//TODO test lemon native HERE





_define_
NavGraph (Process _map){
	map aka _map
	Spike create_bindings
	Spike clear
	Int id (0)
	ManagerId manager(0)

	LogPrinter lp("id_selected")
	manager.selected_id =:> lp.input

	List nodes {
		Node node0(map, 43.316021818382886, 1.4041900634765625, 0.0, 0, "n_011", 0, manager)
		Node node1(map, 43.316006206187375, 1.4047694206237793, 0.0, 0, "n_010", 1, manager)
		Node node2(map, 43.3159281451497, 1.4054131507873535, 0.0, 0, "n_009", 2, manager)
		Node node3(map, 43.31569396143501, 1.4050912857055664, 0.0, 0, "n_008", 3, manager)
		Node node4(map, 43.31580324728085, 1.4047479629516602, 0.0, 0, "n_007", 4, manager)
		Node node5(map, 43.31575641051384, 1.4043188095092773, 0.0, 0, "n_006", 5, manager)
		Node node6(map, 43.31556906308483, 1.404104232788086, 0.0, 0, "n_005", 6, manager)
		Node node7(map, 43.31543635830649, 1.4042115211486816, 0.0, 0, "n_004", 7, manager)
		Node node8(map, 43.315420745960566, 1.404426097869873, 0.0, 0, "n_003", 8, manager)
		Node node9(map, 43.31560028769644, 1.4044904708862305, 0.0, 0, "n_002", 9, manager)
		Node node10(map, 43.3155144199759, 1.4047908782958984, 0.0, 0, "n_001", 10, manager)
		Node node11(map, 43.315327071800844, 1.4045333862304688, 0.0, 0, "n_000", 11, manager)	
	}
	
	List edges {
		Edge edge1(11, 10, 22.11618714809018, nodes)
		Edge edge2(11, 9, 22.11618714809018, nodes)
		Edge edge3(11, 8, 22.11618714809018, nodes)
		Edge edge4(10, 9, 22.11618714809018, nodes)
		Edge edge5(10, 3, 22.11618714809018, nodes)
		Edge edge6(9, 8, 22.11618714809018, nodes)
		Edge edge7(9, 5, 22.11618714809018, nodes)
		Edge edge8(9, 4, 22.11618714809018, nodes)
		Edge edge9(8, 7, 22.11618714809018, nodes)
		Edge edge10(7, 6, 22.11618714809018, nodes)
		Edge edge11(6, 5, 22.11618714809018, nodes)
		Edge edge12(5, 4, 22.11618714809018, nodes)
		Edge edge13(5, 0, 22.11618714809018, nodes)
		Edge edge14(4, 3, 22.11618714809018, nodes)
		Edge edge15(4, 1, 22.11618714809018, nodes)
		Edge edge16(3, 2, 22.11618714809018, nodes)
		Edge edge17(2, 1, 22.11618714809018, nodes)
		Edge edge18(1, 0, 22.11618714809018, nodes)
	}

	




}
