/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2023)
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

_native_code_
%{
   #include <iostream>
%}


_define_
CohomaContext (Process _frame, double _init_lat, double _init_lon, double _init_zoom)
{
    //frame aka _frame

	LogPrinter lp ("Context (debug): ")
    TextPrinter tp

    // FIXME: RosNode here to give access ?
    //RosNode node (map, context, model_manager)

    Double init_lat (_init_lat)
    Double init_lon (_init_lon)
    Double init_zoom (_init_zoom)

    // scale and translation applied to the map
    Double map_zoom (1.0)
    Double map_scale (1.0)
    Double map_translation_x (0.0)
    Double map_translation_y (0.0)

    Double veil_opacity (0.3)

    //map_zoom / _init_zoom =:> map_scale

    //  Max  Zoom = 19  <-->  Map Scale = 100%
    // "Min" Zoom = 16  <-->  Map Scale = 75%
    // Y = a * X + b
    // a = (1 - 0.75) / (19 - 16)
    // b = 1 - a * 19
    (0.25 / 3.0 * map_zoom) - 0.5833333333 =:> map_scale
    
    //map_zoom + " --> " + map_scale =:> lp.input

    // CONST
    Int RIGHT_PANEL_WIDTH (425)
    Int STRIP_WIDTH (225)
    Int STRIP_HEIGHT (125)

    Int EDGE_WIDTH (4)
    Int ITINERARY_WIDTH (8)
    Int TASK_EDGE_WIDTH (12)

    Int DRAK_GRAY (#323232) // R = G = B = 50
    
    Int EDGE_COLOR (#EAEAEA) // R = G = B = 234

    Int TRAP_COLOR (#F00000) // R = 240 - G = B = 0

    Int TASK_EDGE_COLOR (#DC1414) // R = 220 - G = B = 20    
    
    Int TASK_SELECTION_COLOR (#FFFF00) // R = G = 255 - B = 0
    
    Int UNSELECTED_ITINERARY_COL (#232323) // R = G = B = 35
    Int SELECTED_ITINERARY_COL (#1E90FF) // Blue


    // COLORS for VEHICULES -- blues variations
    //Int VAB_COL (#6BC0FF)
    //Int AGI_1_COL (#ADE2ED)
    //Int AGI_2_COL (#51D5F0)
    //Int LYNX_COL (#5C64FF)
    //Int SPOT_COL (#ABBFEB)
    //Int DRONE_COL (#5EFFF1)
    
    // flashy
    Int VAB_COL (#00B1E6)
    Int AGI_1_COL (#0C2EE8)
    Int AGI_2_COL (#B500FF)
    Int LYNX_COL (#B3B100)
    Int SPOT_COL (#0CE820)
    Int DRONE_COL (#1ACAFF)


    // COLORS for SAFETY PILOTS
    Int UAV_COL (#FFFFFF)
    Int UGV_COL (#FFFFFF)

    // COLORS for node states
    Int NODE_COL (#CCCCCC)
    Int WHITE_COL (#FFFFFF)
    Int BLACK_COL (#000000)
    Int ACTIVE_COL (#29ABE2)
    Int START_COL (#70EE49)
    Int MANDATORY_COL (#FF30FF)


    // CONST: always null
    Ref REF_NULL (nullptr)


    // Keyboard inputs 
    // FIXME: Does not work on some keyboards
    Spike ctrl
    Spike ctrl_r
    _frame.key\-pressed == DJN_Key_Control -> ctrl
    _frame.key\-released == DJN_Key_Control -> ctrl_r

    Spike shift
    Spike shift_r
    _frame.key\-pressed == DJN_Key_Shift -> shift
    _frame.key\-released == DJN_Key_Shift -> shift_r

    //Spike space
    //Spike space_r
    //_frame.key\-pressed == DJN_Key_Space -> space
    //_frame.key\-released == DJN_Key_Space -> space_r

    Spike del
    Spike del_r
    _frame.key\-pressed == DJN_Key_Backspace -> del


    //
    // Dynamic properties
    //

    // Model of the node currently selected during graph edition (left click)
    Ref ref_node_graph_edition (nullptr)
    //Bool is_null_node_graph_edition (1)

    DerefString id_node_graph_edition (ref_node_graph_edition, "id", DJNN_GET_ON_CHANGE)
    DerefDouble dx_node_graph_edition (ref_node_graph_edition, "dx_in_map", DJNN_GET_ON_CHANGE)
    DerefDouble dy_node_graph_edition (ref_node_graph_edition, "dy_in_map", DJNN_GET_ON_CHANGE)
    //id_node_graph_edition.value =:> lp.input

    // 0 = not lazy (updated on first activation)
    AssignmentSequence set_node_graph_edition_to_null (0) {
        REF_NULL =: ref_node_graph_edition
    }


    // Model of the node currently edited in the status menu (right click)
    Ref ref_node_status_edition (nullptr)
    Bool is_null_node_status_edition (1)

    // 0 = not lazy (updated on first activation)
    AssignmentSequence set_node_status_edition_to_null (0) {
        REF_NULL =: ref_node_status_edition
    }

    ref_node_status_edition == REF_NULL ? 1 : 0 =:> is_null_node_status_edition
    // Djnn C++
    // is_null_node_status_edition->set_value(get_property_value (ref_node_status_edition) == get_property_value (REF_NULL) ? 1 : 0, 1);
    //"is NULL current node ? " + is_null_node_status_edition =:> tp.input


    // Trap currently being edited in the status menu (right click)
    Ref ref_current_trap (nullptr)
    Bool is_null_current_trap (1)

    // 0 = not lazy (updated on first activation)
    AssignmentSequence set_current_trap_to_null (0) {
        REF_NULL =: ref_current_trap
    }

    ref_current_trap == REF_NULL ? 1 : 0 =:> is_null_current_trap
    // Djnn C++
    //is_null_current_trap->set_value(get_property_value (ref_current_trap) == get_property_value (REF_NULL) ? 1 : 0, 1);
    //"is NULL current trap ? " + is_null_current_trap =:> tp.input
	

    // Don't use empty string
    String selected_itinerary_id ("-1")
    //selected_itinerary_id =:> lp.input

    //String selected_itinerary_type ("")

}