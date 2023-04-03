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

_action_
action_unselect_trap (Process src, Process self)
{
    if (!self.is_null_selected_trap) {
        previous_selected_trap = getRef (self.ref_selected_trap)
        previous_selected_trap.is_selected = false

        //setRef (self.ref_selected_trap, self.REF_NULL)
    }
}


_define_
CohomaContext (Process _frame, double _init_lat, double _init_lon, double _init_zoom)
{
    //frame aka _frame
    frame_released aka _frame.release

	LogPrinter lp ("Context (debug): ")
    TextPrinter tp

    Double init_lat (_init_lat)
    Double init_lon (_init_lon)
    Double init_zoom (_init_zoom)

    // scale and translation applied to the map
    Double map_zoom (1.0)
    Double map_scale (1.0)
    Double map_translation_x (0.0)
    Double map_translation_y (0.0)

    Double veil_opacity (0.2) // 20%

    //map_zoom / _init_zoom =:> map_scale

    //  Max  Zoom = 19  <-->  Map Scale = 100%
    // "Min" Zoom = 16  <-->  Map Scale = 75%
    // Y = a * X + b
    // a = (1 - 0.75) / (19 - 16)
    // b = 1 - a * 19
    (0.25 / 3.0 * map_zoom) - 0.5833333333 =:> map_scale
    
    //map_zoom + " --> " + map_scale =:> lp.input


    // CONST: always null
    Ref REF_NULL (nullptr)

    // **************************************************************************************************
    //
    //  SIZE
    //
    // **************************************************************************************************

    Int TOP_BAR_HEIGHT (26)
    Int LEFT_PANEL_WIDTH (250)
    Int RIGHT_PANEL_WIDTH (250)

    Int OPERATOR_HEADER_HEIGHT (25)
    Int VEHICLE_STRIP_HEIGHT (25)

    Int EDGE_WIDTH (4)
    Int ITINERARY_WIDTH (8)
    Int TASK_EDGE_WIDTH (12)

    Int TRAP_SIZE (30) // 25
    Int TRAP_DETECTION_SIZE (14)


    // **************************************************************************************************
    //
    //  ENUMS
    //
    // **************************************************************************************************

    // Type of operator
    Int OPERATOR_TYPE_TACTICAL (1)
    Int OPERATOR_TYPE_ROBOT (2)
    Int OPERATOR_TYPE_SAFETY (3)

    // Type of vehicle / satellite
    Int VEHICLE_TYPE_VAB (1)
    Int VEHICLE_TYPE_UGV (2)
    Int VEHICLE_TYPE_UAV (3)

    // **************************************************************************************************
    //
    //  COLORS
    //
    // **************************************************************************************************

    Int DARK_GRAY (#3C3C3B)
    
    Int EDGE_COLOR (#EAEAEA) // R = G = B = 234

    Int TRAP_COLOR (#F00000) // R = 240 - G = B = 0
    Int TRAP_DETECTION_COLOR (#FF5200) // R = 255 - G = 82 - B = 0

    Int TASK_EDGE_COLOR (#DC1414) // R = 220 - G = B = 20    
    
    Int TASK_SELECTION_COLOR (#FFFF00)  // R = G = 255 - B = 0
    Int SELECTION_COLOR (#FFFF00)       // Yellow (R = G = 255 - B = 0)
    
    Int UNSELECTED_ITINERARY_COL (#232323) // R = G = B = 35
    Int SELECTED_ITINERARY_COL (#1E90FF) // Blue


    // Colors for operators
    Int OT_COLOR (#4027F5)  // Op. TACTICAL
    Int OG1_COLOR (#317CE8) // Op. Group 1
    Int OG2_COLOR (#29B9FF) // Op. Group 2
    Int OG3_COLOR (#27E8F5) // Op. Group 3
    Int OS_COLOR (#5CAD9D)   // Op. Safety

    //Colors for status strips
    Int OK_COLOR(#314D17) //dark green for OK.
    Int NOK_COLOR(#FF3333) //flashy red for problem

    Int SAFETY_PILOT_COL(#5CAD9D)

    // COLORS for node states
    Int NODE_COL (#CCCCCC)
    Int WHITE_COL (#FFFFFF)
    Int BLACK_COL (#000000)
    Int ACTIVE_COL (#29ABE2)
    Int START_COL (#70EE49)
    Int MANDATORY_COL (#FF30FF)


    // **************************************************************************************************
    //
    //  Keyboard inputs 
    //
    // **************************************************************************************************

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

    
    // **************************************************************************************************
    //
    //  Dynamic properties
    //
    // **************************************************************************************************
    
    // Allow to display the current time
    WallClock w_clock
    "%H:%M:%S" =: w_clock.format // exemple = "%H:%M:%S", "%Hh%Mm%Ss"

    // To update it (FIXME: 900ms is sufficient ?)
    Clock clock_trigger (900)
    clock_trigger.tick -> w_clock.state_text

    // Lat/Long of cursor
    Double pointer_lat (0)
    Double pointer_lon (0)

    // Spikes to notify when a map item is currently dragged
    Spike start_drag_map_item
    Spike stop_drag_map_item


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


    // Trap currently selected
    Ref ref_selected_trap (nullptr)
    Bool is_null_selected_trap (1)
    Spike spike_to_set_selected_trap_to_null
    //is_null_selected_trap ? "No selected trap" : "Trap is selected" =:> lp.input

    // 0 = not lazy (updated on first activation)
    AssignmentSequence set_selected_trap_to_null (0) {
        REF_NULL =: ref_selected_trap
    }
    spike_to_set_selected_trap_to_null -> set_selected_trap_to_null

    NativeAction na_unselect_trap (action_unselect_trap, this, 1)
    na_unselect_trap -> set_selected_trap_to_null

    ref_selected_trap == REF_NULL ? 1 : 0 =:> is_null_selected_trap


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

    
    //DRAG and DROP strip allocation
    Bool show_drop_zones_strip (1)
    Ref dragged_strip (null)
    Spike show_confirmation_form
    Ref dropped_operator (null)

    //ROS TESTS
    Spike ros_test
}