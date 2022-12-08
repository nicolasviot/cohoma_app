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

    Double init_lat (_init_lat)
    Double init_lon (_init_lon)
    Double init_zoom (_init_zoom)

    // scale and translation applied to the map
    Double map_scale (1.0)
    Double map_translation_x (0.0)
    Double map_translation_y (0.0)

    // CONST
    Int RIGHT_PANEL_WIDTH (425)
    Int STRIP_WIDTH (225)
    Int STRIP_HEIGHT (125)

    Int DRAK_GRAY (#323232) // R = G = B = 50

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
    Int UAV_COL (#A056F6)
    Int UGV_COL (#708d23)

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

    Int selected_node_id (0)
    //selected_node_id =:> lp.input


    // Node currently being edited in the status menu
    Bool is_null_current_node (1)
    Ref ref_current_node (nullptr)

    // 0 = not lazy (updated on first activation)
    AssignmentSequence set_current_node_to_null (0) {
        REF_NULL =: ref_current_node
    }

    ref_current_node == REF_NULL ? 1 : 0 =:> is_null_current_node
    // Djnn C++
    // is_null_current_node->set_value(get_property_value (ref_current_node) == get_property_value (REF_NULL) ? 1 : 0, 1);
    //"is NULL current node ? " + is_null_current_node =:> tp.input


    // Trap currently being edited in the status menu
    Bool is_null_current_trap (1)
    Ref ref_current_trap (nullptr)

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
    selected_itinerary_id =:> lp.input

	Ref ref_selected_itinerary (nullptr)

    /*// 0 = not lazy (updated on first activation)
    AssignmentSequence set_selected_itinerary_to_null (0) {
        REF_NULL =: ref_selected_itinerary
    }*/


    //auto-preview : request plan when a node becomes forced
    Spike plan_request
}