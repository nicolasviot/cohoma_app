use core
use gui
use base

/*_native_code_
%{
    #include "cpp/coords-utils.h"

%}*/

_define_
CohomaContext (Process _frame, double _init_lat, double _init_lon, double _init_zoom)
{
    //frame aka _frame

	//LogPrinter lp ("Context (debug): ")

    Double init_lat (_init_lat)
    Double init_lon (_init_lon)
    Double init_zoom (_init_zoom)

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



    //Ref REF_NULL (0)


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

	//Int selected_trap_id (1)

	Ref entered_wpt (0)

	Ref current_trap (0)
	Ref entered_trap (0)
	
}