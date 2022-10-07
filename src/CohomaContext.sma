use core
use gui
use base

/*_native_code_
%{
    #include "cpp/coords-utils.h"

%}*/

_define_
CohomaContext (double _init_lat, double _init_lon, double _init_zoom)
{
	//LogPrinter lp ("debug in Context: ")

    Double init_lat (_init_lat)
    Double init_lon (_init_lon)
    Double init_zoom (_init_zoom)

    // COLORS FOR VEHICULES -- blues variations
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


    // COLORS FOR SAFETY PILOTS
    Int UAV_COL (#A056F6)
    Int UGV_COL (#708d23)


    Ref RFF_NULL (0)

    //
    // Dynamic properties
    //

    Int selected_id (0)
    //selected_id =:> lp.input

	//Int selected_trap_id (1)

	Ref current_wpt (0)
	Ref entered_wpt (0)
	Ref current_trap (0)
	Ref entered_trap (0)
	
}