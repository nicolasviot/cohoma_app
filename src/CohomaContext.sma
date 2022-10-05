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

    Double init_lat (_init_lat)
    Double init_lon (_init_lon)
    Double init_zoom (_init_zoom)

    // COLORS FOR VEHICULES -- blues variations
    //Int VAB_COL (#6BC0FF)
    //Int agi1_COL (#ADE2ED)
    //Int agi2_COL2 (#51D5F0)
    //Int lynxCOL (#5C64FF)
    //Int spotCOL (#ABBFEB)
    //Int droneCOL (#5EFFF1)
    
    // flashy
    Int VAB_COL (#00B1E6)
    Int AGI_1_COL (#0C2EE8)
    Int AGI_2_COL (#B500FF)
    Int LYNX_COL (#B3B100)
    Int SPOT_COL (#0CE820)
    Int DRONE_COL (#1ACAFF)
}