use core
use gui
use base

import PointModel

_native_code_
%{
    #include <iostream>
%}


_define_
ExclusionZoneModel (int _type, string _name)
{
	/*
    uint8 TYPE_UNKNOWN    = 0 # Unknown zone type
    uint8 TYPE_RFA        = 1 # Restricted Fire Area (deactivation only on clearance)
    uint8 TYPE_NFA        = 2 # No Fire Area (deactivation forbidden)
    uint8 TYPE_NFZ        = 3 # No Fly Zone
    uint8 TYPE_FFA        = 4 # Free Fire Area (deactivation allowed)
    uint8 TYPE_ROZ_ALL    = 5 # Restricted Operation Zone (forbidden to all vehicles)
    uint8 TYPE_ROZ_GROUND = 6 # Restricted Operation Zone (forbidden to ground vehicles)
    */
	Int n_type (_type)

	String type ("UNKNOWN")

	SwitchList switch_type (0) {
        Component zero {
            "UNKNOWN" =: type
        }
        Component one {
            "RFA" =: type
        }
        Component two {
            "NFA" =: type
        }
        Component three {
            "NFZ" =: type
        }
        Component four {
            "FFA" =:type
        }
        Component five {
            "ROZ_ALL" =: type
        }
		Component six {
            "ROZ_GROUND" =: type
        }
    }
    n_type + 1 =: switch_type.index

	String name (_name)

	List points

    PointModel barycenter (0, 0, 0)
	
	//TextPrinter tp
	//"Model of Exclusion Zone " + type + " (" + n_type + ") '" + name + "'" =: tp.input
}