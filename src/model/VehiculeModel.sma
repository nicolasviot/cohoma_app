use core
use gui
use base

_native_code_
%{
    #include "cpp/coords-utils.h"

%}

_define_
VehiculeModel (Process _context, string _type, string _name, double _lat, double _lon, int _color)
{
    //context aka _context

    String type (_type)
    String name (_name)
    Double lat (_lat)
    Double lon (_lon)
    Int color (_color)

    Double battery_voltage (24)
    Int battery_percentage (75)

    Double altitude_msl (500)
    Double heading_rot (180)

    Bool emergency_stop (0)
    Bool failsafe (0)

    // OPERATING_MODE_UNKNOWN = 0            # Default value
    // OPERATING_MODE_MANUAL = 1             # Operated by security pilot
    // OPERATING_MODE_TELEOPERATION = 2      # Operated by remote operator
    // OPERATING_MODE_AUTONOMOUS = 3    
    Int operation_mode (0)
    String status ("")
    (operation_mode == 1) ? "Manual" : ((operation_mode == 2 ) ? "TeleOP" : ((operation_mode == 3 ) ? "Auto" : "???")) =:> status

}