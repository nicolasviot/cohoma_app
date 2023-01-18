use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
VehicleModel (Process _context, string _type, string _name, double _lat, double _lon, int _color)
{
    //context aka _context

    // ROBOT_ID_UNKNOWN = 0 # unknown robot
    // ROBOT_ID_UAV_1 = 1
    // ROBOT_ID_AGILEX_1 = 2
    // ROBOT_ID_AGILEX_2 = 3
    // ROBOT_ID_LYNX = 4
    // ROBOT_ID_SPOT = 5
    // ROBOT_ID_VAB = 6
    // ROBOT_ID_UAV_TELEPILOT = 7
    // ROBOT_ID_UGV_TELEPILOT = 8
    //Int uid (0)

    String type (_type)
    String name (_name)

    // Latitude [deg] - Longitude [deg] - Altitude (geoide)
    Double lat (_lat)
    Double lon (_lon)

    // Body frame linear velocity [m/s]

    Int color (_color)

    // Battery voltage
    Double battery_voltage (24)

    // Estimation of battery charge (0-100) negative if unknown
    Int battery_percentage (75)

    // Mean Sea Level altitude (m)
    Double altitude_msl (500)

    // Robot heading with respect to north direction positive
    Double heading_rot (180)

    // State of the physical emergency stop
    Bool emergency_stop (0)

    // Safety stop in case of communication failure
    Bool failsafe (0)

    // Current operating mode
    // OPERATING_MODE_UNKNOWN = 0           # Default value
    // OPERATING_MODE_MANUAL = 1            # Operated by security pilot
    // OPERATING_MODE_TELEOPERATION = 2     # Operated by remote operator
    // OPERATING_MODE_AUTONOMOUS = 3        # Operated by on-board computer
    Int operation_mode (0)
    String status ("")
    (operation_mode == 1) ? "Manual" : ((operation_mode == 2 ) ? "TeleOP" : ((operation_mode == 3 ) ? "Auto" : "???")) =:> status


    Spike start_locate
    Spike stop_locate
}