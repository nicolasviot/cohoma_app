use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
VehicleModel (Process _context, int _uid, int _type, string _code, string _title, double _lat, double _lon)
{
    //context aka _context

    Int uid (_uid)
    Int type (_type)
    String code (_code)
    String title (_title)
    //String name (_name)

    // Latitude [deg] - Longitude [deg] - Altitude (geoide)
    Double lat (_lat)
    Double lon (_lon)

    // Body frame linear velocity [m/s]

    // Model of the operator currently in charge of our robot
    Ref ref_operator (nullptr)
    DerefInt operator_color (ref_operator, "color", DJNN_GET_ON_CHANGE)

    // FIXME: TODO after a D&D
    /*ref_operator -> na_ref_operator:(this) {
        _ref_operator = getRef (this.ref_operator)
        if (&_ref_operator != null) {
            print ("ref_operator\n")
            _ref_operator.robots.add = this
            setRef (_ref_operator.robots.add, this)
        }
    }*/

    print ("New model of Vehicle (" + uid + ") type: " + code + " (" + type + ") title: " + title  + "\n")

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


    //Capacité robots
    Bool camera (0)
    Bool laser (0)

    //Ignorrer les pieges dans le manager
    Bool detect_traps (1)
    TextPrinter tp
    "TODO: faire un message pour dire que " + title + " trap detection " + detect_traps => tp.input 

    // Current operating mode
    // OPERATING_MODE_UNKNOWN = 0           # Default value
    // OPERATING_MODE_MANUAL = 1            # Operated by security pilot
    // OPERATING_MODE_TELEOPERATION = 2     # Operated by remote operator
    // OPERATING_MODE_AUTONOMOUS = 3        # Operated by on-board computer
    Int operation_mode (0)
    String operation_mode_status ("Unknown")
    (operation_mode == 1) ? "Manual" : ((operation_mode == 2 ) ? "TeleOP" : ((operation_mode == 3 ) ? "Auto" : "Unknown")) =:> operation_mode_status


    //TODO bind to messages for life of the robot =:> if no data --> warning
    String status ("error")


    /// Alive mecanism
    //link status with geo pos data
   Spike data_in
   lat -> data_in

   FSM link_status_FSM{
      State disconnected{
        "warning" =: status
      }
        
      State connected{
        Timer status_timer (4000) //wait 4seconds
        data_in -> status_timer.reset
        "ok" =: status
      }

      disconnected -> connected (data_in)
      connected -> disconnected (connected.status_timer.end) 
   }




    Spike start_locate
    Spike stop_locate
}