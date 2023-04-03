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

    // Latitude [deg] - Longitude [deg]
    Double lat (_lat)
    Double lon (_lon)
    // Robot heading with respect to north direction positive
    Double heading_rot (180)

    // Model of the operator currently in charge of our robot
    Ref ref_operator (nullptr)
    DerefInt operator_color (ref_operator, "color", DJNN_GET_ON_CHANGE)

    //if this vehicle can be reassigned (only ground robots)
    Bool can_allocate (0)
    if(_type == 2){
      can_allocate = 1
    }

    //print ("New model of Vehicle (" + uid + ") type: " + type + " code: " + code + " title: " + title  + " can allocate " + can_allocate +"\n")
    
    // State of the physical emergency stop
    Bool emergency_stop (0)

    // Safety stop in case of communication failure
    Bool failsafe (0)


    //Capacité robots
    Bool laser (0)
    Bool available (0)
    Bool teleoperated (0)
    Bool contact (0)
    Bool detection (0)
    Bool identification (0)

    //Ignorrer les pieges dans le manager
    Bool trap_detection (1)
    TextPrinter tp
    "TODO: faire un message pour dire que " + title + " trap detection " + trap_detection => tp.input 


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
        Timer status_timer (10000) //wait 4seconds
        data_in -> status_timer.reset
        "ok" =: status
      }

      disconnected -> connected (data_in)
      connected -> disconnected (connected.status_timer.end) 
   }

    Spike start_locate
    Spike stop_locate

}