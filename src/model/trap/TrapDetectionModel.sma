use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}

_define_
//TrapDetectionModel (Process _context, int _robot_id, double _lat, double _lon)
TrapDetectionModel (string _uid, double _lat, double _lon)
{
    //TextPrinter tp

    String uid (_uid) // Made with combination of local_id & robot_id ?

    Double lat (_lat)
    Double lon (_lon)
    //Double altitude_msl (0)
    
    print ("New model of trap detection " + uid + "\n")
}