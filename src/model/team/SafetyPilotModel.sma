use core
use gui
use base

/*_native_code_
%{
    #include "cpp/coords-utils.h"

%}*/

// inherits from OperatorModel ?

_define_
SafetyPilotModel (Process _context, string _type, string _name, double _lat, double _lon, int _color)
{
    //context aka _context

    String type (_type)
    String name (_name)
    Double lat (_lat)
    Double lon (_lon)
    Int color (_color)

    Double altitude_msl (0)
    Double radius (100)

}