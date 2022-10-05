use core
use gui
use base

_native_code_
%{
    #include "cpp/coords-utils.h"

%}

_define_
VehiculeModel (Process map, string _type, string _name, double _lat, double _lon, int _color)
{
    String type (_type)
    String name (_name)
    Double lat (_lat)
    Double lon (_lon)
    Int color (_color)

    Double battery_voltage(24)
    Int battery_percentage(75)
    Double altitude_msl(500)
    Double heading_rot(180)
    Bool emergency_stop(0)
    Bool failsafe(0)
    Int operation_mode(0)

    Scaling sc (1, 1, 0, 0)
    map.zoom =:> sc.sx, sc.sy

    Translation pos (0, 0)
    map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
    map.ypan - map.cur_ref_y + map.py0 =:> pos.ty
 
    Translation screen_translation (0, 0)
    map.t0_y - lat2py ($lat, $map.zoomLevel) =:> screen_translation.ty
    (lon2px ($lon, $map.zoomLevel) - map.t0_x) =:> screen_translation.tx

    Rotation rot (0, 0, 0)
    heading_rot =:> rot.a
    

    /*Switch graphics(vab) {
        Component vab{
            svg = loadFromXML ("res/svg/vab.svg")
            icon << svg.icon
            color =: icon.shape.fill.value
        }
        Component agilex1{
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            color =: icon.shape.fill.value
        }
        Component agilex2{
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            color =: icon.shape.fill.value
        }
        Component lynx {
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            color =: icon.shape.fill.value
        }
        Component spot {
            svg = loadFromXML ("res/svg/robot.svg")
            icon << svg.icon
            color =: icon.shape.fill.value
        }
        Component drone {
            svg = loadFromXML ("res/svg/drone.svg")
            icon << svg.icon
            color =: icon.shape.fill.value
          
        }
    } 
    type =:> graphics.state*/

}