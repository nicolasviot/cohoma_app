/*
 *	Smala Map component
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */


use core
use base
use gui
use animation

import Slider
import Map
import MapLayer
import Animator
import Waypoints
import Dispatcher
import ros_subscriber
import ros_publisher
import Strip


_native_code_
%{
#include "cpp/tiles_manager.h"
#include "cpp/coords-utils.h"
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"

static void
init_ros ()
{
  rclcpp::init(0,0); //argc, argv);
}

int
get_arg_double (int argc, char** argv, int n)
{
    if (argc < n+1) {
      return -1;
    }
    double r = std::strtod (argv[n], nullptr);
    return r;
}


%}

_main_
Component root {
  init_ros ()
  int init_width = 1024
  int init_height = 768
  Frame f ("Map", 0, 0, init_width, init_height)
  Exit quit (0, 1)
  f.close->quit

  RosSubscriber sub ("/RobotState")
  RosPublisher  ros_pub ("/RobotState")

  double init_lat = get_arg_double (argc, argv, 1)
  if (init_lat == -1) {
    init_lat = 43.315313261816485
  }
  double init_lon = get_arg_double (argc, argv, 2)
  if (init_lon == -1){
    init_lon = 1.404974527891014
  }
  int init_zoom = 17
  mouseTracking = 1
  f.background_color.r = 50
  f.background_color.g = 50
  f.background_color.b = 50
  
  FillColor color1 (0, 250, 0)
  FillColor color2 (0, 0, 250)
  NoFill _
  Layer l {
    Map map (f, 0, 0, init_width, init_height, init_lat, init_lon, init_zoom)
    MapLayer layer1 (f, map, load_geoportail_tile, "geoportail")
    MapLayer layer2 (f, map, load_osm_tile, "osm")
    Waypoints wp (map, $init_lat, $init_lon, color1)
    Waypoints wp2 (map, $init_lat, $init_lon, color2)
/*
    Waypoints wp3 (map, $init_lat, $init_lon)
    Waypoints wp4 (map, $init_lat, $init_lon)
    Waypoints wp5 (map, $init_lat, $init_lon)
    Waypoints wp6 (map, $init_lat, $init_lon)
*/
    List satelites
   // g << svg.Strip
    addChildrenTo satelites{
      wp,
      wp2/*,
      wp3,
      wp4,
      wp5,
      wp6
      */
    } 
    addChildrenTo map.layers {
      layer1,
      layer2,
      satelites
    }
  }
  Strip strip1("uav 1", f)
  Strip strip2("uav 2", f)
  svg = loadFromXML ("res/svg/icon_menu.svg")
  l.map.layers.satelites.[1].battery_voltage =:> strip1.battery_voltage
  l.map.layers.satelites.[2].battery_voltage =:> strip2.battery_voltage
  l.map.layers.satelites.[1].altitude_msl =:> strip1.altitude_msl
  l.map.layers.satelites.[2].altitude_msl =:> strip2.altitude_msl
  
  main_bg << svg.layer1.main_bg
  Dispatcher dispatch (sub, l.map.layers.satelites)
  /*sub.longitude =:> l.map.layers.satelites.[1].lon
  sub.latitude =:> l.map.layers.satelites.[1].lat
*/
  Component sliders {
    Scaling sc (0, 0, 0, 0)
    FontFamily _ ("B612")
    FontWeight _ (75)
    FontSize _ (0, 12)
    TextAnchor _ (1)
    FillColor _ (White)
    Text t (0, 15, "Layers opacity")
    Translation pos (0, 20)
    Slider s1 (f, 5, 5, 0, 100)
    s1.width/2 + 5 =:> t.x
    s1.output/100 =:> l.map.layers.layer2.opacity
    Slider s2 (f, 5, 0, 0, 100)
    //s2.output/100 =:> l.map.layers.wp.opacity
    s1.height + 10 =: s2.y
  }
  Animator anim (200, 0, 1, DJN_IN_SINE, 0, 0)
  FSM menu {
    State start {
      fg << clone (svg.layer1.fg)
    }
    State folded {
      fg << svg.layer1.fg
      28 =: main_bg.width, main_bg.height
      0 =: sliders.sc.sx, sliders.sc.sy
      2 =: main_bg.ry
    }
    State unfold {
      anim.output => sliders.sc.sx, sliders.sc.sy
      anim.output * (sliders.s1.width - 18) + 28 =:> main_bg.width
      anim.output * (sliders.s1.height * 2 - 13 + sliders.pos.ty) + 28 =:> main_bg.height
    }
    State fold {
      1 - anim.output => sliders.sc.sx, sliders.sc.sy
      (sliders.s1.width - 18)- anim.output * (sliders.s1.width - 18) + 28 =:> main_bg.width
      (sliders.s1.height * 2 - 13 + sliders.pos.ty) - anim.output * (sliders.s1.height * 2 - 13 + sliders.pos.ty) + 28 =:> main_bg.height
    }
    State unfolded {
      1 =: sliders.sc.sx, sliders.sc.sy
      (sliders.s1.width + 10) =: main_bg.width
      (sliders.s1.height * 2 + 15 + sliders.pos.ty) =: main_bg.height
      5 =: main_bg.ry
    }
    start->folded (f.move) // we need this to avoid a false move event at startup
    folded->unfold (main_bg.enter, anim.start)
    unfold->unfolded (anim.end)
    unfolded->fold (l.map.enter, anim.start)
    fold->folded (anim.end)
  }
  /*
  OutlineColor _(Black)
  Line l1 (0, 0, 0, 0)
  Line l2 (0, 0, 0, 0)
  f.width/2 =:> l1.x1, l1.x2
  f.height/2 =:> l2.y1, l2.y2
  f.width =:> l2.x2
  f.height =:> l1.y2*/


  FillColor _ (White)
  NoOutline _
  Component rr {
    FillOpacity _ (0.5)
 //   Rectangle bg1 (0, 0, 100, 30)
 //   l.map.width/2 =:> bg1.x
 //   l.map.height/2 =:> bg1.y
    Rectangle bg2 (0, 0, 100, 30)
    f.move.x =:> bg2.x
    f.move.y =:> bg2.y
  }
  FillColor _(Black)
  //Text t1 (0, 0, "")
  Text t2 (0, 0, "")
  //rr.bg1.x + 10 =:> t1.x
  //rr.bg1.y + 20 =:> t1.y
  //"" + l.map.lat_center + " " + l.map.lon_center + " " + l.map.g_map.pz.zoom =:> t1.text
  String ty ("")
  String tx ("")
  lat2tiley ($l.map.pointer_lat, $l.map.zoomLevel + 1) => ty
  lon2tilex ($l.map.pointer_lon, $l.map.zoomLevel + 1) => tx
  "" + l.map.pointer_lat + " " + l.map.pointer_lon =:> t2.text
  //"col: " + l.map.pointer_col + " row: " + l.map.pointer_row =:> t2.text
  rr.bg2.x + 10 =:> t2.x
  rr.bg2.y + 20 =:> t2.y

  //t1.width + 20 =:> rr.bg1.width
  t2.width + 20 =:> rr.bg2.width

}
