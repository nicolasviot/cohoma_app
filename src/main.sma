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
use display

import Slider
import Map
import MapLayer
import Animator
import Waypoints
import Dispatcher
import ros_subscriber
import ros_publisher
import Strip
import NavGraph
import GraphPannel
import Node
import Edge

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
  Spike show_reticule
  Spike hide_reticule
 Component RightPannel{
    Translation t(1024, 0)
    Rectangle bg (0, 0, 700, 768)
    GraphPannel graphpannel(root)

  }

  RosSubscriber sub ("/robot_state")
//  RosPublisher  ros_pub ("/robot_state")

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
  
  Int r_1(255)
  Int g_1(0)
  Int b_1(0)
  Int r_2(0)
  Int g_2(255)
  Int b_2(0)
  /*
  FillColor color1 (0, 250, 0)
  FillColor color2 (0, 0, 250)
  */
  Layer l {
    Map map (f, 0, 0, init_width, init_height, init_lat, init_lon, init_zoom)
    MapLayer layer1 (f, map, load_geoportail_tile, "geoportail")
    MapLayer layer2 (f, map, load_osm_tile, "osm")
    Waypoints wp (map, $init_lat, $init_lon, $r_1, $g_1, $b_1)
    Waypoints wp2 (map, $init_lat, $init_lon, $r_2, $g_2, $b_2)
    NavGraph navgraph (map)
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
      satelites,
      navgraph
    }
  }
  Component StripsComponent{
    Translation t (0, 768)
    Strip strip1("uav 1", f)
    t.tx =:> strip1.parent_tx
    t.ty =:> strip1.parent_ty
    Translation t2(400, 0) 
    Strip strip2("uav 2", f)
    t.tx + t2.tx =:> strip2.parent_tx
    t.ty + t2.ty =:> strip2.parent_ty

  }
  


  Spike ctrl
  Spike ctrl_r
  f.key\-pressed == 16777249 -> ctrl
  f.key\-released == 16777249-> ctrl_r
  Spike shift
  Spike shift_r
  f.key\-pressed == 16777248-> shift
  f.key\-released == 16777248-> shift_r
  Spike space
  Spike space_r
  f.key\-pressed == 32 -> space
  f.key\-released == 32 -> space_r
  
  Spike addWptToLayer
  FSM addNode {
    State idle 
    State preview{
      Waypoints temporary (l.map, 0, 0, 50, 50, 50)
      //Waypoints temporary (l.map, 0, 0, 50, 50, 50)
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon
    }
    idle -> preview (ctrl, show_reticule)
    preview -> idle (ctrl_r, hide_reticule)
    preview -> preview (f.release, addWptToLayer) 
  }
  LogPrinter lp ("debug add index")
 addWptToLayer -> (root){
      addChildrenTo root.l.map.layers.navgraph.nodes{
        Node new (root.l.map, 0, 0, 0, 0, "added_manually", 0, root.l.map.layers.navgraph.manager)
       
    }
      root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].wpt.lat = root.addNode.preview.temporary.lat
      root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].wpt.lon = root.addNode.preview.temporary.lon
      
      root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].id = root.l.map.layers.navgraph.nodes.size - 1
      
  }

  Spike clear_temp_list
  Spike add_segment
  Spike add_first_wpt
  FSM addEdge{
    State idle
    State shift_on
    State preview_on{
      List temp_id_list 
      root.l.map.layers.navgraph.manager.selected_id -> (root){
        addChildrenTo root.addEdge.preview_on.temp_id_list{
          Int _($root.l.map.layers.navgraph.manager.selected_id)
          }

        int size = $root.addEdge.preview_on.temp_id_list.size 
        int src = $root.addEdge.preview_on.temp_id_list.[size]
        int dest = $root.addEdge.preview_on.temp_id_list.[size - 1]
        addChildrenTo root.l.map.layers.navgraph.shadow_edges{
          Edge _(src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
         
          }


      }
      Waypoints temporary (l.map, 0, 0, 50, 50, 50)
      //Waypoints temporary (l.map, 0, 0, 50, 50, 50)
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon
      0 =: temporary.opacity
      0 =: temporary.outline_opacity
      OutlineOpacity _ (0.5)
      OutlineWidth _ (5)
      OutlineColor _ (180, 90, 140)
      Translation pos (0, 0)
      //l.map.layers.navgraph.nodes.[$l.map.layers.navgraph.nodes.size].wpt.pos.tx =:> pos.tx
      //l.map.layers.navgraph.nodes.[$l.map.layers.navgraph.nodes.size].wpt.pos.ty =:> pos.ty
  
      LogPrinter lp ("debug :" )
      Line temp_shadow_edge (0, 0, 0, 0)

      Int index (1)
      RefProperty current (l.map.layers.navgraph.nodes.[index])
      index->(root) {
        setRef (root.addEdge.preview_on.current, root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index])
      }
      l.map.layers.navgraph.manager.selected_id + 1 =:> index

      l.map.layers.navgraph.nodes.[index].wpt.c.cx =: temp_shadow_edge.x1

      l.map.layers.navgraph.nodes.[index].wpt.c.cy =: temp_shadow_edge.y1
 
 
      DerefDouble ddx (current, "wpt/c/cx", DJNN_GET_ON_CHANGE)
      DerefDouble ddy (current, "wpt/c/cy", DJNN_GET_ON_CHANGE)
      DerefDouble ddtx (current, "wpt/pos/tx", DJNN_GET_ON_CHANGE)
      DerefDouble ddty (current, "wpt/pos/ty", DJNN_GET_ON_CHANGE)

      ddx.value  =:> temp_shadow_edge.x1
      ddy.value =:> temp_shadow_edge.y1
      ddtx.value =:> pos.tx
      ddty.value =:> pos.ty

      /*
      l.map.layers.navgraph.nodes.[l.map.layers.navgraph.manager.selected_id].wpt.c.cx =:> temp_shadow_edge.x1
      l.map.layers.navgraph.nodes.[l.map.layers.navgraph.manager.selected_id].wpt.c.cy =:> temp_shadow_edge.y1
      */
      temporary.c.cx=:> temp_shadow_edge.x2
      temporary.c.cy=:> temp_shadow_edge.y2


      



      }
    

    idle -> shift_on (shift, clear_temp_list)
    shift_on -> preview_on (root.l.map.layers.navgraph.manager.selected_id, add_first_wpt)
    preview_on -> idle (shift_r, add_segment)
    shift_on -> idle (shift_r, hide_reticule)

  }
  clear_temp_list -> (root){
    for (int i = $root.addEdge.preview_on.temp_id_list.size; i >= 1; i--) {
      delete root.addEdge.preview_on.temp_id_list.[i]
    }
    for (int i = $root.l.map.layers.navgraph.shadow_edges.size; i >= 2; i--){
      // keep the OutlineOpacity for now.
      delete root.l.map.layers.navgraph.shadow_edges.[i]
    }
  }
  clear_temp_list -> show_reticule
  add_segment -> hide_reticule


add_first_wpt -> (root){
  addChildrenTo root.addEdge.preview_on.temp_id_list{
    Int _($root.l.map.layers.navgraph.manager.selected_id)
    }

}
add_segment -> (root){
  for (int i = 1; i < $root.addEdge.preview_on.temp_id_list.size; i++){
    int src = $root.addEdge.preview_on.temp_id_list.[i]
    int dest = $root.addEdge.preview_on.temp_id_list.[i+1]
  
    addChildrenTo root.l.map.layers.navgraph.edges{
      Edge _(src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
    }
  }
}


  svg = loadFromXML ("res/svg/icon_menu.svg")
  l.map.layers.satelites.[1].battery_voltage =:> StripsComponent.strip1.battery_voltage
  l.map.layers.satelites.[2].battery_voltage =:> StripsComponent.strip2.battery_voltage
  l.map.layers.satelites.[1].altitude_msl =:> StripsComponent.strip1.altitude_msl
  l.map.layers.satelites.[2].altitude_msl =:> StripsComponent.strip2.altitude_msl
  l.map.layers.satelites.[1].rot.a =:> StripsComponent.strip1.compass_heading
  l.map.layers.satelites.[2].rot.a =:> StripsComponent.strip2.compass_heading
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
  
 

  FSM reticule{
    State off 
    State on {
      OutlineColor _(Black)
  Line l1 (0, 0, 0, 0)
  Line l2 (0, 0, 0, 0)
  f.move.x =:> l1.x1, l1.x2
  f.move.y =:> l2.y1, l2.y2
  f.width =:> l2.x2
  f.height =:> l1.y2 
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
    off -> on (show_reticule)
    on -> off (hide_reticule)
  }


  

}
