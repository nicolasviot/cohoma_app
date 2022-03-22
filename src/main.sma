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
/*import ros_subscriber
import ros_publisher
*/import Strip
import NavGraph
import GraphPannel
import Node
import Edge
import RosManager
import CheckBox

_native_code_
%{
#include "cpp/tiles_manager.h"
#include "cpp/coords-utils.h"
#include <iostream>
#include <string>

#ifndef NO_ROS
#include "rclcpp/rclcpp.hpp"
#endif

static Process* find_without_warning (Process* p, string path)
{
  return p->find_child_impl(path);
}

static void
init_ros ()
{
#ifndef NO_ROS
  rclcpp::init(0,0); //argc, argv);
#endif
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
  Frame f ("CoHoMa", 0, 0, init_width, init_height)
  Exit quit (0, 1)
  f.close->quit



  Spike show_reticule
  Spike hide_reticule

  Component right_pannel{
    Translation t(1024, 0)
    Rectangle bg (0, 0, 700, 900)
    GraphPannel graph_pannel(root, f)
  }

  
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


  //Create one layer per data.
  // from bottom to top :
  //  - geoportail tiles
  //  - OSM tiles
  //  - Waypoints + Navigation graphs 
  //  - Vehicules TODO
  //  - Traps  TODO
  //  - Zones  TODO


  Layer l {
    Map map (f, 0, 0, init_width, init_height, init_lat, init_lon, init_zoom)
    Component layer1 {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          MapLayer layer (f, map, load_geoportail_tile, "geoportail")
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      opacity->l.damaged
      String name ("Geoportail")
    }
    Component layer2 {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          MapLayer layer (f, map, load_osm_tile, "osm")
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      opacity->l.damaged
      String name ("OSM")
    }
    Component wp {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          Waypoints layer (map, $init_lat, $init_lon, $r_1, $g_1, $b_1)
        }
      }
      battery_voltage aka ctrl_visibility.visible.layer.battery_voltage
      altitude_msl aka ctrl_visibility.visible.layer.altitude_msl
      rot aka ctrl_visibility.visible.layer.rot
      String name ("Wp")
    }
    Component wp2 {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          Waypoints layer (map, $init_lat, $init_lon, $r_2, $g_2, $b_2)
        }
      }
      battery_voltage aka ctrl_visibility.visible.layer.battery_voltage
      altitude_msl aka ctrl_visibility.visible.layer.altitude_msl
      rot aka ctrl_visibility.visible.layer.rot
      String name ("Wp2")
    }
    Component navgraph {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          NavGraph layer (map, f)
        }
      }
      nodes aka ctrl_visibility.visible.layer.nodes
      shadow_edges aka ctrl_visibility.visible.layer.shadow_edges
      edges aka ctrl_visibility.visible.layer.edges
      manager aka ctrl_visibility.visible.layer.manager
      String name ("Navgraph")
    }


    List satelites
    addChildrenTo satelites {
      wp,
      wp2
    } 
    addChildrenTo map.layers {
      layer1,
      layer2,
      satelites,
      navgraph
    }
  }

  // Ros node w/ all sub and pub fonctions
  RosManager ros_manager(root, l.map, l.map.layers.navgraph.manager)

  right_pannel.graph_pannel.plan_request -> ros_manager.plan_request
  right_pannel.graph_pannel.validate_plan -> ros_manager.validate_plan
  right_pannel.graph_pannel.update_graph -> ros_manager.update_graph


  // Strips container
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
  


  // Keyboard inputs 
  // Does not work on some keyboards
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
  Spike del
  Spike del_r
  f.key\-pressed == 16777223 -> del




  // Add waypoints FSM
  Spike addWptToLayer
  FSM addNode {
    State idle 
    State preview{
      Waypoints temporary (l.map, 0, 0, 50, 50, 50)
       l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon
    }
    idle -> preview (ctrl, show_reticule)
    preview -> idle (ctrl_r, hide_reticule)
    preview -> preview (f.release, addWptToLayer) 
  }


 addWptToLayer -> (root){
      addChildrenTo root.l.map.layers.navgraph.nodes{
        Node new (root.l.map, 0, 0, 0, 0, "added_manually", 0, root.l.map.layers.navgraph.manager)
       
    }
      root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].wpt.lat = root.addNode.preview.temporary.lat
      root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].wpt.lon = root.addNode.preview.temporary.lon
      
      root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].id = root.l.map.layers.navgraph.nodes.size - 1
      
  }


  //Add Edge between waypoints 
  Spike clear_temp_list
  Spike add_segment
  Spike add_first_wpt
  Spike clear_all
  del -> clear_all
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
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon
      0 =: temporary.opacity
      0 =: temporary.outline_opacity
      OutlineOpacity _ (0.5)
      OutlineWidth _ (5)
      OutlineColor _ (180, 90, 140)
      Translation pos (0, 0)
    
      Line temp_shadow_edge (0, 0, 0, 0)

      Int index (1)
      RefProperty current (l.map.layers.navgraph.nodes.[index])
      index->(root) {
        setRef (root.addEdge.preview_on.current, root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index])
      }
      l.map.layers.navgraph.manager.selected_id + 1 =:> index

      index -> (root){
        root.addEdge.preview_on.temp_shadow_edge.x1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.c.cx

        root.addEdge.preview_on.temp_shadow_edge.y1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.c.cy
      }

 
      DerefDouble ddx (current, "wpt/c/cx", DJNN_GET_ON_CHANGE)
      DerefDouble ddy (current, "wpt/c/cy", DJNN_GET_ON_CHANGE)
      DerefDouble ddtx (current, "wpt/pos/tx", DJNN_GET_ON_CHANGE)
      DerefDouble ddty (current, "wpt/pos/ty", DJNN_GET_ON_CHANGE)

      ddx.value  =:> temp_shadow_edge.x1
      ddy.value =:> temp_shadow_edge.y1
      ddtx.value =:> pos.tx
      ddty.value =:> pos.ty

      temporary.c.cx=:> temp_shadow_edge.x2
      temporary.c.cy=:> temp_shadow_edge.y2


      



      }
    

    idle -> shift_on (shift, clear_temp_list)
    shift_on -> preview_on (root.l.map.layers.navgraph.manager.selected_id, add_first_wpt)
    preview_on -> idle (shift_r, add_segment)
    shift_on -> idle (shift_r, hide_reticule)

  }

  // clear shadow edge list
  clear_temp_list -> (root){
    for (int i = $root.addEdge.preview_on.temp_id_list.size; i >= 1; i--) {
      delete root.addEdge.preview_on.temp_id_list.[i]
    }
    for (int i = $root.l.map.layers.navgraph.shadow_edges.size; i >= 2; i--){
      // keep the OutlineOpacity for now.
      delete root.l.map.layers.navgraph.shadow_edges.[i]
    }
  }

  // clear everything (waypoints + edges) => does not work 
  clear_all -> (root){
    for (int i =$root.l.map.layers.navgraph.edges.size; i>= 1; i--){
      delete root.l.map.layers.navgraph.edges.[i]
    }

    for (int i =$root.l.map.layers.navgraph.nodes.size; i>= 1; i--){
      delete root.l.map.layers.navgraph.nodes.[i]
    }
    for (int i =$root.l.map.layers.navgraph.shadow_edges.size; i>=1; i--){
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
/*  Dispatcher dispatch (sub, l.map.layers.satelites)
  sub.longitude =:> l.map.layers.satelites.[2].lon
  sub.latitude =:> l.map.layers.satelites.[2].lat
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
    Int height (0)
    Int width (0)

    Translation pos2 (0, 0)
    FontWeight _ (50)
    TextAnchor _ (0)
    s2.y + s2.height + 10 =:> pos2.ty
    int off_y = 0
    int nb_items = 0
    List cb_left {
      for item : l.map.layers {
        if (find_without_warning (item, "name") != 0 ) {
          Component _ {
            CheckBox cb (getString (item.name), 5, off_y)
            cb.state =:> item.ctrl_visibility.state
            width aka cb.min_width
          }
          off_y += 20
          nb_items ++
        }
      }
    }
    MaxList max_width (cb_left, "width")
    CheckBox cb_wp ("wp", 30, 0)
    cb_wp.state =:> l.map.layers.satelites.[1].ctrl_visibility.state
    CheckBox cb_wp2 ("wp2", 30, 20)
    cb_wp2.state =:> l.map.layers.satelites.[2].ctrl_visibility.state
    max_width.output + 10 =:> cb_wp.x, cb_wp2.x

    s1.height + s2.height + nb_items * 20 + 10 =:> height
    s1.width =:> width
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
      (sliders.width - 18)- anim.output * (sliders.width - 18) + 28 =:> main_bg.width
      (sliders.height - 13 + sliders.pos.ty) - anim.output * (sliders.height - 13 + sliders.pos.ty) + 28 =:> main_bg.height
    }
    State unfolded {
      1 =: sliders.sc.sx, sliders.sc.sy
      (sliders.s1.width + 10) =: main_bg.width
      (sliders.height + 15 + sliders.pos.ty) =: main_bg.height
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
