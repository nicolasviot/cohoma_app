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
import UpperLeftMenu
import StripContainer
import Trap


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
  double init_lat = get_arg_double (argc, argv, 1)
  if (init_lat == -1) {
    init_lat = 43.315313261816485
  }
  double init_lon = get_arg_double (argc, argv, 2)
  if (init_lon == -1){
    init_lon = 1.404974527891014
  }
  int init_zoom = 17
  
  int r_1 = 255
  int g_1 = 0
  int b_1 = 0
  int r_2 = 0
  int g_2 = 255
  int b_2 = 0

  Frame f ("CoHoMa", 0, 0, init_width + 300, init_height + 300)
  Exit quit (0, 1)
  f.close->quit
  mouseTracking = 1
  f.background_color.r = 50
  f.background_color.g = 50
  f.background_color.b = 50

  Spike show_reticule
  Spike hide_reticule

  //Create one layer per data.
  // from bottom to top :
  //  - geoportail tiles
  //  - OSM tiles
  //  - Waypoints + Navigation graphs 
  //  - Vehicules TODO
  //  - Traps  TODO
  //  - Zones  TODO

  Component l {
    Map map (f, 0, 0, init_width, init_height, init_lat, init_lon, init_zoom)
    Component geoportail {
      Switch ctrl_visibility (visible) {
        Component hidden
        Layer visible {
          MapLayer layer (f, map, load_geoportail_tile, "geoportail")
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      String name ("Geoportail")
    }
    Component osm {
      Switch ctrl_visibility (visible) {
        Component hidden
        Layer visible {
          MapLayer layer (f, map, load_osm_tile, "osm")
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      String name ("OSM")
    }
    Component satelites {
      Switch ctrl_visibility (visible) {
        Component hidden
        Layer visible {
          List layers {
            Waypoints wp (map, $init_lat, $init_lon, r_1, g_1, b_1)
            Waypoints wp2 (map, $init_lat, $init_lon, r_2, g_2, b_2)
          }
        }
      }
      wp aka ctrl_visibility.visible.layers.[1]
      wp2 aka ctrl_visibility.visible.layers.[2]
       String name ("Satelites")
    }
    Component navgraph {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          NavGraph layer (map, f)
        }
      }
    Component traps{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          List layers{
            Trap trap (map, $init_lat, $init_lon, r_1, g_1, b_1)
          }
        }
      }
      String name("Traps")
    }
      nodes aka ctrl_visibility.visible.layer.nodes
      shadow_edges aka ctrl_visibility.visible.layer.shadow_edges
      itinerary_edges aka ctrl_visibility.visible.layer.itinerary_edges
      edges aka ctrl_visibility.visible.layer.edges
      manager aka ctrl_visibility.visible.layer.manager
      String name ("Navgraph")
    }


    addChildrenTo map.layers {
      geoportail,
      osm,
      satelites,
      navgraph
    }
  }

  show_reticule->l.map.reticule.show_forever, l.map.layers.navgraph.ctrl_visibility.visible.layer.create
  hide_reticule->l.map.reticule.end_forever, l.map.layers.navgraph.ctrl_visibility.visible.layer.edit

  Component right_pannel {
    Translation t (1024, 0)
    Rectangle bg (0, 0, 700, 900)
    GraphPannel graph_pannel (root, f)
  }

  // Ros node w/ all sub and pub fonctions
  RosManager ros_manager(root, l.map, l.map.layers.navgraph.manager)
  right_pannel.graph_pannel.plan_request -> ros_manager.plan_request
  right_pannel.graph_pannel.validate_plan -> ros_manager.validate_plan
  right_pannel.graph_pannel.update_graph -> ros_manager.update_graph

  // Strips container
  StripContainer strips (f, 0, 768)
  l.map.layers.satelites.wp.battery_voltage =:> strips.strip1.battery_voltage
  l.map.layers.satelites.wp2.battery_voltage =:> strips.strip2.battery_voltage
  l.map.layers.satelites.wp.altitude_msl =:> strips.strip1.altitude_msl
  l.map.layers.satelites.wp2.altitude_msl =:> strips.strip2.altitude_msl
  l.map.layers.satelites.wp.rot.a =:> strips.strip1.compass_heading
  l.map.layers.satelites.wp2.rot.a =:> strips.strip2.compass_heading
  
  UpperLeftMenu menu (l.map, f)

  // Keyboard inputs 
  // Does not work on some keyboards
  Spike ctrl
  Spike ctrl_r
  f.key\-pressed == DJN_Key_Control -> ctrl
  f.key\-released == DJN_Key_Control-> ctrl_r
  Spike shift
  Spike shift_r
  f.key\-pressed == DJN_Key_Shift-> shift
  f.key\-released == DJN_Key_Shift-> shift_r
  Spike space
  Spike space_r
  f.key\-pressed == DJN_Key_Space -> space
  f.key\-released == DJN_Key_Space -> space_r
  Spike del
  Spike del_r
  f.key\-pressed == DJN_Key_Backspace -> del

  // Add waypoints FSM
  Spike addWptToLayer
  FSM addNode {
    State idle 
    State preview{
      Waypoints temporary (l.map, 0, 0, 50, 50, 50)
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon
      f.release -> addWptToLayer
    }
    idle -> preview (ctrl, show_reticule)
    preview -> idle (ctrl_r, hide_reticule)
    //preview -> preview (f.release, addWptToLayer) 
  }

  addWptToLayer -> (root){
    addChildrenTo root.l.map.layers.navgraph.nodes {
      Node new (root.l.map, $root.l.map.pointer_lat, $root.l.map.pointer_lon, 0, 0, "by_operator", 0, root.l.map.layers.navgraph.manager)
    }
    //print (root.l.map.layers.navgraph.nodes.size)
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
  FSM addEdge {
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
      ddx.value =:> temp_shadow_edge.x1
      ddy.value =:> temp_shadow_edge.y1
      ddtx.value =:> pos.tx
      ddty.value =:> pos.ty 
      temporary.c.cx =:> temp_shadow_edge.x2
      temporary.c.cy =:> temp_shadow_edge.y2
    }

    idle -> shift_on (shift, clear_temp_list)
    shift_on -> preview_on (root.l.map.layers.navgraph.manager.selected_id, add_first_wpt)
    preview_on -> idle (shift_r, add_segment)
    shift_on -> idle (shift_r, hide_reticule)

  }

  //Node null_ref (root.l.map, 0, 0, 0, 0, "added_manually", 0, root.l.map.layers.navgraph.manager)
  Ref null_ref (0)
  // clear shadow edge list
  clear_temp_list -> (root){
    
    root.l.map.layers.navgraph.manager.current_wpt = &(root.null_ref)
    root.l.map.layers.navgraph.manager.entered_wpt  =  &(root.null_ref)
    root.addEdge.preview_on.current = &(root.null_ref)

    for (int i = $root.l.map.layers.navgraph.shadow_edges.size; i >= 1; i--){
      // keep the OutlineOpacity for now.
      delete root.l.map.layers.navgraph.shadow_edges.[i]
    }

    for (int i = $root.addEdge.preview_on.temp_id_list.size; i >= 1; i--) {
      delete root.addEdge.preview_on.temp_id_list.[i]
    }
    
    /*
    delete_content root.l.map.layers.navgraph.shadow_edges
    root.l.map.layers.navgraph.shadow_edges.size = 0 
    delete_content root.addEdge.preview_on.temp_id_list
    root.addEdge.preview_on.temp_id_list.size = 0
    */
    //root.l.map.layers.navgraph.manager.selected_id = 1
  }

  // clear everything (waypoints + edges) => does not work 
  clear_all -> (root){
    
    root.l.map.layers.navgraph.manager.current_wpt  =  &(root.null_ref)
    root.l.map.layers.navgraph.manager.entered_wpt  =  &(root.null_ref)
    root.addEdge.preview_on.current = &(root.null_ref)
    for (int i =$root.l.map.layers.navgraph.edges.size; i>= 1; i--){
      delete root.l.map.layers.navgraph.edges.[i]
    }
    for (int i =$root.l.map.layers.navgraph.shadow_edges.size; i>=1; i--){
      delete root.l.map.layers.navgraph.shadow_edges.[i]
    }
    for (int i =$root.l.map.layers.navgraph.nodes.size; i>= 1; i--){
      delete root.l.map.layers.navgraph.nodes.[i]
    }
    
    /*
    delete_content root.l.map.layers.navgraph.edges
    root.l.map.layers.navgraph.edges.size = 0
    delete_content root.l.map.layers.navgraph.shadow_edges
    root.l.map.layers.navgraph.shadow_edges.size = 0
    delete_content root.l.map.layers.navgraph.nodes
    root.l.map.layers.navgraph.nodes.size = 0
    */
    //root.l.map.layers.navgraph.manager.selected_id = 1
  }

  clear_temp_list -> show_reticule
  add_segment -> hide_reticule

  add_first_wpt -> (root){
    root.addEdge.preview_on.current = &(root.null_ref)
    addChildrenTo root.addEdge.preview_on.temp_id_list{
      Int _($root.l.map.layers.navgraph.manager.selected_id)
    }
  }

  add_segment -> (root){
    for (int i = 1; i < $root.addEdge.preview_on.temp_id_list.size; i++){
      int src = $root.addEdge.preview_on.temp_id_list.[i]
      int dest = $root.addEdge.preview_on.temp_id_list.[i+1]

      addChildrenTo root.l.map.layers.navgraph.edges {
        Edge _(src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
     }
    }
    //delete_content root.addEdge.preview_on.null_ref  
  }

   
}
