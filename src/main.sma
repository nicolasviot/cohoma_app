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
import GraphNode
import Strip
import NavGraph
import Itineraries
import RightPannel
import Node
import Edge
import RosManager
import UpperLeftMenu
import StripContainer
import Trap
import Vehicule
import TaskLayer


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

  Frame f ("CoHoMa", 0, 0, init_width + 600, init_height + 600)
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
  //  - GraphNode + Navigation graphs 
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
            Vehicule vab (map, $init_lat, $init_lon, r_1, g_1, b_1, "vab")
            Vehicule agilex1 (map, $init_lat + 0.0005, $init_lon, r_2, g_2, b_2, "agilex1")
            Vehicule agilex2 (map, $init_lat + 0.001, $init_lon, r_2, g_2, b_2, "agilex2")
            Vehicule lynx (map, $init_lat, $init_lon + 0.001, r_2, g_2, b_2, "lynx")
            Vehicule spot (map, $init_lat+ 0.001 , $init_lon + 0.001, r_2, g_2, b_2, "spot")
            Vehicule drone (map, $init_lat+ 0.0015 , $init_lon + 0.0015, r_2, g_2, b_2, "drone")
          }
        }
      }
      vab aka ctrl_visibility.visible.layers.[1]
      agilex1 aka ctrl_visibility.visible.layers.[2]
      agilex2 aka ctrl_visibility.visible.layers.[3]
      lynx aka ctrl_visibility.visible.layers.[4]
      spot aka ctrl_visibility.visible.layers.[5]
      drone aka ctrl_visibility.visible.layers.[6]
      
      String name ("Satelites")
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
      itinerary_edges aka ctrl_visibility.visible.layer.itinerary_edges
      edges aka ctrl_visibility.visible.layer.edges
      manager aka ctrl_visibility.visible.layer.manager

      String name ("Navgraph")
    }
    Component itineraries {
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible{
          Itineraries layer (map, f, l.map.layers.navgraph.ctrl_visibility.visible.layer.nodes)
        }
      }
      itineraries_list aka ctrl_visibility.visible.layer.itineraries_list
      ref_current_itinerary aka ctrl_visibility.visible.layer.ref_current_itinerary
      id aka ctrl_visibility.visible.layer.id
      String name ("Itineraries")
    }

    Component traps{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          List layer{
            Trap trap (map, $init_lat , $init_lon- 0.002, r_1, g_1, b_1)
          }
        }
      }
      String name("Traps")
    }
    Component tasks{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          TaskLayer layer (map)
        }
      }
      String name("Tasks")
    }

      
    


    addChildrenTo map.layers {
      geoportail,
      osm,
      satelites,
      navgraph,
      itineraries,
      traps, 
      tasks
    }
  }

  Double lat_flightsim(40.0)
  Double lon_flightsim(1.30)

  lat_flightsim =:> l.map.lat_center
  lon_flightsim =:> l.map.lon_center

  show_reticule -> l.map.reticule.show_reticule, l.map.layers.navgraph.ctrl_visibility.visible.layer.create
  hide_reticule -> l.map.reticule.hide_reticule, l.map.layers.navgraph.ctrl_visibility.visible.layer.edit

  Component right_pannel {
    Translation t (1024, 0)
    Rectangle bg (0, 0, 700, 900)
    RightPannel right_pannel (root, f)

     //legend for NavGraph
    Component NavGraph
    {
      nav_svg = loadFromXML ("res/svg/GraphNav_legend.svg")
      Translation legend (350, 400)
      nav << nav_svg.GraphNav
      Translation legend_off (-350, -400)

    //TODO Use the buuton to send and updated graph via Ros
    //nav.update_button.rect.press -> xxx
    }

  }

  // Ros node w/ all sub and pub fonctions
  RosManager ros_manager(root, l.map, l.map.layers.navgraph.manager)
  right_pannel.right_pannel.plan_request -> ros_manager.plan_request
  right_pannel.right_pannel.validate_plan -> ros_manager.validate_plan
  right_pannel.right_pannel.update_graph -> ros_manager.update_graph
  right_pannel.right_pannel.test_multiple_itineraries_spike -> ros_manager.test_multiple_itineraries_spike
  right_pannel.right_pannel.itineraryPannel.idSelected =:> l.map.layers.itineraries.id
  // Strips container
 StripContainer strips (f, 0, 768)
 /* l.map.layers.satelites.wp.battery_voltage =:> strips.strip1.battery_voltage
  l.map.layers.satelites.wp2.battery_voltage =:> strips.strip2.battery_voltage
  l.map.layers.satelites.wp.altitude_msl =:> strips.strip1.altitude_msl
  l.map.layers.satelites.wp2.altitude_msl =:> strips.strip2.altitude_msl
  l.map.layers.satelites.wp.rot.a =:> strips.strip1.compass_heading
  l.map.layers.satelites.wp2.rot.a =:> strips.strip2.compass_heading
  */
  UpperLeftMenu menu (l.map, f)

 
  

  // Keyboard inputs 
  // Does not work on some keyboards
  Spike ctrl
  Spike ctrl_r
  f.key\-pressed == DJN_Key_Control -> ctrl
  f.key\-released == DJN_Key_Control -> ctrl_r
  Spike shift
  Spike shift_r
  f.key\-pressed == DJN_Key_Shift -> shift
  f.key\-released == DJN_Key_Shift -> shift_r
  Spike space
  Spike space_r
  f.key\-pressed == DJN_Key_Space -> space
  f.key\-released == DJN_Key_Space -> space_r
  Spike del
  Spike del_r
  f.key\-pressed == DJN_Key_Backspace -> del

  // Add GraphNode FSM
  Spike addWptToLayer
  FSM addNode {
    State idle 
    State preview{
      GraphNode temporary (l.map, 0, 0, 50, 50, 50)
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
    root.l.map.layers.navgraph.nodes[$root.l.map.layers.navgraph.nodes.size].id = root.l.map.layers.navgraph.nodes.size    
  }



  //Add Edge between GraphNode 
  Spike clear_temp_list
  Spike add_segment
  Spike add_first_wpt
  Spike clear_all
  del -> clear_all
  
  Ref null_ref (0)
  RefProperty current_addedge_node (nullptr)
  DerefDouble ddx (current_addedge_node, "wpt/screen_translation/tx", DJNN_GET_ON_CHANGE)
  DerefDouble ddy (current_addedge_node, "wpt/screen_translation/ty", DJNN_GET_ON_CHANGE)
  DerefDouble ddtx (current_addedge_node, "wpt/pos/tx", DJNN_GET_ON_CHANGE)
  DerefDouble ddty (current_addedge_node, "wpt/pos/ty", DJNN_GET_ON_CHANGE)
  
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
        int src = $root.addEdge.preview_on.temp_id_list.[size - 1]
        int dest = $root.addEdge.preview_on.temp_id_list.[size]
        addChildrenTo root.l.map.layers.navgraph.shadow_edges{
          Edge _(src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
         
          }
      }
      GraphNode temporary (l.map, 0, 0, 50, 50, 50)
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
      index->(root) {
        setRef (root.current_addedge_node, root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index])
      }
      l.map.layers.navgraph.manager.selected_id =:> index

      index -> (root){
        root.addEdge.preview_on.temp_shadow_edge.x1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.screen_translation.tx

        root.addEdge.preview_on.temp_shadow_edge.y1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.screen_translation.ty
      }
      ddx.value =:> temp_shadow_edge.x1
      ddy.value =:> temp_shadow_edge.y1
      ddtx.value =:> pos.tx
      ddty.value =:> pos.ty 
      temporary.screen_translation.tx =:> temp_shadow_edge.x2
      temporary.screen_translation.ty =:> temp_shadow_edge.y2
    }

    idle -> shift_on (shift, clear_temp_list)
    shift_on -> preview_on (root.l.map.layers.navgraph.manager.selected_id, add_first_wpt)
    preview_on -> idle (shift_r, add_segment)
    shift_on -> idle (shift_r, hide_reticule)

  }

  clear_temp_list -> (root) {
    
    root.l.map.layers.navgraph.manager.current_wpt = &(root.null_ref)
    root.l.map.layers.navgraph.manager.entered_wpt  =  &(root.null_ref)
    root.current_addedge_node = &(root.null_ref)

    delete_content root.l.map.layers.navgraph.shadow_edges
    delete_content root.addEdge.preview_on.temp_id_list
  }

  clear_all -> (root) {

    root.l.map.layers.navgraph.manager.current_wpt  =  &(root.null_ref)
    root.l.map.layers.navgraph.manager.entered_wpt  =  &(root.null_ref)
    root.current_addedge_node  = &(root.null_ref)
    
    delete_content root.l.map.layers.navgraph.edges
    delete_content root.l.map.layers.navgraph.shadow_edges
    delete_content root.l.map.layers.navgraph.nodes
  }

  clear_temp_list -> show_reticule
  add_segment -> hide_reticule

  add_first_wpt -> (root){
    root.current_addedge_node = &(root.null_ref)
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
  }
}
