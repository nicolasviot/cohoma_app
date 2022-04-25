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
import SafetyPilot
import Trap
import Vehicule
import TaskLayer
import LimaLayer
import ExclusionAreaLayer



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
    //init_lat = 43.315313261816485
    //Caylus data 
    init_lat = 44.27432196595285
  }
  double init_lon = get_arg_double (argc, argv, 2)
  if (init_lon == -1){
    //init_lon = 1.404974527891014
    //Caylus data
    init_lon = 1.729783361205679
  }
  //maximum lvl of zoom of 19
  int init_zoom = 19

  
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
      //itinerary_edges aka ctrl_visibility.visible.layer.itinerary_edges
      edges aka ctrl_visibility.visible.layer.edges
      manager aka ctrl_visibility.visible.layer.manager

      String name ("Navgraph")
    }
    Component itineraries {
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible{
          Itineraries layer (map, f)
        }
      }
      itineraries_list aka ctrl_visibility.visible.layer.itineraries_list
      ref_current_itinerary aka ctrl_visibility.visible.layer.ref_current_itinerary
      edge_released_na aka  ctrl_visibility.visible.layer.edge_released_na
      id aka ctrl_visibility.visible.layer.id
      String name ("Itineraries")
    }

    Component traps{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          List layer{
            Trap trap (map, $init_lat , $init_lon- 0.002, 0)
          }
        }
      }
      String name("Traps")
      traplayer aka ctrl_visibility.visible.layer
    }
    Component tasks{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          TaskLayer layer (map)
        }
      }
      String name("Tasks")
      tasklayer aka ctrl_visibility.visible.layer
    }
    Component lima{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          LimaLayer layer (map)
        }
      }
      String name("LIMA")
    }
    Component result{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          TaskLayer layer (map)
        }
      }
      String name("Result")
    }
    Component mission_zones{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible{
          ExclusionAreaLayer layer (map)
        }
      }
      String name("Exclusion areas")
    }


    Component actors{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          List layer{
            SafetyPilot sfty_pilot (map, $init_lat , $init_lon- 0.005, 0)
          }
        }
      }
      String name("Actors")
    }

      
    
    addChildrenTo map.layers {
      geoportail,
      osm,
      mission_zones,
      satelites,
      navgraph,
      itineraries,
      traps, 
      tasks,
      actors,
      lima,
      result
    }
  }


  show_reticule -> l.map.reticule.show_reticule, l.map.layers.navgraph.ctrl_visibility.visible.layer.create
  hide_reticule -> l.map.reticule.hide_reticule, l.map.layers.navgraph.ctrl_visibility.visible.layer.edit

  Component right_pannel {
    Translation t (1024, 0)
    Rectangle bg (0, 0, 700, 900)
    RightPannel right_pannel (root, f)

  }

  // Ros node w/ all sub and pub fonctions
  RosManager ros_manager(root, l.map, l.map.layers.navgraph.manager)
  right_pannel.right_pannel.plan_request -> ros_manager.plan_request
  right_pannel.right_pannel.validate_plan -> ros_manager.validate_plan
  right_pannel.right_pannel.update_graph -> ros_manager.update_graph
  right_pannel.right_pannel.test_multiple_itineraries_spike -> ros_manager.test_multiple_itineraries_spike
  right_pannel.right_pannel.itineraryPannel.plan_set ->ros_manager.validate_plan
  // Strips container
  StripContainer strips (f, 0, 768)/*
    vab aka ctrl_visibility.visible.layers.[1]
      agilex1 aka ctrl_visibility.visible.layers.[2]
      agilex2 aka ctrl_visibility.visible.layers.[3]
      lynx aka ctrl_visibility.visible.layers.[4]
      spot aka ctrl_visibility.visible.layers.[5]
      drone aka ctrl_visibility.visible.layers.[6]
      Double battery_voltage(0)
    Int battery_percentage(0)
    Double altitude_msl(0)
    Double heading_rot(0)
    Bool emergency_stop(0)
    Bool failsafe(0)
    Int operation_mode(0)*/
  // VAB
  l.map.layers.satelites.vab.battery_voltage =:> strips.strip_vab.battery_voltage
  l.map.layers.satelites.vab.battery_percentage =:> strips.strip_vab.battery_percentage
  l.map.layers.satelites.vab.altitude_msl =:> strips.strip_vab.altitude_msl
  l.map.layers.satelites.vab.heading_rot =:> strips.strip_vab.heading_rot
  l.map.layers.satelites.vab.emergency_stop =:> strips.strip_vab.emergency_stop
  l.map.layers.satelites.vab.failsafe =:> strips.strip_vab.failsafe
  l.map.layers.satelites.vab.operation_mode =:> strips.strip_vab.operation_mode


  // agilex 1
  l.map.layers.satelites.agilex1.battery_voltage =:> strips.strip_agilex_1.battery_voltage
  l.map.layers.satelites.agilex1.battery_percentage =:> strips.strip_agilex_1.battery_percentage
  l.map.layers.satelites.agilex1.altitude_msl =:> strips.strip_agilex_1.altitude_msl
  l.map.layers.satelites.agilex1.heading_rot =:> strips.strip_agilex_1.heading_rot
  l.map.layers.satelites.agilex1.emergency_stop =:> strips.strip_agilex_1.emergency_stop
  l.map.layers.satelites.agilex1.failsafe =:> strips.strip_agilex_1.failsafe
  l.map.layers.satelites.agilex1.operation_mode =:> strips.strip_agilex_1.operation_mode

  // agilex 2
  l.map.layers.satelites.agilex2.battery_voltage =:> strips.strip_agilex_2.battery_voltage
  l.map.layers.satelites.agilex2.battery_percentage =:> strips.strip_agilex_2.battery_percentage
  l.map.layers.satelites.agilex2.altitude_msl =:> strips.strip_agilex_2.altitude_msl
  l.map.layers.satelites.agilex2.heading_rot =:> strips.strip_agilex_2.heading_rot
  l.map.layers.satelites.agilex2.emergency_stop =:> strips.strip_agilex_2.emergency_stop
  l.map.layers.satelites.agilex2.failsafe =:> strips.strip_agilex_2.failsafe
  l.map.layers.satelites.agilex2.operation_mode =:> strips.strip_agilex_2.operation_mode

  // lynx
  l.map.layers.satelites.lynx.battery_voltage =:> strips.strip_lynx.battery_voltage
  l.map.layers.satelites.lynx.battery_percentage =:> strips.strip_lynx.battery_percentage
  l.map.layers.satelites.lynx.altitude_msl =:> strips.strip_lynx.altitude_msl
  l.map.layers.satelites.lynx.heading_rot =:> strips.strip_lynx.heading_rot
  l.map.layers.satelites.lynx.emergency_stop =:> strips.strip_lynx.emergency_stop
  l.map.layers.satelites.lynx.failsafe =:> strips.strip_lynx.failsafe
  l.map.layers.satelites.lynx.operation_mode =:> strips.strip_lynx.operation_mode

  // spot
  l.map.layers.satelites.spot.battery_voltage =:> strips.strip_spot.battery_voltage
  l.map.layers.satelites.spot.battery_percentage =:> strips.strip_spot.battery_percentage
  l.map.layers.satelites.spot.altitude_msl =:> strips.strip_spot.altitude_msl
  l.map.layers.satelites.spot.heading_rot =:> strips.strip_spot.heading_rot
  l.map.layers.satelites.spot.emergency_stop =:> strips.strip_spot.emergency_stop
  l.map.layers.satelites.spot.failsafe =:> strips.strip_spot.failsafe
  l.map.layers.satelites.spot.operation_mode =:> strips.strip_spot.operation_mode

  // drone
  l.map.layers.satelites.drone.battery_voltage =:> strips.strip_drone.battery_voltage
  l.map.layers.satelites.drone.battery_percentage =:> strips.strip_drone.battery_percentage
  l.map.layers.satelites.drone.altitude_msl =:> strips.strip_drone.altitude_msl
  l.map.layers.satelites.drone.heading_rot =:> strips.strip_drone.heading_rot
  l.map.layers.satelites.drone.emergency_stop =:> strips.strip_drone.emergency_stop
  l.map.layers.satelites.drone.failsafe =:> strips.strip_drone.failsafe
  l.map.layers.satelites.drone.operation_mode =:> strips.strip_drone.operation_mode



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
      GraphNode temporary (l.map,f, 0, 0, 50, 50, 50)
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
      Node new (root.l.map, root.f, $root.l.map.pointer_lat, $root.l.map.pointer_lon, 0, 0, "by_operator", 0, root.l.map.layers.navgraph.manager)
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
  Spike disable_drag
  Spike renable_drag
  Spike addEdgeSpike1
  Spike addEdgeSpike2
  del -> clear_all
  LogPrinter lp ("debug selected_id dans le main")
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
      NoOutline _
      NoFill _
      GraphNode temporary (l.map, f, 0, 0, 50, 50, 50)
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon
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

      index =:> lp.input

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

    idle -> shift_on (shift, addEdgeSpike1)
    shift_on -> preview_on (root.l.map.layers.navgraph.manager.selected_id, add_first_wpt)
    preview_on -> idle (shift_r, add_segment)
    shift_on -> idle (shift_r, addEdgeSpike2)

  }

  addEdgeSpike1 -> addEdge.preview_on.temporary.disable_drag, clear_temp_list
  addEdgeSpike2 -> hide_reticule, addEdge.preview_on.temporary.renable_drag

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



/* */
  add_segment -> (root){
    for (int i = 1; i < $root.addEdge.preview_on.temp_id_list.size; i++){
      int src = $root.addEdge.preview_on.temp_id_list.[i]
      int dest = $root.addEdge.preview_on.temp_id_list.[i+1]
      print (root.addEdge.preview_on.temp_id_list.[i])
      addChildrenTo root.l.map.layers.navgraph.edges {
        Edge _(src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
     }
    }
  }
}



//TODO : use inheritance for graphNode, Trap, Vehicule
//TODO : tester le constructeur C++