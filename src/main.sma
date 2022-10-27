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

import CohomaContext
import model.ModelManager
import map.Map
import map.MapLayer
import map.MapLayerSync
import map.EnvMapLayer
import graph.Node
import graph.GraphNode
import graph.Edge
import graph.NavGraph
import graph.NodeStatusSelector
import itinerary.Itineraries
import RosManager
import strip.StripContainer
import SafetyPilot
import Vehicule
import task.TaskLayer
import trap.TrapLayer
import trap.TrapStatusSelector
import site.SiteLayer
import menu.UpperLeftMenu
import menu.RightPannel


_native_code_
%{
  #include "cpp/tiles_manager.h"
  #include "cpp/coords-utils.h"
  #include <iostream>
  #include <string>

  #ifndef NO_ROS
  #include "rclcpp/rclcpp.hpp"
  #endif


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
  
  // Use static data model for debug
  int is_debug = 1
  // Use "#ifndef NO_ROS" instead


  double init_lat = get_arg_double (argc, argv, 1)
  if (init_lat == -1) {
    //Esperces data
    //init_lat = 43.315313261816485
    //Caylus data 
    //init_lat = 44.27432196595285
    //Beynes data
    init_lat = 48.86109526727752
  }
  double init_lon = get_arg_double (argc, argv, 2)
  if (init_lon == -1){
    //Esperces data
    //init_lon = 1.404974527891014
    //Caylus data
    //init_lon = 1.729783361205679
    //Baynes data
    init_lon = 1.8933138875646296
  }
  //maximum lvl of zoom of 19
  int init_zoom = 19


  // Width: 1424 + 600 = 2024 why ?
  int init_frame_width = 2048
  // Hieght: 868 + 600 = 1468 why ?
  int init_frame_height = 1280

  Frame f ("CoHoMa", 0, 0, init_frame_width, init_frame_height)
  mouseTracking = 1
  f.background_color.r = 50
  f.background_color.g = 50
  f.background_color.b = 50

  Exit quit (0, 1)
  f.close->quit

  LogPrinter lp ("Main (debug): ")

  Spike press_on_background
  Spike show_reticule
  Spike hide_reticule

  // Load only once SVG files
	svg_vab = loadFromXML ("res/svg/vab.svg")
  svg_robot = loadFromXML ("res/svg/robot.svg")
  svg_drone = loadFromXML ("res/svg/drone.svg")
  svg_safety_pilot = loadFromXML("res/svg/safety_pilot.svg")

  CohomaContext context (f, init_lat, init_lon, init_zoom)
  /*context.ctrl -> {
    "key CONTROL" =: lp.input
  }
  context.shift -> {
    "key SHIFT" =: lp.input
  }
  //context.space -> {
  //  "key SPACE" =: lp.input
  //}
  context.del -> {
    "key DEL" =: lp.input
  }*/

  ModelManager model_manager (context, is_debug)

  //Create one layer per data.
  // from bottom to top :
  //  - geoportail tiles
  //  - OSM tiles
  //  - GraphNode + Navigation graphs 
  //  - Vehicules TODO
  //  - Traps  TODO
  //  - Zones  TODO

  Component l {
    Map map (f, 0, 0, init_frame_width - $context.RIGHT_PANEL_WIDTH, init_frame_height - $context.STRIP_HEIGHT, init_lat, init_lon, init_zoom)
    // FIXME: map crash if I add dynamic width/height:
    //f.width - context.RIGHT_PANEL_WIDTH =:> map.width
    //f.height - context.STRIP_HEIGHT =:> map.height
    init_frame_width - context.RIGHT_PANEL_WIDTH =: map.width
    init_frame_height - context.STRIP_HEIGHT =: map.height

    map.g_map.pz.press_trigger -> press_on_background
    map.zoom =:> context.map_scale
    map.xpan - map.cur_ref_x + map.px0 =:> context.map_translation_x
    map.ypan - map.cur_ref_y + map.py0 =:> context.map_translation_y

    Component geoportail {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          //MapLayer layer (f, map, load_geoportail_tile, "geoportail")
          MapLayerSync layer (f, map, load_geoportail_tile, "geoportail")
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      String name ("Geoportail")
    }
    /*Component osm {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          MapLayer layer (f, map, load_osm_tile, "osm")
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      String name ("OSM")
    }*/

    Component satelites {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible { //using Layer prevents some animations to work (TODO Stephane)

          Scaling sc (1, 1, 0, 0)
          map.zoom =:> sc.sx, sc.sy

          Translation pos (0, 0)
          map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
          map.ypan - map.cur_ref_y + map.py0 =:> pos.ty

          List layers {
            Vehicule vab (map, context, model_manager.vehicles.vab, svg_vab)
            Vehicule agilex1 (map, context, model_manager.vehicles.agilex1, svg_robot)
            Vehicule agilex2 (map, context, model_manager.vehicles.agilex2, svg_robot)
            Vehicule lynx (map, context, model_manager.vehicles.lynx, svg_robot)
            Vehicule spot (map, context, model_manager.vehicles.spot, svg_robot)
            Vehicule drone (map, context, model_manager.vehicles.drone, svg_drone)
          }
        }
      }
      String name ("Satelites")
    }

    Component navgraph {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          NavGraph layer (map, context, f, model_manager)
        }
      }
      nodes aka ctrl_visibility.visible.layer.nodes
      shadow_edges aka ctrl_visibility.visible.layer.shadow_edges
      //itinerary_edges aka ctrl_visibility.visible.layer.itinerary_edges
      edges aka ctrl_visibility.visible.layer.edges

      String name ("Navgraph")
    }

    Component itineraries {
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible{
          Itineraries layer (map)
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
          TrapLayer layer (map, context, model_manager)
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

    Component result{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible  {
          EnvMapLayer layer (map)
        }
      }
      String name("Result")
      georef_visibility_map aka ctrl_visibility.visible.layer.georef_visibility_map
      visibility_map aka ctrl_visibility.visible.layer.visibility_map
      visibility_map_resolution aka ctrl_visibility.visible.layer.visibility_map_resolution
    }

    Component actors{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          Scaling sc (1, 1, 0, 0)
          map.zoom =:> sc.sx, sc.sy

          Translation pos (0, 0)
          map.xpan - map.cur_ref_x + map.px0 =:> pos.tx
          map.ypan - map.cur_ref_y + map.py0 =:> pos.ty

          SafetyPilot drone_safety_pilot (map, context, model_manager.safety_pilots.drone_safety_pilot, svg_safety_pilot)
          SafetyPilot ground_safety_pilot (map, context, model_manager.safety_pilots.ground_safety_pilot, svg_safety_pilot)
        }
      }
      String name("Actors")
    }

    Component site{
      Switch ctrl_visibility(visible){
        Component hidden
        Component visible {
          SiteLayer layer (map, context, model_manager)
        }
      }
      String name("Site")
      sitelayer aka ctrl_visibility.visible.layer
    }

    Component allocated_tasks{
      Switch ctrl_visibility(visible){
        Component hidden
        Component visible{
          TaskLayer layer (map)
        }
      }
      String name("Allocation")
      allocated_tasks_layer aka ctrl_visibility.visible.layer
    }
     
    
    addChildrenTo map.layers {
      geoportail,
      //osm,
      result,
      site, // exclusion areas + limas
      navgraph,
      itineraries,
      satelites,
      traps, 
      tasks,
      actors,
      allocated_tasks
    }
  }


  // Foreground (absolute position in frame)
  Component foreground {
    Spike edit
    Spike create

    FSM fsm_mode {
      State mode_wp_edit {
        NodeStatusSelector node_menu (f, context)
        //f.move.x =:> node_menu.x
        //f.move.y =:> node_menu.y
      }
      State mode_wp_create

      mode_wp_create->mode_wp_edit (edit)
      mode_wp_edit->mode_wp_create (create)
    }

    TrapStatusSelector trap_menu (f, context)
    press_on_background -> trap_menu.close
  }

  show_reticule -> l.map.reticule.show_reticule, foreground.create
  hide_reticule -> l.map.reticule.hide_reticule, foreground.edit

  // ROS manager
  RosManager ros_manager(root, l.map, context, model_manager)
  
  // Right panel
  RightPannel right_pannel (context, model_manager, f, ros_manager.node, root.l.map.layers.itineraries)

  // Ros node w/ all sub and pub fonctions
  right_pannel.plan_request -> ros_manager.plan_request
  right_pannel.validate_plan -> ros_manager.validate_plan
  right_pannel.update_graph -> ros_manager.update_graph
  right_pannel.test_multiple_itineraries_spike -> ros_manager.test_multiple_itineraries_spike
  right_pannel.test_allocation_spike -> ros_manager.test_allocation_spike
  right_pannel.itineraryPannel.plan_set ->ros_manager.validate_plan
  right_pannel.test_lima_spike -> ros_manager.test_lima_spike
  right_pannel.send_selected_tasks -> ros_manager.send_selected_tasks
  right_pannel.test_visibility_map -> ros_manager.test_visibility_map
  
  // Strips container
  StripContainer strips (context, model_manager, f)

  UpperLeftMenu menu (l.map, f)


  // Add Graph Node FSM
  Spike addWptToLayer

  FSM addNode {
    State idle 

    State preview {
      Scaling sc (1, 1, 0, 0)
      l.map.zoom =:> sc.sx, sc.sy

      Translation pos (0, 0)
      l.map.xpan - l.map.cur_ref_x + l.map.px0 =:> pos.tx
      l.map.ypan - l.map.cur_ref_y + l.map.py0 =:> pos.ty

      // Init Temporary with an id to -1
      GraphNode temporary (l.map, context, -1, 0, 0)
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon

      f.release -> addWptToLayer
    }
    idle -> preview (context.ctrl, show_reticule)
    preview -> idle (context.ctrl_r, hide_reticule)
  }

  addWptToLayer -> (root) {
    nodes = find(root.l.map.layers.navgraph.nodes)
    addChildrenTo nodes {
      Node new (root.l.map, root.context, $root.l.map.pointer_lat, $root.l.map.pointer_lon, 0, 0, "by_operator", 0)
    }
    //print (nodes.size)
    //node = find(nodes[$nodes.size])
    //node.wpt.lat = root.addNode.preview.temporary.lat
    nodes[$nodes.size].wpt.lat = root.addNode.preview.temporary.lat
    nodes[$nodes.size].wpt.lon = root.addNode.preview.temporary.lon
    nodes[$nodes.size].id = nodes.size   
 }


  // Add Edge between GraphNode 
  Spike clear_temp_list
  Spike add_edges
  Spike add_first_wpt
  Spike clear_all
  Spike disable_drag
  Spike renable_drag

  context.del -> clear_all
  

  Ref null_ref (0)
  RefProperty current_addedge_node (nullptr)
  DerefDouble ddx (current_addedge_node, "wpt/screen_translation/tx", DJNN_GET_ON_CHANGE)
  DerefDouble ddy (current_addedge_node, "wpt/screen_translation/ty", DJNN_GET_ON_CHANGE)
 

  FSM addEdge {
    State idle

    State shift_on

    State preview_on {
      List temp_id_list

      root.context.selected_node_id -> (root){
        addChildrenTo root.addEdge.preview_on.temp_id_list {
          Int _ ($root.context.selected_node_id)
        }

        int size = $root.addEdge.preview_on.temp_id_list.size 
        int src = $root.addEdge.preview_on.temp_id_list.[size - 1]
        int dest = $root.addEdge.preview_on.temp_id_list.[size]
        addChildrenTo root.l.map.layers.navgraph.shadow_edges {
          Edge _ (src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
        }
      }
      
      NoFill _
      OutlineOpacity _ (0.5)
      OutlineWidth _ (5)
      OutlineColor _ (234, 234, 234)

      Scaling sc (1, 1, 0, 0)
      l.map.zoom =:> sc.sx, sc.sy

      Translation pos (0, 0)
      l.map.xpan - l.map.cur_ref_x + l.map.px0 =:> pos.tx
      l.map.ypan - l.map.cur_ref_y + l.map.py0 =:> pos.ty
    
      Line temp_shadow_edge (0, 0, 0, 0)

      Int index (1)
      index->(root) {
        setRef (root.current_addedge_node, root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index])
      }
      context.selected_node_id =:> index

      //index =:> lp.input

      index -> (root){
        root.addEdge.preview_on.temp_shadow_edge.x1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.screen_translation.tx
        root.addEdge.preview_on.temp_shadow_edge.y1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.screen_translation.ty
      }
      ddx.value =:> temp_shadow_edge.x1
      ddy.value =:> temp_shadow_edge.y1

      f.move.x - pos.tx =:> temp_shadow_edge.x2
      f.move.y - pos.ty =:> temp_shadow_edge.y2
    }

    idle -> shift_on (context.shift, show_reticule)
    shift_on -> idle (context.shift_r, hide_reticule)
    shift_on -> preview_on (root.context.selected_node_id, add_first_wpt)
    preview_on -> idle (context.shift_r, add_edges) // + hide_reticule
  }
  add_edges -> hide_reticule

  clear_temp_list -> (root) {
    root.context.entered_wpt = &(root.null_ref)
    root.current_addedge_node = &(root.null_ref)

    delete_content root.l.map.layers.navgraph.shadow_edges
    delete_content root.addEdge.preview_on.temp_id_list
  }

  clear_all -> (root) {
    root.context.entered_wpt = &(root.null_ref)
    root.current_addedge_node = &(root.null_ref)
    
    delete_content root.l.map.layers.navgraph.edges
    delete_content root.l.map.layers.navgraph.shadow_edges
    delete_content root.l.map.layers.navgraph.nodes
  }

  add_first_wpt -> (root){
    root.current_addedge_node = &(root.null_ref)
    addChildrenTo root.addEdge.preview_on.temp_id_list{
      Int _($root.context.selected_node_id)
    }
  }


  add_edges -> na_add_edges:(root){
    //print ("add_edges: " + root.addEdge.preview_on.temp_id_list.size + "\n")
    for (int i = 1; i < $root.addEdge.preview_on.temp_id_list.size; i++) {
      int src = $root.addEdge.preview_on.temp_id_list.[i]
      int dest = $root.addEdge.preview_on.temp_id_list.[i+1]
      //print ("Add edge\n")
      addChildrenTo root.l.map.layers.navgraph.edges {
        Edge _(src, dest, 22.11618714809018, root.l.map.layers.navgraph.nodes)
     }
    }
  }
  na_add_edges -> clear_temp_list

}