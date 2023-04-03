/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2023)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *    Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use base
use gui
use display
use files

import CohomaContext
import model.ModelManager
import model.NoRosModelManager
import model.trap.TrapDetectionModel
import model.trap.TrapModel

import panels.TopBar
import panels.UpperLeftMenu
import panels.RightPanel
import panels.PositionedItemSelector

import left_panel.LeftPanel

import SubLayer
import map.Map
import map.MapLayer
import SubLayerVisibilityMap
import graph.SubLayerNavigGraph
import graph.Node
import graph.NodeStatusSelector
import graph.CGraph
import itinerary.SubLayerItineraries
import RosManager
import movable.SubLayerVehicles
import movable.SubLayerSafetyPilots
import task.SubLayerTasks
import trap.SubLayerTrapDetections
import trap.SubLayerTraps
import trap.TrapStatusSelector
import trap.TrapForm
import site.SubLayerSite
import Reticule


_native_code_
%{
  #include "cpp/tiles_manager.h"
  #include "cpp/coords-utils.h"
  #include <iostream>
  #include <string>

  #ifndef NO_ROS
  #include "rclcpp/rclcpp.hpp"
  #endif

  using namespace std;


  double init_latitude = -1;
  double init_longitude = -1;
  string map_provider = "";
  string proxy = "";


  static void
  init_ros ()
  {
  #ifndef NO_ROS
    rclcpp::init(0,0); //argc, argv);
  #endif
  }

  char* getCmdOption(char ** begin, char ** end, const std::string & option)
  {
      char ** itr = std::find(begin, end, option);
      if (itr != end && ++itr != end)
      {
          return *itr;
      }
      return 0;
  }

  bool cmdOptionExists(char** begin, char** end, const std::string& option)
  {
      return std::find(begin, end, option) != end;
  }

  // pseudo /dev/null for iostream
  class toto {}; template <typename X> toto& operator<<(toto& t, const X&) { return t; }
  int myendl=0;

  void init_args (int argc, char * argv[])
  {
	  char* lat = getCmdOption(argv, argv + argc, "-lat");
    //auto& out = cout;
    //auto& myendl = endl;
    toto out;
    if (lat) {
        init_latitude = std::strtod (lat, nullptr);
    }
    else {
        init_latitude = 48.86109526727752;
        out << "using default latitude '" << init_latitude << "' of Beynes" << myendl;
    }

    char* lon = getCmdOption(argv, argv + argc, "-lon");
    if (lon) {
        init_longitude = std::strtod (lon, nullptr);
    }
    else {
        init_longitude = 1.8933138875646296;
        out << "using default longitude '" << init_longitude << "' of Beynes" << myendl;
    }

    char* map = getCmdOption(argv, argv + argc, "-m");
    if (map) {
        map_provider = string (map);
    }
    else {
        map_provider = "geoportail";
        out << "using default map provider: '" << map_provider << "'" << myendl;
    }

    char* p = getCmdOption(argv, argv + argc, "-p");
    if (p) {
        proxy = string (p);
    }

    if (proxy == "") {
      out << "Run COHOMA with map provider '" << map_provider << "' at " << init_latitude << " " << init_longitude << " (without proxy)..." << myendl;
    }
    else {
      out << "Run COHOMA with map provider '" << map_provider << "' at " << init_latitude << " " << init_longitude << " and proxy '" << proxy << "'..." << myendl;
    }
  }
%}


_main_
Component root {

  // _DEBUG
  _DEBUG_SEE_COLOR_PICKING_VIEW = 0
  _DEBUG_SEE_RECOMPUTE_PIXMAP_ONLY = 0

  // CONFIG
  _DEBUG_NO_TOUCH_EVENT = 1

  init_ros ()

  init_args (argc, argv)
  

  // Use static data model for debug
  int is_debug = 1
  // Use "#ifndef NO_ROS" instead

  // Maximum level of zoom of 19
  int init_zoom = 19


  int init_frame_width = 1920
  int init_frame_height = 1080

  Frame f ("CoHoMa", 0, 0, init_frame_width, init_frame_height)
  mouseTracking = 1
  f.background_color.r = 50
  f.background_color.g = 50
  f.background_color.b = 50

  Exit quit (0, 1)
  f.close->quit


  Spike left_press_on_map_bg
  Spike right_press_on_map_bg
  Spike show_reticule
  Spike hide_reticule

  LogPrinter lp ("Main (debug): ")

  WallClock wc_file_name

  // FIXME: do we need 2 different log files ?
  FileWriter fw ("logs/Log_" + wc_file_name.state_text + ".log")
  FileWriter fw_console ("logs/Log_console_" + wc_file_name.state_text + ".log")

  // To centralize the current context
  CohomaContext context (f, init_latitude, init_longitude, init_zoom)

  context.w_clock.state_text + " - Application start\n" =: fw.input
  "Click at (" + f.press.x + ", " + f.press.y + ")\n" => fw.input

  context.w_clock.state_text + " - Application start\n" =: fw_console.input

  //left_press_on_map_bg -> context.set_selected_trap_to_null
  left_press_on_map_bg -> context.na_unselect_trap

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

  // Data model manager
  if (is_debug) {
    NoRosModelManager model (context, is_debug)
  }
  else {
    ModelManager model (context, is_debug)
  }
  model_manager = find(root.model)


    // ----------------------------------------------------
    //  MAP
    Map map (f, 0, 0, init_frame_width - $context.RIGHT_PANEL_WIDTH, init_frame_height, init_latitude, init_longitude, init_zoom)
    //Map map (f, $context.LEFT_PANEL_WIDTH, $context.TOP_BAR_HEIGHT, init_frame_width - $context.LEFT_PANEL_WIDTH - $context.RIGHT_PANEL_WIDTH, init_frame_height - $context.TOP_BAR_HEIGHT, init_latitude, init_longitude, init_zoom)
    
    // FIXME: map crash if I add dynamic width/height:
    //f.width - context.LEFT_PANEL_WIDTH - context.RIGHT_PANEL_WIDTH =:> map.width
    //f.height - context.TOP_BAR_HEIGHT =:> map.height
    
    // Don't take into account LEFT_PANEL_WIDTH for x / width
    //init_frame_width - context.LEFT_PANEL_WIDTH - context.RIGHT_PANEL_WIDTH =: map.width
    init_frame_width - context.RIGHT_PANEL_WIDTH =: map.width
    // Don't take into account TOP_BAR_HEIGHT for y / height
    //init_frame_height - context.TOP_BAR_HEIGHT =: map.height

    map.g_map.pz.left_press_trigger -> left_press_on_map_bg
    map.g_map.pz.right_press_trigger -> right_press_on_map_bg
    map.zoomLevel =:> context.map_zoom
    //map.xpan - map.cur_ref_x + map.px0 =:> context.map_translation_x
    //map.ypan - map.cur_ref_y + map.py0 =:> context.map_translation_y
    map.real_xpan =:> context.map_translation_x
    map.real_ypan =:> context.map_translation_y

    // Map provider
    SubLayer sub_layer_map (model_manager.layers.[1])
    addChildrenTo sub_layer_map.switch.true {
      MapLayer map_layer (f, map, map_provider, proxy)
    }

    /*// Geoportail tiles
    SubLayer sub_layer_geoportail (model_manager.layers.[11])
    addChildrenTo sub_layer_geoportail.switch.true {
      MapLayer map_layer_geoportail (f, map, "geoportail", "http://proxy.recherche.enac.fr:3128") // geoportail may need proxy - using https
    }*/

    // OSM tiles
    /*SubLayer sub_layer_osm (model_manager.layers.[12])
    addChildrenTo sub_layer_osm.switch.true {
      MapLayer map_layer_osm (f, map, "osm", "") // osm do not need proxy - using http
    }*/


    // ----------------------------------------------------
    //  Result of the map exploration by drones and robots
    SubLayerVisibilityMap visibility_map (model_manager.layers.[2], map, context)


    // ----------------------------------------------------
    //  SITE = Limits + Exclusion zones + Limas
    SubLayerSite site (model_manager.layers.[3], map, context, model_manager)


    // ----------------------------------------------------
    //  Navigation GRAPH = Nodes + Segments
    SubLayerNavigGraph navigation_graph (model_manager.layers.[4], map, context, model_manager)


    // ----------------------------------------------------
    //  ITINERARY
    SubLayerItineraries itineraries (model_manager.layers.[5], map, context, model_manager)


    // ----------------------------------------------------
    //  TRAP (RAW) DETECTIONS
    SubLayerTrapDetections trap_detections (model_manager.layers.[6], map, context, model_manager)

    // ----------------------------------------------------
    //  TRAP
    SubLayerTraps traps (model_manager.layers.[7], map, context, model_manager)


    // ----------------------------------------------------
    //  TASK
    SubLayerTasks tasks (model_manager.layers.[8], map, context, model_manager)


    // ----------------------------------------------------
    //  Allocated Tasks
    SubLayerTasks allocated_tasks (model_manager.layers.[9], map, context, model_manager)


    // ----------------------------------------------------
    //  Safety Pilots
    SubLayerSafetyPilots safety_pilots (model_manager.layers.[10], map, context, model_manager)


    // ----------------------------------------------------
    //  VEHICLE = VAB + SATELLITEs (UGV + UAV)
    SubLayerVehicles vehicles (model_manager.layers.[11], map, context, model_manager)

    // Add layers, from bottom to top:
    addChildrenTo map.layers {
      sub_layer_map,
      //sub_layer_geoportail,
      //sub_layer_osm,
      visibility_map,
      site,
      navigation_graph,
      itineraries,
      trap_detections,
      traps, 
      tasks,
      allocated_tasks,
      safety_pilots, //actors,
      vehicles //satellites
    }


    // Foreground (absolute position in frame)
    Component fg {
      Spike edit
      Spike create

      // Reticule
      Reticule reticule (f, context)

      FSM fsm_dragging_map_item {
        State no_drag {
          map.pointer_lat => context.pointer_lat
          map.pointer_lon => context.pointer_lon
        }
        State drag_map_item

        no_drag -> drag_map_item (context.start_drag_map_item)
        drag_map_item -> no_drag (context.stop_drag_map_item)
      }

      Translation pos (0, 0)
      context.map_translation_x =:> pos.tx
      context.map_translation_y =:> pos.ty

      FSM fsm_mode {
        State mode_edit_node {
          NodeStatusSelector node_menu (f, context)
          left_press_on_map_bg -> context.set_node_status_edition_to_null
        }
        State mode_create_node

        mode_create_node -> mode_edit_node (edit)
        mode_edit_node -> mode_create_node (create)
      }

      TrapStatusSelector trap_menu (f, context)
      left_press_on_map_bg -> context.set_current_trap_to_null

      PositionedItemSelector add_item_selector (map, context)
      left_press_on_map_bg -> add_item_selector.hide
      right_press_on_map_bg -> add_item_selector.show

      show_reticule -> reticule.show, create
      hide_reticule -> reticule.hide, edit
    }

  
  // Bar at the top of the window
  TopBar top_bar (context, f)

  // Panel on the left of the window
  LeftPanel left_panel (context, model_manager, f)

  // Menu to show/hide layers
  UpperLeftMenu menu (map, context, model_manager, f)

  // 
  CGraph graph (root, f, model_manager, map, context)

  // Form to edit the selected trap
  TrapForm trap_form (context, f)


  // ----------------------------------------------------
  // ROS manager
  RosManager ros_manager (root, map, context, model_manager)

  // FIXME: ros_manager.node passed as parameter for console (to call ros_node->save_console)
  // Panel on the right of the window
  RightPanel right_panel (context, model_manager, f, ros_manager.node)

  // ----------------------------------------------------
  // Ros node w/ all sub and pub fonctions
	right_panel.itinerary_request -> model_manager.clear_itineraries
  right_panel.itinerary_request -> ros_manager.ask_itinerary

  right_panel.update_graph -> ros_manager.update_graph
  
  right_panel.layer.itinerary_panel.set_plan -> ros_manager.validate_plan

  // FIXME: We can't clear the model first, because views are coupled to these models
  //right_panel.layer.itinerary_panel.set_plan -> model_manager.clear_tasks

  // --> Clear the view first
  right_panel.layer.itinerary_panel.set_plan -> root.map.layers.allocated_tasks.clean_only_views
  right_panel.layer.itinerary_panel.set_plan -> root.map.layers.tasks.clean_views_then_models

  right_panel.send_selected_tasks -> ros_manager.send_selected_tasks
  
  //right_panel.send_selected_tasks -> root.map.layers.allocated_tasks.clean_only_views
  //right_panel.send_selected_tasks -> root.map.layers.tasks.clean_views_then_models
	

  /*fg.add_item_selector.add_trap -> {
    fg.add_item_selector.lat =: ros_manager.lat
    fg.add_item_selector.lon =: ros_manager.lon
    "Add a new trap at " + fg.add_item_selector.lat + " " + fg.add_item_selector.lon =: lp.input
  }*/
  fg.add_item_selector.add_trap -> na_add_trap:(root) {
    print ("Add a new trap at " + root.fg.add_item_selector.lat + " " + root.fg.add_item_selector.lon + " (" + root.context.w_clock.state_text + ")\n")

    addChildrenTo root.model.traps {
      TrapModel trap (root.context, $root.model.trap_new_id, $root.fg.add_item_selector.lat, $root.fg.add_item_selector.lon, root.ros_manager.node)
    }
    root.model.traps.[root.model.traps.size].detection_time = toString(root.context.w_clock.state_text)

    // FIXME: DEBUG
    addChildrenTo root.model.trap_detections {
      TrapDetectionModel trap_detection_1 ("1_1", $root.fg.add_item_selector.lat + 0.0001, $root.fg.add_item_selector.lon)
      TrapDetectionModel trap_detection_2 ("1_2", $root.fg.add_item_selector.lat, $root.fg.add_item_selector.lon + 0.0001)
      TrapDetectionModel trap_detection_3 ("1_3", $root.fg.add_item_selector.lat + 0.0001, $root.fg.add_item_selector.lon + 0.0001)
    }
    
    root.model.trap_new_id = root.model.trap_new_id + 1

    //notify root.ros_manager.send_add_trap
  }

  fg.add_item_selector.add_message -> {
    //fg.add_item_selector.lat =: ros_manager.lat
    //fg.add_item_selector.lon =: ros_manager.lon
    "Add a new message at " + fg.add_item_selector.lat + " " + fg.add_item_selector.lon =: lp.input
  }

  fg.add_item_selector.add_goto -> {
    //fg.add_item_selector.lat =: ros_manager.lat
    //fg.add_item_selector.lon =: ros_manager.lon
    "Add a go to at " + fg.add_item_selector.lat + " " + fg.add_item_selector.lon =: lp.input
  }

}