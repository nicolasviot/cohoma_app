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
use animation
use display

import CohomaContext
import model.ModelManager
import model.NoRosModelManager
import model.EdgeModel

import SubLayer
import map.Map
import map.MapLayer
import SubLayerVisibilityMap
import graph.SubLayerNavigGraph
import graph.Node
import graph.NodeStatusSelector
import itinerary.SubLayerItineraries
import RosManager
import strip.StripContainer
import movable.SubLayerVehicles
import movable.SubLayerSafetyPilots
import task.SubLayerTasks
import trap.SubLayerTraps
import trap.TrapStatusSelector
import site.SubLayerSite
import menu.UpperLeftMenu
import menu.RightPanel
import ClockComponent


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

  void init_args (int argc, char * argv[])
  {
	  char* lat = getCmdOption(argv, argv + argc, "-lat");
    if (lat) {
        init_latitude = std::strtod (lat, nullptr);
    }
    else {
        init_latitude = 48.86109526727752;
        cout << "using default latitude '" << init_latitude << "' of Beynes" << endl;
    }

    char* lon = getCmdOption(argv, argv + argc, "-lon");
    if (lon) {
        init_longitude = std::strtod (lon, nullptr);
    }
    else {
        init_longitude = 1.8933138875646296;
        cout << "using default longitude '" << init_longitude << "' of Beynes" << endl;
    }

    char* map = getCmdOption(argv, argv + argc, "-m");
    if (map) {
        map_provider = string (map);
    }
    else {
        map_provider = "geoportail";
        cout << "using default map provider: '" << map_provider << "'" << endl;
    }

    char* p = getCmdOption(argv, argv + argc, "-p");
    if (p) {
        proxy = string (p);
    }

    if (proxy == "") {
      cout << "Run COHOMA with map provider '" << map_provider << "' at " << init_latitude << " " << init_longitude << " (without proxy)..." << endl;
    }
    else {
      cout << "Run COHOMA with map provider '" << map_provider << "' at " << init_latitude << " " << init_longitude << " and proxy '" << proxy << "'..." << endl;
    }
  }
%}


_main_
Component root {

  _DEBUG_SEE_COLOR_PICKING_VIEW = 0

  init_ros ()

  init_args (argc, argv)
  

  // Use static data model for debug
  int is_debug = 1
  // Use "#ifndef NO_ROS" instead

  // Maximum level of zoom of 19
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


  CohomaContext context (f, init_latitude, init_longitude, init_zoom)
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

  if (is_debug) {
    NoRosModelManager model (context, is_debug)
  }
  else {
    ModelManager model (context, is_debug)
  }
  model_manager = find(root.model)


  // Create one layer per data, from bottom to top:
  Component l {

    // ----------------------------------------------------
    //  MAP
    Map map (f, 0, 0, init_frame_width - $context.RIGHT_PANEL_WIDTH, init_frame_height - $context.STRIP_HEIGHT, init_latitude, init_longitude, init_zoom)
    // FIXME: map crash if I add dynamic width/height:
    //f.width - context.RIGHT_PANEL_WIDTH =:> map.width
    //f.height - context.STRIP_HEIGHT =:> map.height
    init_frame_width - context.RIGHT_PANEL_WIDTH =: map.width
    init_frame_height - context.STRIP_HEIGHT =: map.height

    map.g_map.pz.press_trigger -> press_on_background
    map.zoomLevel =:> context.map_zoom
    map.xpan - map.cur_ref_x + map.px0 =:> context.map_translation_x
    map.ypan - map.cur_ref_y + map.py0 =:> context.map_translation_y

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
    //  TRAP
    SubLayerTraps traps (model_manager.layers.[6], map, context, model_manager)


    // ----------------------------------------------------
    //  TASK
    SubLayerTasks tasks (model_manager.layers.[7], map, context, model_manager)


    // ----------------------------------------------------
    //  Allocated Tasks
    SubLayerTasks allocated_tasks (model_manager.layers.[8], map, context, model_manager)


    // ----------------------------------------------------
    //  Safety Pilots
    SubLayerSafetyPilots safety_pilots (model_manager.layers.[9], map, context, model_manager)


    // ----------------------------------------------------
    //  VEHICLE = VAB + SATELLITEs (UGV + UAV)
    SubLayerVehicles vehicles (model_manager.layers.[10], map, context, model_manager)
    

    // Add layers, from bottom to top:
    addChildrenTo map.layers {
      sub_layer_map,
      //sub_layer_geoportail,
      //sub_layer_osm,
      visibility_map,
      site,
      navigation_graph,
      itineraries,
      traps, 
      tasks,
      allocated_tasks,
      safety_pilots, //actors,
      vehicles //satellites
    }
  }


  // ----------------------------------------------------
  // Foreground (absolute position in frame)
  Component foreground {
    Spike edit
    Spike create

    Translation pos (0, 0)
    context.map_translation_x =:> pos.tx
    context.map_translation_y =:> pos.ty

    FSM fsm_mode {
      State mode_edit_node {
        NodeStatusSelector node_menu (f, context)
        press_on_background -> context.set_node_status_edition_to_null
      }
      State mode_create_node

      mode_create_node -> mode_edit_node (edit)
      mode_edit_node -> mode_create_node (create)
    }

    TrapStatusSelector trap_menu (f, context)
    press_on_background -> context.set_current_trap_to_null
  }

  show_reticule -> l.map.reticule.show_reticule, foreground.create
  hide_reticule -> l.map.reticule.hide_reticule, foreground.edit


  // ----------------------------------------------------
  // ROS manager
  RosManager ros_manager (root, l.map, context, model_manager)
  
  // Right panel
  RightPanel right_panel (context, model_manager, f, ros_manager.node)

  // ----------------------------------------------------
  // Ros node w/ all sub and pub fonctions
  right_panel.layer.plan_request -> ros_manager.plan_request
  right_panel.layer.update_graph -> ros_manager.update_graph
  right_panel.layer.itinerary_panel.set_plan -> ros_manager.validate_plan
  right_panel.layer.send_selected_tasks -> ros_manager.send_selected_tasks
  
	right_panel.layer.plan_request -> model_manager.clear_itineraries

  // FIXME: We can't clear the model first, because views are coupled to these models and it generates errors in std-out:
  // WARNING: 0  -  CouplingProcess::~CouplingProcess - noname
  // WARNING: 0  -  CoreProcess::~CoreProcess - noname - _vertex is NOT NULL and it should
  //right_panel.itinerary_panel.set_plan -> model_manager.clear_tasks

  // --> Clear the view first
  right_panel.layer.itinerary_panel.set_plan -> root.l.map.layers.tasks.clear
	
  // ----------------------------------------------------
  Component cclock {
    Translation _ (1626, 395)
    ClockComponent clock (f)
  }

  // ----------------------------------------------------
  // Strips container
  StripContainer strips (context, model_manager, f)

  // ----------------------------------------------------
  // Menu to show/hide layers
  UpperLeftMenu menu (l.map, context, model_manager, f)

  // ----------------------------------------------------
  // FSM to manage the addition of node in the graph
  FSM fsm_add_node {
    State idle 

    State preview {

      Translation pos (0, 0)
      context.map_translation_x =:> pos.tx
      context.map_translation_y =:> pos.ty

      // Temporary view uses temporary model
      Node temporary (l.map, context, model_manager.temp_node)

      // Update model
      l.map.pointer_lat =:> model_manager.temp_node.lat
      l.map.pointer_lon =:> model_manager.temp_node.lon

      f.release -> model_manager.create_node_from_temp
    }
    idle -> preview (context.ctrl, show_reticule)
    preview -> idle (context.ctrl_r, hide_reticule)
  }


  // Spikes
  Spike start_create_edges
  Spike stop_create_edges
  Spike clear_all

  context.del -> context.set_node_graph_edition_to_null
  context.del -> context.set_node_status_edition_to_null
  context.del -> clear_all
  

  // FSM to manage the addition of edges in the graph
  FSM fsm_add_edge {
    State idle

    State shift_on

    State preview_on {
      List temp_id_list

      root.context.id_node_graph_edition.value -> (root) {
        addChildrenTo root.fsm_add_edge.preview_on.temp_id_list {
          String _ (toString(root.context.id_node_graph_edition.value))
        }

        int size = $root.fsm_add_edge.preview_on.temp_id_list.size 
        string source_id = toString (root.fsm_add_edge.preview_on.temp_id_list.[size - 1])
        string target_id = toString (root.fsm_add_edge.preview_on.temp_id_list.[size])
        string edge_id = source_id + "_" + target_id

        addChildrenTo root.model.edge_ids {
          String _ (edge_id)
        }
        source = find (root.model.nodes, source_id)
        target = find (root.model.nodes, target_id)
        EdgeModel (root.model.edges, edge_id, source, target, 0.0)
      }
      
      Translation pos (0, 0)
      context.map_translation_x =:> pos.tx
      context.map_translation_y =:> pos.ty

      // UI of temporary edge
      NoFill _
      OutlineOpacity _ (0.5)
      OutlineWidth _ (5)
      //OutlineColor _ ($context.EDGE_COLOR)
      OutlineColor _ (#FFFF00)
    
      Line temp_shadow_edge (0, 0, 0, 0)


      context.dx_node_graph_edition.value =:> temp_shadow_edge.x1
      context.dy_node_graph_edition.value =:> temp_shadow_edge.y1

      f.move.x - pos.tx =:> temp_shadow_edge.x2
      f.move.y - pos.ty =:> temp_shadow_edge.y2
    }

    idle -> shift_on (context.shift, show_reticule)
    shift_on -> idle (context.shift_r, hide_reticule)
    shift_on -> preview_on (root.context.id_node_graph_edition.value, start_create_edges)
    preview_on -> idle (context.shift_r, stop_create_edges) // + hide_reticule
  }
  stop_create_edges -> hide_reticule
  stop_create_edges -> context.set_node_graph_edition_to_null


  clear_all -> (root) {
    print ("Clear all\n")
    
    delete_content root.fsm_add_edge.preview_on.temp_id_list
    
    //delete_content root.l.map.layers.navgraph.edges
    delete_content root.model.edges

    //delete_content root.l.map.layers.navgraph.nodes
    delete_content root.model.nodes
  }

  start_create_edges -> (root) {
    //print ("Start create edges\n")

    // Add the id of the first selected node
    addChildrenTo root.fsm_add_edge.preview_on.temp_id_list{
      String _ (toString(root.context.id_node_graph_edition.value))
    }
  }

  stop_create_edges -> na_stop_create_edges:(root) {
    delete_content root.fsm_add_edge.preview_on.temp_id_list
  }

}