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

import map.Map
import map.MapLayer
import map.EnvMapLayer
import graph.Node
import graph.NavGraph
import graph.NodeStatusSelector
import itinerary.Itineraries
import RosManager
import strip.StripContainer
import SafetyPilot
import Vehicule
import task.TasksLayer
import trap.TrapsLayer
import trap.TrapStatusSelector
import site.SiteLayer
import menu.UpperLeftMenu
import menu.RightPanel


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

  // Load only once SVG files
	svg_vab = load_from_XML_once ("res/svg/vab.svg")
  svg_robot = load_from_XML_once ("res/svg/robot.svg")
  svg_drone = load_from_XML_once ("res/svg/drone.svg")
  svg_safety_pilot = load_from_XML_once("res/svg/safety_pilot.svg")

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
    map.zoom =:> context.map_scale
    map.xpan - map.cur_ref_x + map.px0 =:> context.map_translation_x
    map.ypan - map.cur_ref_y + map.py0 =:> context.map_translation_y

    // Geoportail tiles
    Component geoportail {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          //MapLayer layer (f, map, "geoportail", "http://proxy.recherche.enac.fr:3128") // geoportail may need proxy - using https
          MapLayer layer (f, map, map_provider, proxy)
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      String name ("Geoportail")
    }

    // OSM tiles
    /*Component osm {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          //MapLayer layer (f, map, "osm", "") // osm do not need proxy - using http
          MapLayer layer (f, map, map_provider, proxy)
        }
      }
      opacity aka ctrl_visibility.visible.layer.opacity
      String name ("OSM")
    }*/


    // ----------------------------------------------------
    //  SATELITE = VEHICLE = VAB + UGV + UAV
    Component satelites {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible { //using Layer prevents some animations to work (TODO Stephane)

          Scaling sc (1, 1, 0, 0)
          context.map_scale =:> sc.sx, sc.sy

          Translation pos (0, 0)
          context.map_translation_x =:> pos.tx
          context.map_translation_y =:> pos.ty

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


    // ----------------------------------------------------
    //  Navigation GRAPH
    Component navgraph {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible {
          NavGraph layer (map, context, model_manager)
        }
      }
      String name ("Navgraph")
    }


    // ----------------------------------------------------
    //  ITINERARY
    Component itineraries {
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible{
          Itineraries layer (map, context, model_manager)
        }
      }
      String name ("Itineraries")
    }


    // ----------------------------------------------------
    //  TRAP
    Component traps{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          TrapsLayer layer (map, context, model_manager)
        }
      }
      String name("Traps")
    }


    // ----------------------------------------------------
    //  TASK
    Component tasks{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          TasksLayer layer (map, context, model_manager)
        }
      }
      String name("Tasks")
    }


    // ----------------------------------------------------
    //  Result of the map exploration by drones and robots
    Component result{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible  {
          EnvMapLayer layer (map, context)
        }
      }
      String name("Result")
      result_layer aka ctrl_visibility.visible.layer
      visibility_map aka ctrl_visibility.visible.layer.visibility_map
    }


    // ----------------------------------------------------
    //  ACTOR = Safety Pilot
    Component actors{
      Switch ctrl_visibility (visible){
        Component hidden
        Component visible {
          Scaling sc (1, 1, 0, 0)
          context.map_scale =:> sc.sx, sc.sy

          Translation pos (0, 0)
          context.map_translation_x =:> pos.tx
          context.map_translation_y =:> pos.ty

          SafetyPilot drone_safety_pilot (map, context, model_manager.safety_pilots.drone_safety_pilot, svg_safety_pilot)
          SafetyPilot ground_safety_pilot (map, context, model_manager.safety_pilots.ground_safety_pilot, svg_safety_pilot)
        }
      }
      String name("Actors")
    }


    // ----------------------------------------------------
    //  SITE = exclusion areas + limas
    Component site{
      Switch ctrl_visibility(visible){
        Component hidden
        Component visible {
          SiteLayer layer (map, context, model_manager)
        }
      }
      String name("Site")
    }


    // ----------------------------------------------------
    //  Allocated TASK
    Component allocated_tasks{
      Switch ctrl_visibility(visible){
        Component hidden
        Component visible{
          TasksLayer layer (map, context, model_manager)
        }
      }
      String name("Allocation")
    }
    
    
    // Add layers, from bottom to top:
    addChildrenTo map.layers {
      geoportail,
      //osm,
      result,
      site,
      navgraph,
      itineraries,
      satelites,
      traps, 
      tasks,
      actors,
      allocated_tasks
    }
  }


  // ----------------------------------------------------
  // Foreground (absolute position in frame)
  Component foreground {
    Spike edit
    Spike create

    Scaling sc (1, 1, 0, 0)
    context.map_scale =:> sc.sx, sc.sy

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
  right_panel.plan_request -> ros_manager.plan_request
  right_panel.update_graph -> ros_manager.update_graph
  right_panel.itinerary_panel.set_plan -> ros_manager.validate_plan
  right_panel.send_selected_tasks -> ros_manager.send_selected_tasks
  
	right_panel.plan_request -> model_manager.clear_itineraries

  // FIXME: We can't clear the model first, because views are coupled to these models and it generates errors in std-out:
  // WARNING: 0  -  CouplingProcess::~CouplingProcess - noname
  // WARNING: 0  -  CoreProcess::~CoreProcess - noname - _vertex is NOT NULL and it should
  //right_panel.itinerary_panel.set_plan -> model_manager.clear_tasks

  // --> Clear the view first
  right_panel.itinerary_panel.set_plan -> root.l.map.layers.tasks.ctrl_visibility.visible.layer.clear
	
  // ----------------------------------------------------
  // Strips container
  StripContainer strips (context, model_manager, f)

  // ----------------------------------------------------
  // Menu to show/hide layers
  UpperLeftMenu menu (l.map, f)

  // ----------------------------------------------------
  // FSM to manage the addition of node in the graph
  FSM fsm_add_node {
    State idle 

    State preview {
      Scaling sc (1, 1, 0, 0)
      context.map_scale =:> sc.sx, sc.sy

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
          Int _ ($root.context.id_node_graph_edition.value)
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
      
      Scaling sc (1, 1, 0, 0)
      context.map_scale =:> sc.sx, sc.sy

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
      Int _ ($root.context.id_node_graph_edition.value)
    }
  }

  stop_create_edges -> na_stop_create_edges:(root) {
    delete_content root.fsm_add_edge.preview_on.temp_id_list
  }

}