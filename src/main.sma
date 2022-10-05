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

import model.VehiculeModel
import map.Map
import map.MapLayer
import map.MapLayerSync
import map.EnvMapLayer
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
import TrapLayer
import SiteLayer


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
  int init_width = 1424
  int init_height = 868
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


  Frame f ("CoHoMa", 0, 0, init_width + 600, init_height + 600)
  Exit quit (0, 1)
  f.close->quit
  mouseTracking = 1
  f.background_color.r = 50
  f.background_color.g = 50
  f.background_color.b = 50

  Spike show_reticule
  Spike hide_reticule

  //COLORS FOR VEHICULES -- blues variations
  //Int vabCOL (#6BC0FF)
  //Int agiCOL (#ADE2ED)
  //Int agiCOL2 (#51D5F0)
  //Int lynxCOL (#5C64FF)
  //Int spotCOL (#ABBFEB)
  //Int droneCOL (#5EFFF1)
  
  //flashy
  Int vabCOL (#00B1E6)
  Int agiCOL (#0C2EE8)
  Int agiCOL2 (#B500FF)
  Int lynxCOL (#B3B100)
  Int spotCOL (#0CE820)
  Int droneCOL (#1ACAFF)

  // Load only once SVG files
	svg_vab = loadFromXML ("res/svg/vab.svg")
  svg_robot = loadFromXML ("res/svg/robot.svg")
  svg_drone = loadFromXML ("res/svg/drone.svg")
  svg_safety_pilot = loadFromXML("res/svg/safety_pilot.svg")


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

    Component model {
      Component vehicules {
        VehiculeModel vab (map, "vab", "VAB", init_lat, init_lon, $vabCOL)
        VehiculeModel agilex1 (map, "agilex1", "AGILEX 1", init_lat + 0.0005, init_lon, $agiCOL)
        VehiculeModel agilex2 (map, "agilex2", "AGILEX 2", init_lat + 0.001, init_lon, $agiCOL2)
        VehiculeModel lynx (map, "lynx", "LYNX", init_lat, init_lon + 0.001, $lynxCOL)
        VehiculeModel spot (map, "spot", "SPOT", init_lat + 0.001 , init_lon + 0.001, $spotCOL)
        VehiculeModel drone (map, "drone", "DRONE", init_lat + 0.0015 , init_lon + 0.0015, $droneCOL)
      }
    }

    Component satelites {
      Switch ctrl_visibility (visible) {
        Component hidden
        Component visible { //using Layer prevents some animations to work (TODO Stephane)
          List layers {
            Vehicule vab (map, init_lat, init_lon, "vab", $vabCOL, svg_vab)
            Vehicule agilex1 (map, init_lat + 0.0005, init_lon, "agilex1", $agiCOL, svg_robot)
            Vehicule agilex2 (map, init_lat + 0.001, init_lon, "agilex2", $agiCOL2, svg_robot)
            Vehicule lynx (map, init_lat, init_lon + 0.001, "lynx", $lynxCOL, svg_robot)
            Vehicule spot (map, init_lat + 0.001 , init_lon + 0.001, "spot", $spotCOL, svg_robot)
            Vehicule drone (map, init_lat + 0.0015 , init_lon + 0.0015, "drone", $droneCOL, svg_drone)
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
          TrapLayer layer (map)
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
            SafetyPilot sfty_pilot_uav (map, init_lat, init_lon - 0.005, 0, "UAV", svg_safety_pilot)
            SafetyPilot sfty_pilot_ugv (map, init_lat, init_lon + 0.005, 0, "UGV", svg_safety_pilot)
        }
      }
      String name("Actors")
      sfty_pilot_uav aka ctrl_visibility.visible.sfty_pilot_uav
      sfty_pilot_ugv aka ctrl_visibility.visible.sfty_pilot_ugv
    }
    Component site{
      Switch ctrl_visibility(visible){
        Component hidden
        Component visible {
          SiteLayer layer (map)
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
      site,
      //mission_zones,
      satelites,
      navgraph,
      itineraries,
      traps, 
      tasks,
      actors,
      allocated_tasks
      //lima,
    }
  }


  show_reticule -> l.map.reticule.show_reticule, l.map.layers.navgraph.ctrl_visibility.visible.layer.create
  hide_reticule -> l.map.reticule.hide_reticule, l.map.layers.navgraph.ctrl_visibility.visible.layer.edit
  RosManager ros_manager(root, l.map, l.map.layers.navgraph.manager)
  
  Component right_pannel {
    Translation t (1424, 0)
    Rectangle bg (0, 0, 700, init_height + 600)

    Switch ctrl_tab(plan){
      Component plan{

        RightPannel right_pannel (root, f, ros_manager.node)

      }
      Component supervise{
      
      }
    }
    right_pannel aka ctrl_tab.plan.right_pannel


   
  }

  // Ros node w/ all sub and pub fonctions
  right_pannel.right_pannel.plan_request -> ros_manager.plan_request
  right_pannel.right_pannel.validate_plan -> ros_manager.validate_plan
  right_pannel.right_pannel.update_graph -> ros_manager.update_graph
  right_pannel.right_pannel.test_multiple_itineraries_spike -> ros_manager.test_multiple_itineraries_spike
  right_pannel.right_pannel.test_allocation_spike -> ros_manager.test_allocation_spike
  right_pannel.right_pannel.itineraryPannel.plan_set ->ros_manager.validate_plan
  right_pannel.right_pannel.test_lima_spike -> ros_manager.test_lima_spike
  right_pannel.right_pannel.send_selected_tasks -> ros_manager.send_selected_tasks
  right_pannel.right_pannel.test_visibility_map -> ros_manager.test_visibility_map
  
  // Strips container
  StripContainer strips (f, 0, 868, l.map.layers.satelites )

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

  addWptToLayer -> (root) {
    nodes = find(root.l.map.layers.navgraph.nodes)
    addChildrenTo nodes {
      Node new (root.l.map, root.f, $root.l.map.pointer_lat, $root.l.map.pointer_lon, 0, 0, "by_operator", 0, root.l.map.layers.navgraph.manager)
    }
    //print (nodes.size)
    //node = find(nodes[$nodes.size])
    //node.wpt.lat = root.addNode.preview.temporary.lat
    nodes[$nodes.size].wpt.lat = root.addNode.preview.temporary.lat
    nodes[$nodes.size].wpt.lon = root.addNode.preview.temporary.lon
    nodes[$nodes.size].id = nodes.size    
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
  //LogPrinter lp ("debug selected_id dans le main")
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
   /*   GraphNode temporary (l.map, f, 0, 0, 50, 50, 50)
      0 =: temporary.opacity
      l.map.pointer_lat =:> temporary.lat
      l.map.pointer_lon =:> temporary.lon*/
      OutlineOpacity _ (0.5)
      OutlineWidth _ (5)
      OutlineColor _ (234, 234, 234)
      Translation pos (0, 0)
    
      Line temp_shadow_edge (0, 0, 0, 0)

      Int index (1)
      index->(root) {
        setRef (root.current_addedge_node, root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index])
      }
      l.map.layers.navgraph.manager.selected_id =:> index

      //index =:> lp.input

      index -> (root){
        root.addEdge.preview_on.temp_shadow_edge.x1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.screen_translation.tx

        root.addEdge.preview_on.temp_shadow_edge.y1 = root.l.map.layers.navgraph.nodes.[root.addEdge.preview_on.index].wpt.screen_translation.ty
      }
      ddx.value =:> temp_shadow_edge.x1
      ddy.value =:> temp_shadow_edge.y1
      ddtx.value =:> pos.tx
      ddty.value =:> pos.ty 
      f.move.x  - pos.tx =:> temp_shadow_edge.x2
      f.move.y  - pos.ty =:> temp_shadow_edge.y2
      /*temporary.screen_translation.tx =:> temp_shadow_edge.x2
      temporary.screen_translation.ty =:> temp_shadow_edge.y2
    */}

    //idle -> shift_on (shift, addEdgeSpike1)
    idle -> shift_on (shift, clear_temp_list)
    shift_on -> preview_on (root.l.map.layers.navgraph.manager.selected_id, add_first_wpt)
    preview_on -> idle (shift_r, add_segment)
    //shift_on -> idle (shift_r, addEdgeSpike2)
    shift_on -> idle (shift_r, hide_reticule)
  }

  //addEdgeSpike1 -> addEdge.preview_on.temporary.disable_drag, clear_temp_list
  //addEdgeSpike2 -> hide_reticule, addEdge.preview_on.temporary.renable_drag

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