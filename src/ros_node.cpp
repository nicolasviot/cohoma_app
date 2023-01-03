#include "ros_node.h"

#include "exec_env/global_mutex.h"
//#include "core/execution/graph.h"
#include "core/control/binding.h"
#include "core/ontology/coupling.h"
#include "base/connector.h"
#include "core/core-dev.h"
#include "core/tree/list.h"
#include "gui/shape/poly.h"
#include "gui/widgets/multiline_edit.h"
#include "core/utils/getset.h"

#include <nlohmann/json.hpp>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <iterator>
#include "math.h"

#include "model/LimaModel.h"
#include "model/PointModel.h"
#include "model/ExclusionZoneModel.h"
#include "model/NodeModel.h"
#include "model/EdgeModel.h"
#include "model/TrapModel.h"

#include "model/task/TaskEdgeModel.h"
#include "model/task/TaskAreaModel.h"
#include "model/task/TaskTrapModel.h"


#include "task/TaskTrap.h"
#include "task/TaskAreaSummit.h"
#include "task/OldTaskArea.h"
#include "task/OldTaskEdge.h"


using std::placeholders::_1;

using namespace djnn;
using namespace std;

extern Process* get_edge_model (Process* model_manager, int index_1, int index_2);

//TODO : MP
// remove - find_child
// remove set_value
// remove get_value
// check : exclusive access and release access - RETURN
// check boucle for et utilisation des msg ROS2 


RosNode::RosNode (ParentProcess* parent, const string& n, CoreProcess* my_map, CoreProcess* context, CoreProcess* model_manager) :
  FatProcess (n),
  ExternalSource (n),
  //arguments
  _map (my_map),
  _context (context),
  _model_manager (model_manager),

  // Nav Graph fields
  navgraph_data (this, "data", ""),

  // Planif VAB
  _current_plan_id_vab (this, "current_plan_id", 0),
  _start_plan_vab_id (this, "start_plan_id", 0),
  _end_plan_vab_id (this, "end_plan_id", 0)

    #ifndef NO_ROS
    //ROS
  ,qos_best_effort(10),
  qos(1),
  qos_transient(1)
    #endif
{

  #ifndef NO_ROS
  _node = std::make_shared<rclcpp::Node>(n);
  // reliable ~= TCP connections => for the navgraph msgs
  // best_effort allows to drop some pacquets => for robot_state msgs

  
  qos.reliable();
  qos.durability_volatile();
  qos_best_effort.best_effort();
  qos_best_effort.durability_volatile();
  qos_transient.reliable();
  qos_transient.transient_local();
  #endif

  finalize_construction (parent, n);
}


void
RosNode::impl_activate ()
{ 
  #ifndef NO_ROS
  // SUBSCRIBE
  //Replace 10 with qosbesteffort
  sub_navgraph =_node->create_subscription<icare_interfaces::msg::StringStamped>("/navgraph", qos_transient, std::bind(&RosNode::receive_msg_navgraph, this, _1));
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>("/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, this, _1));
  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItineraryList>("/itinerary", qos, std::bind(&RosNode::receive_msg_graph_itinerary_loop, this, _1));
  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>("/plan", qos, std::bind(&RosNode::receive_msg_graph_itinerary_final, this, _1));
  sub_candidate_tasks = _node->create_subscription<icare_interfaces::msg::Tasks>("/candidate_tasks", qos, std::bind(&RosNode::receive_msg_candidate_tasks, this, _1));
  sub_allocation = _node->create_subscription<icare_interfaces::msg::Allocation>("/allocation", qos, std::bind(&RosNode::receive_msg_allocation, this, _1));
  sub_traps = _node->create_subscription<icare_interfaces::msg::TrapList>("/traps", qos_transient, std::bind(&RosNode::receive_msg_trap, this, _1));
  sub_site = _node->create_subscription<icare_interfaces::msg::Site>("/site", qos_transient, std::bind(&RosNode::receive_msg_site, this, _1));
  sub_map = _node->create_subscription<icare_interfaces::msg::EnvironmentMap>("/map", qos_transient, std::bind(&RosNode::receive_msg_map, this, _1));

  // PUBLISH
  publisher_planning_request =_node->create_publisher<icare_interfaces::msg::PlanningRequest>("/planning_request", qos);
  publisher_validation = _node->create_publisher<icare_interfaces::msg::StringStamped>("/validation", qos);
  publisher_navgraph_update = _node->create_publisher<icare_interfaces::msg::StringStamped>("/navgraph_update", qos_transient);
  publisher_tasks = _node->create_publisher<icare_interfaces::msg::Tasks>("/tasks", qos);
  publisher_validation_tasks = _node->create_publisher<icare_interfaces::msg::StringStamped>("/validate", qos);
  publisher_lima = _node->create_publisher<icare_interfaces::msg::LimaCrossed>("/lima", qos);
  publisher_trap_activation = _node->create_publisher<icare_interfaces::msg::TrapActivation>("/activation", qos);
  #endif

  //activate navgraph fields
  navgraph_data.activate();

  _current_plan_id_vab.activate();
  _start_plan_vab_id.activate();
  _end_plan_vab_id.activate();

  GET_CHILD_VAR2 (_frame, CoreProcess, _parent, parent/f)
  GET_CHILD_VAR2 (_layer_filter, CoreProcess, _parent, parent/menu/ui/cb_left)

  // ---------------------------
  // MODEL

  // SITE
  GET_CHILD_VAR2 (_limit_models, CoreProcess, _model_manager, limits)
  GET_CHILD_VAR2 (_zone_models, CoreProcess, _model_manager, zones)
  GET_CHILD_VAR2 (_lima_models, CoreProcess, _model_manager, limas)
  GET_CHILD_VAR2 (_node_models, CoreProcess, _model_manager, nodes)
  GET_CHILD_VAR2 (_edge_models, CoreProcess, _model_manager, edges)

  // Itineraries
  CoreProcess *shortest, *safest, *tradeoff;
  GET_CHILD_VAR2 (shortest, CoreProcess, _model_manager, shortest_itinerary)
  GET_CHILD_VAR2 (safest, CoreProcess, _model_manager, safest_itinerary)
  GET_CHILD_VAR2 (tradeoff, CoreProcess, _model_manager, tradeoff_itinerary)
  _itineraries.push_back(shortest);
  _itineraries.push_back(safest);
  _itineraries.push_back(tradeoff);
  
  // Vehicles
  GET_CHILD_VAR2 (_vab, CoreProcess, _model_manager, vehicles/vab)
  GET_CHILD_VAR2 (_agilex1, CoreProcess, _model_manager, vehicles/agilex1)
  GET_CHILD_VAR2 (_agilex2, CoreProcess, _model_manager, vehicles/agilex2)
  GET_CHILD_VAR2 (_lynx, CoreProcess, _model_manager, vehicles/lynx)
  GET_CHILD_VAR2 (_spot, CoreProcess, _model_manager, vehicles/spot)
  GET_CHILD_VAR2 (_drone, CoreProcess, _model_manager, vehicles/drone)

  // Safety pilots
  GET_CHILD_VAR2 (_drone_safety_pilot, CoreProcess, _model_manager, safety_pilots/drone_safety_pilot)
  GET_CHILD_VAR2 (_ground_safety_pilot, CoreProcess, _model_manager, safety_pilots/ground_safety_pilot)

  // Context
  GET_CHILD_VAR2 (_ref_NULL, RefProperty, _context, REF_NULL)
  GET_CHILD_VAR2 (_ref_node_graph_edition, RefProperty, _context, ref_node_graph_edition)
  GET_CHILD_VAR2 (_ref_node_status_edition, RefProperty, _context, ref_node_status_edition)
  GET_CHILD_VAR2 (_ref_current_trap, RefProperty, _context, ref_current_trap)
  GET_CHILD_VAR2 (_selected_itinerary_id, TextProperty, _context, selected_itinerary_id)

  // TASKS
  GET_CHILD_VAR2 (_task_area_models, CoreProcess, _model_manager, task_areas)
  GET_CHILD_VAR2 (_task_edge_models, CoreProcess, _model_manager, task_edges)
  GET_CHILD_VAR2 (_task_trap_models, CoreProcess, _model_manager, task_traps)

  // TRAPS
  GET_CHILD_VAR2 (_trap_models, CoreProcess, _model_manager, traps)


  // ---------------------------
  // VIEW
  
  GET_CHILD_VAR2 (_nodes, CoreProcess, _parent, parent/l/map/layers/navgraph/nodes)
  GET_CHILD_VAR2 (_edges, CoreProcess, _parent, parent/l/map/layers/navgraph/edges)

  GET_CHILD_VAR2 (_task_edges, CoreProcess, _parent, parent/l/map/layers/tasks/tasklayer/edges)
  GET_CHILD_VAR2 (_task_areas, CoreProcess, _parent, parent/l/map/layers/tasks/tasklayer/areas)
  GET_CHILD_VAR2 (_task_traps, CoreProcess, _parent, parent/l/map/layers/tasks/tasklayer/traps)
  GET_CHILD_VAR2 (_task_allocated_edges, CoreProcess, _parent, parent/l/map/layers/allocated_tasks/allocated_tasks_layer/edges)
  GET_CHILD_VAR2 (_task_allocated_areas, CoreProcess, _parent, parent/l/map/layers/allocated_tasks/allocated_tasks_layer/areas)
  GET_CHILD_VAR2 (_task_allocated_traps, CoreProcess, _parent, parent/l/map/layers/allocated_tasks/allocated_tasks_layer/traps)

  GET_CHILD_VAR2 (_clock, CoreProcess, _parent, parent/right_panel/clock)
  GET_CHILD_VAR2 (_fw_input, CoreProcess, _parent, parent/right_panel/clock/fw/input)
  GET_CHILD_VAR2 (_fw_console_input, CoreProcess, _parent, parent/right_panel/clock/fw_console/input)
  GET_CHILD_VAR2 (_console, CoreProcess, _parent, parent/right_panel/console)

  GET_CHILD_VAR2 (_georef_visibility_map, CoreProcess, _parent, parent/l/map/layers/result/georef_visibility_map)
  GET_CHILD_VAR2 (_visibility_map, DataImage, _parent, parent/l/map/layers/result/visibility_map)
  GET_CHILD_VAR2 (_visibility_map_resolution, DoubleProperty, _parent, parent/l/map/layers/result/visibility_map_resolution)

  //start the thread
  ExternalSource::start ();  
}


void
RosNode::impl_deactivate ()
{
  // Here we should disable the subcription but it 
  // seems there is no way to do it properly
  // some insights here:
  // https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/

#ifndef NO_ROS
  sub_navgraph.reset ();
  sub_robot_state.reset ();
  sub_graph_itinerary_loop.reset ();
  sub_graph_itinerary_final.reset ();
  sub_candidate_tasks.reset ();
  sub_allocation.reset ();

#endif  
  //deactivate navgraph fields
  navgraph_data.deactivate();

  _current_plan_id_vab.deactivate();
  _start_plan_vab_id.deactivate();
  _end_plan_vab_id.deactivate();

  ExternalSource::please_stop ();
}


void
RosNode::run () {
#ifndef NO_ROS
  rclcpp::spin(_node);
  rclcpp::shutdown();
#endif
}


void
RosNode::activate_layer(const string& layer_to_activate)
{
  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container != nullptr)
  {
    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--)
    {
      auto *child = _layer_filter_container->children ()[i];
      GET_CHILD_VALUE (layer_name, Text, child, name)
      if (layer_name == layer_to_activate)
      {
        GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
        if (activation_state == "hidden"){
          child->find_child("cb/press")->activate();
        }
      }
    }
  }
}

void
RosNode::deactivate_layer (const string& layer_to_deactivate)
{
  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container != nullptr)
  {
    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--)
    {
      auto *child = _layer_filter_container->children ()[i];
      GET_CHILD_VALUE (layer_name, Text, child, name)
      if (layer_name == layer_to_deactivate)
      {
        GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
        if (activation_state == "visible") {
          child->find_child("cb/press")->activate();
        }
      }
    }
  }
}


#ifndef NO_ROS

// **************************************************************************************************
//
//  Navigation Graph
//
// **************************************************************************************************

// Receive msg "Navigation Graph"
void 
RosNode::receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg)
{
  get_exclusive_access(DBG_GET);

  cout << "Receive msg Navigation Graph" << endl;

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received new navgraph\n", true)
  
  // Reset "nodes" in case it contains a pointer on a node that will be removed
  //_ref_node_graph_edition->set_value ((CoreProcess*)nullptr, true);
  _ref_node_graph_edition->set_value (_ref_NULL, true);
    //_ref_node_status_edition->set_value ((CoreProcess*)nullptr, true);
  _ref_node_status_edition->set_value (_ref_NULL, true);


  /*Container *_task_edge_container = dynamic_cast<Container *> (_task_edges);
  if (_task_edge_container) {
    int _task_edge_container_size = _task_edge_container->children ().size ();
    for (int i = _task_edge_container_size - 1; i >= 0; i--) {
      auto *item = _task_edge_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }*/

  /*Container *_trap_container = dynamic_cast<Container *> (_task_traps);
  if (_trap_container) {
    int _trap_container_size = _trap_container->children ().size ();
    for (int i = _trap_container_size - 1; i >= 0; i--) {
      auto *item = _trap_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }*/

  /*Container *_task_areas_container = dynamic_cast<Container *> (_task_areas);
  if (_task_areas_container) {
    int _task_container_size = _task_areas_container->children ().size ();
    for (int i = _task_container_size - 1; i >= 0; i--) {
      auto *item = _task_areas_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }*/

  // schedule delete old itineraries
  // schedule delete old edges
  // schedule delete old nodes

  nlohmann::json j = nlohmann::json::parse(msg->data);
  nlohmann::json j_graph;
  if (j.contains("graph"))
    j_graph = j["graph"];
  else if (j.contains("graphs")) {
    if (j.size() > 1)
      j_graph = j["graphs"][0];
    else if (j.size() == 0)
      return;
  }
  
  // NODES
  for (int i = j_graph["nodes"].size() - 1; i >= 0; i--)
  {
    auto& json_node = j_graph["nodes"][i];
    const string& node_id = json_node["id"].get<string>();
    const string& label = json_node["label"].get<string>();
    //const string& node_id = json_node["id"];
    //const string& label = json_node["label"];
    
    auto& m = json_node["metadata"];
    int phase = m["phase"].get<int>();
    double latitude = m["latitude"].get<double>();
    double longitude = m["longitude"].get<double>();
    double altitude = m["altitude"].get<double>();
    bool mandatory = m["compulsory"].get<bool>();
    //bool locked = m["locked"].get<bool>();

    NodeModel (_node_models, "", std::stoi(node_id) + 1, phase, label, latitude, longitude, altitude, mandatory);
  }

  // EDGES
  for (auto& j_edge : j_graph["edges"])
  {
    const string& str_source = j_edge["source"].get<string>();
    const string& str_target = j_edge["target"].get<string>();
    int n_source = stoi(str_source) + 1;
    int n_target = stoi(str_target) + 1;

    auto& m = j_edge["metadata"];
    double length = m["length"].get<double>();
    
    //cout << n_source << "-->" << n_target << endl;

    Process* source = _node_models->find_child (to_string (n_source));
    Process* target = _node_models->find_child (to_string (n_target));
    EdgeModel (_edge_models, "", source, target, length);
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

#endif


#ifndef NO_ROS

// Receive msg Graph itineraries
void 
RosNode::receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg)
{
  cout << "Receive msg Graph itineraries (loop) with " << msg->itineraries.size() << " itineraries." << endl;
  
  if (msg->itineraries.size() <= 0)
    return;

  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(msg->itineraries.size()) + " itineraries\n", true);
 
  if (msg->itineraries.size() == _itineraries.size())
  {
    CoreProcess* model = nullptr;
    CoreProcess* list_node_ids = nullptr;

    for (int i = 0; i < msg->itineraries.size(); i++)
    {
      cout << "Iti " << i << " with " << msg->itineraries[i].nodes.size() << " nodes" << endl;

      model = _itineraries[i];
      SET_CHILD_VALUE (Text, model, uid, msg->itineraries[i].id, true)
      SET_CHILD_VALUE (Text, model, description_input, msg->itineraries[i].description, true)
      
      GET_CHILD_VAR2 (list_node_ids, CoreProcess, model, node_ids)
      //GET_CHILD_VAR2 (list_node_indexes, CoreProcess, model, node_indexes)

      for (int j = 0; j < msg->itineraries[i].nodes.size(); j++) {
        //cout << "New IntProperty " << msg->itineraries[i].nodes[j] << " in " << i << endl;
        new IntProperty (list_node_ids, "", std::stoi(msg->itineraries[i].nodes[j]));
        //new IntProperty (list_node_indexes, "", std::stoi(msg->itineraries[i].nodes[j]));
      }
    }
  }
  else
    cerr << "Different size about itineraries between 'msg_graph_itinerary_loop' and models of itineraries" << endl;

  /*string first_id = "";
  for (auto ros_itinerary : msg_struct) {
    // set first id
    if (first_id == "")
      first_id = ros_itinerary.first;
    
  SET_CHILD_VALUE (Text, _selected_itinerary_id, , first_id, true)*/
  
  _model_manager->find_child("itineraries_updated")->notify_activation();

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


// Receive msg FINAL itinerary (in graph)
void 
RosNode::receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg)
{
  get_exclusive_access(DBG_GET);

  cout << "Receive msg FINAL itinerary (in graph) (id = " << msg->id << ")" << endl;

  // schedule delete old itineraries

  /*Component *new_itinerary = new Component ( _itineraries_list, msg->id );
  new TextProperty (new_itinerary, "id", msg->id);
  List* new_ite_edges = new List (new_itinerary, "edges");
  int ite_edges_size = msg->nodes.size ();
  if ( ite_edges_size > 0) {
    for (int i = 1; i < ite_edges_size; i++) {
      ParentProcess* edge = Edge( new_ite_edges, "", std::stoi(msg->nodes[i-1]) + 1,std::stoi(msg->nodes[i]) + 1, 20, _nodes);
      SET_CHILD_VALUE (Int, edge, outline_color/value, selected, true)
    }
  }*/

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


// **************************************************************************************************
//
//  SATELLITE
//
// **************************************************************************************************
void 
RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState::SharedPtr msg)
{  
  //RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);

  djnn::Process * robots[] = {nullptr, _drone, _agilex1, _agilex2, _lynx, _spot, _vab, _drone_safety_pilot, _ground_safety_pilot};
  static const string robots_name[] = {"", "drone", "agilex1", "agilex2", "lynx", "spot", "vab", "drone_safety_pilot", "ground_safety_pilot"};
  if ((msg->robot_id < 1) || (msg->robot_id >= sizeof(robots))) {
    RCLCPP_INFO(_node->get_logger(), "incorrect robot_id: '%d'", msg->robot_id);
    return;
  }
    
  djnn::Process * robot = robots[msg->robot_id];
  const string& robot_name = robots_name[msg->robot_id];

  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);

  SET_CHILD_VALUE (Double, robot, lat, msg->position.latitude, true);
  SET_CHILD_VALUE (Double, robot, lon, msg->position.longitude, true);
  //SET_CHILD_VALUE (Double, robot, altitude_msl, msg->position.altitude, true);

  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Received robot_state for " + robot_name + "\n", true);
  
  if ((robot != _drone_safety_pilot) && (robot != _ground_safety_pilot)) {
    SET_CHILD_VALUE (Double, robot, altitude_msl, msg->position.altitude, true);
    SET_CHILD_VALUE (Double, robot, heading_rot, msg->compass_heading, true);
    SET_CHILD_VALUE (Int, robot, battery_percentage, msg->battery_percentage, true);
    SET_CHILD_VALUE (Int, robot, operation_mode, msg->operating_mode, true); // FIXME: operation_mode vs operating_mode
    SET_CHILD_VALUE (Bool, robot, emergency_stop, msg->emergency_stop, true);
    SET_CHILD_VALUE (Bool, robot, failsafe, msg->failsafe, true);
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


// **************************************************************************************************
//
//  TRAP
//
// **************************************************************************************************

// Receive msg Trap
void 
RosNode::receive_msg_trap (const icare_interfaces::msg::TrapList msg)
{    
  get_exclusive_access(DBG_GET);

  cout << "Receive msg Trap" << endl;
  
  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  
  int new_trap = 0;
  int update_trap = 0;
  int index_found = -1;
  CoreProcess *current_trap_model;
  Container *trap_models = dynamic_cast<Container*> (_trap_models);

  for (int k = 0; k < msg.traps.size(); k ++)
  {
    index_found = -1;
    current_trap_model = nullptr;

    for (int i = 0; i < trap_models->children().size(); i++)
    {
      GET_CHILD_VALUE (id, Int, trap_models->children()[i], id)
      if (id == msg.traps[k].id) {
        index_found = i;
        break;
      }
    }

    // New Trap
    if (index_found == -1)
    {
      new_trap = new_trap + 1;
      
      ParentProcess *new_trap = TrapModel (trap_models, "", _context, msg.traps[k].id, msg.traps[k].location.latitude, msg.traps[k].location.longitude, this);
  
      current_trap_model = new_trap;

      SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - New trap #" + std::to_string(msg.traps[k].id) + "\n", true);
      
      if (msg.traps[k].identified)
        SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - New trap identified "+ msg.traps[k].info.id +"(#" +std::to_string(msg.traps[k].id) + ")" +  " " + msg.traps[k].info.code  + " " + msg.traps[k].info.hazard + "\n", true);
    }
    // Already Existing Trap
    else
    {  
      update_trap = update_trap + 1;

      current_trap_model = trap_models->children()[index_found];

      if ( msg.traps[k].identified && !((BoolProperty*)current_trap_model->find_child("identified"))->get_value() )
        SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - trap identified "+ msg.traps[k].info.id +"(#" +std::to_string(msg.traps[k].id) + ")" +  " " + msg.traps[k].info.code  + " " + msg.traps[k].info.hazard + "\n", true)
      else if ( msg.traps[k].identified )
        SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp+ " - trap updated "+ msg.traps[k].info.id +"(#" +std::to_string(msg.traps[k].id) + ")" +  " " + msg.traps[k].info.code  + " " + msg.traps[k].info.hazard + "\n", true)
    }

    SET_CHILD_VALUE (Bool, current_trap_model, active, msg.traps[k].active, true)
    SET_CHILD_VALUE (Bool, current_trap_model, identified, msg.traps[k].identified, true)
    SET_CHILD_VALUE (Text, current_trap_model, str_id, msg.traps[k].info.id, true)
    SET_CHILD_VALUE (Text, current_trap_model, description, msg.traps[k].info.description, true)
    SET_CHILD_VALUE (Int, current_trap_model, contact_mode, msg.traps[k].info.contact_mode, true)
    SET_CHILD_VALUE (Text, current_trap_model, code, msg.traps[k].info.code, true)
    SET_CHILD_VALUE (Text, current_trap_model, hazard, msg.traps[k].info.hazard, true)
    //SET_CHILD_VALUE (Double, current_trap_model, radius, msg.traps[k].info.radius, true) 
    SET_CHILD_VALUE (Bool, current_trap_model, remotely_deactivate, msg.traps[k].info.remotely_deactivate, true)
    SET_CHILD_VALUE (Bool, current_trap_model, contact_deactivate, msg.traps[k].info.contact_deactivate, true)

    GRAPH_EXEC;
  }
  
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(msg.traps.size()) + " traps (" + std::to_string(new_trap) + " new, " + std::to_string(update_trap) + " updated)\n" , true);
      
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


// **************************************************************************************************
//
//  TASK
//
// **************************************************************************************************

// Receive msg Candidate Tasks
void 
RosNode::receive_msg_candidate_tasks(const icare_interfaces::msg::Tasks msg)
{
  get_exclusive_access(DBG_GET);

  cout << "Receive msg Candidate Tasks" << endl;

  /*Container *_edge_container = dynamic_cast<Container *> (_task_edges);
  if (_edge_container) {
    int _edge_container_size = _edge_container->children ().size ();
    for (int i = _edge_container_size - 1; i >= 0; i--) {
      auto *item = _edge_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }*/

  /*Container *_trap_container = dynamic_cast<Container *> (_task_traps);
  if (_trap_container) {
    int _trap_container_size = _trap_container->children ().size ();
    for (int i = _trap_container_size - 1; i >= 0; i--) {
      auto *item = _trap_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }*/

  /*Container *_task_areas_container = dynamic_cast<Container *> (_task_areas);
  if (_task_areas_container) {
    int _task_container_size = _task_areas_container->children ().size ();
    for (int i = _task_container_size - 1; i >= 0; i--) {
      auto *item = _task_areas_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }*/
  
  // Aerial
  int nb_uav_zone = msg.uav_zones.size();
  // Ground
  int nb_ugv_edges = msg.ugv_edges.size();

  int nb_trap_identification = msg.trap_identifications.size();  
  int nb_trap_deactivation = msg.trap_deactivations.size();
  
  int nb_total = nb_uav_zone + nb_ugv_edges + nb_trap_deactivation + nb_trap_identification;
  
  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(nb_total) + " tasks ("+ std::to_string(nb_uav_zone) + " uav_zones, " + std::to_string(nb_ugv_edges) + " ugv_edges, " + std::to_string(nb_trap_identification) + " trap_identifications, " + std::to_string(nb_trap_deactivation) + " trap_deactivations)\n", true)
 
  // Aerial --> Task Area Model
  for (int i = 0; i < nb_uav_zone; i++)
  {
    double area = msg.uav_zones[i].area;
    double explored = msg.uav_zones[i].explored;

    ParentProcess* task = TaskAreaModel (_task_area_models, "", area, explored);
    ParentProcess* points = task->find_child ("points");
    for (int j = 0; j < msg.uav_zones[i].points.size(); j++)
    {
      PointModel (points, "", msg.uav_zones[i].points[j].latitude, msg.uav_zones[i].points[j].longitude, msg.uav_zones[i].points[j].altitude);
    }
  }

  // Ground --> Task Edge Model
  for (int i = 0; i < nb_ugv_edges; i++)
  {
    int n_source = stoi(msg.ugv_edges[i].source);
    int n_target = stoi(msg.ugv_edges[i].target);
    //double length = msg.ugv_edges[i].length;
    double explored = msg.ugv_edges[i].explored;
    //cout << n_source << "-->" << n_target << "(" << length << "m) explored = " << explored << endl;

    Process* edge = get_edge_model (_model_manager, n_source, n_target);
    if (edge != nullptr) {
      TaskEdgeModel (_task_edge_models, "", edge, explored);
    }
    else
      cerr << "NO edge model " << n_source << " --> " << n_target << " to create the corresponding task !" << endl;
  }
  

  CoreProcess* trap_model = nullptr;
  Container* trap_models = dynamic_cast<Container*> (_trap_models);

  // Pièges à identifier. On doit envoyer un robot pour aller les identifier (lire le QR code)
  for (int i = 0; i < msg.trap_identifications.size(); i++)
  {
    trap_model = nullptr;
    // FIXME: id (= task id ?) vs trap_id (= always empty ?)
    int id = msg.trap_identifications[i].id;
    string trap_id = msg.trap_identifications[i].info.id;
    cout << "Trying to add a task to IDENTIFY trap: id '" << id << "' or trap id '" << trap_id << "'" << endl;

    // Try to get the existing model with this id
    for (int j = 0; j < trap_models->children().size(); j++)
    {
      GET_CHILD_VALUE (tmp_id, Int, trap_models->children()[j], id)

      if (tmp_id == id) {
        trap_model = trap_models->children()[j];
        break;
      }
    }

    if (trap_model != nullptr)
    {
      // FIXME: Need to update something in the trap model ?
      /*ParentProcess* trap_to_add = TaskTrap(_task_traps, "", _map, msg.trap_identifications[i].id, msg.trap_identifications[i].location.latitude, msg.trap_identifications[i].location.longitude);
      SET_CHILD_VALUE (Bool, trap_to_add, active, msg.trap_identifications[i].active, true)
      SET_CHILD_VALUE (Bool, trap_to_add, identified, msg.trap_identifications[i].identified, true)
      SET_CHILD_VALUE (Text, trap_to_add, trap_id_str, msg.trap_identifications[i].info.id, true)
      SET_CHILD_VALUE (Text, trap_to_add, description, msg.trap_identifications[i].info.description, true)
      SET_CHILD_VALUE (Int, trap_to_add, contact_mode, msg.trap_identifications[i].info.contact_mode, true)
      SET_CHILD_VALUE (Text, trap_to_add, code, msg.trap_identifications[i].info.code, true)
      SET_CHILD_VALUE (Text, trap_to_add, hazard, msg.trap_identifications[i].info.hazard, true)
      SET_CHILD_VALUE (Double, trap_to_add, radius, msg.trap_identifications[i].info.radius, true)*/

      // Create a task model with this trap model
      TaskTrapModel (_task_trap_models, "", trap_model);
    }
    else {
      cerr << "There is NO model for trap id '" << id << "'. Can't create the task to identify trap." << endl;
    }
  }
  
  // Pièges déjà identifiés et qui nécessitent une désactivation au contact
  for (int i = 0; i < msg.trap_deactivations.size(); i++)
  {  
    trap_model = nullptr;
    // FIXME: id (= task id ?) vs trap_id (= always empty ?)
    int id = msg.trap_deactivations[i].id;
    string trap_id = msg.trap_deactivations[i].info.id;
    cout << "Trying to add a task to DE-ACTIVATE trap: id '" << id << "' or trap id '" << trap_id << "'" << endl;

    // Try to get the existing model with this id
    for (int j = 0; j < trap_models->children().size(); j++)
    {
      GET_CHILD_VALUE (tmp_id, Int, trap_models->children()[j], id)

      if (tmp_id == id) {
        trap_model = trap_models->children()[j];
        break;
      }
    }

    if (trap_model != nullptr)
    {
      // FIXME: Need to update something in the trap model ?
      /*ParentProcess* trap_to_add = TaskTrap(_task_traps, "", _map, msg.trap_deactivations[i].id, msg.trap_deactivations[i].location.latitude, msg.trap_deactivations[i].location.longitude);
      SET_CHILD_VALUE (Bool, trap_to_add, active, msg.trap_deactivations[i].active, true)
      SET_CHILD_VALUE (Bool, trap_to_add, identified, msg.trap_deactivations[i].identified, true)
      SET_CHILD_VALUE (Text, trap_to_add, trap_id_str, msg.trap_deactivations[i].info.id, true)
      SET_CHILD_VALUE (Text, trap_to_add, description, msg.trap_deactivations[i].info.description, true)
      SET_CHILD_VALUE (Int, trap_to_add, contact_mode, msg.trap_deactivations[i].info.contact_mode, true)
      SET_CHILD_VALUE (Text, trap_to_add, code, msg.trap_deactivations[i].info.code, true)
      SET_CHILD_VALUE (Text, trap_to_add, hazard, msg.trap_deactivations[i].info.hazard, true)
      SET_CHILD_VALUE (Double, trap_to_add, radius, msg.trap_deactivations[i].info.radius, true)*/

      // Create a task model with this trap model
      TaskTrapModel (_task_trap_models, "", trap_model);
    }
    else {
      cerr << "There is NO model for trap id " << id << ". Can't create the task to deactivate trap." << endl;
    }
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


// Receive msg Tasks Assignment (Allocation)
void 
RosNode::receive_msg_allocation(const icare_interfaces::msg::Allocation msg)
{
  get_exclusive_access(DBG_GET);

  cout << "Receive msg Tasks Assignment (Allocation)" << endl;

  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container)
  {
    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--)
    {
      auto *child = _layer_filter_container->children ()[i];
      GET_CHILD_VALUE (layer_name, Text, child, name)
      //cout << "Found layer" << layer_name << std::endl;

      if (layer_name == "Tasks")
      {
        cout << "Found 'Tasks' layer" << endl;
        GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
        if (activation_state == "visible")
        {
          cout << "Tasks layer is visible --> Notify activation to cb/press" << endl;
          child->find_child("cb/press")->notify_activation();
        }
      }

      if (layer_name == "Allocation")
      {
        cout << "Found 'Allocation' layer" << endl;
        GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
        if (activation_state == "hidden")
        {
          cout << "Allocation layer is hidden --> Notify activation to cb/press" << endl;
          child->find_child("cb/press")->notify_activation();
        }
      }
    }
  }

  Container *_edge_container = dynamic_cast<Container *> (_task_allocated_edges);
  if (_edge_container) {
    int _edge_container_size = _edge_container->children ().size ();
    for (int i = _edge_container_size - 1; i >= 0; i--) {
      auto *item = _edge_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

  Container *_trap_container = dynamic_cast<Container *> (_task_allocated_traps);
  if (_trap_container) {
    int _trap_container_size = _trap_container->children ().size ();
    for (int i = _trap_container_size - 1; i >= 0; i--) {
      auto *item = _trap_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

  Container *_task_areas_container = dynamic_cast<Container *> (_task_allocated_areas);
  if (_task_areas_container) {
    int _task_container_size = _task_areas_container->children ().size ();
    for (int i = _task_container_size - 1; i >= 0; i--) {
      auto *item = _task_areas_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

  GRAPH_EXEC;
  
  // Allocation = list of assigned tasks
  /*
  std_msgs/Header header
  uint8 robot_id
  uint8 task_type
  icare_interfaces/ExplorationPolygon zone
  icare_interfaces/GraphEdge edge
  icare_interfaces/Trap identification
  icare_interfaces/Trap deactivation

  uint8 TASK_TYPE_UNKNOWN = 0
  uint8 TASK_TYPE_ZONE = 1
  uint8 TASK_TYPE_EDGE = 2
  uint8 TASK_TYPE_IDENTIFICATION = 3
  uint8 TASK_TYPE_DEACTIVATION = 4
  */

  int nb_uav_zone = 0;
  int nb_ugv_edges = 0;
  int nb_trap_identification = 0;  
  int nb_trap_deactivation = 0;
  int nb_total = msg.tasks.size();
  
  //GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  //SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(nb_total) + " tasks ("+ std::to_string(nb_uav_zone) + " uav_zones, " + std::to_string(nb_ugv_edges) + " ugv_edges, " + std::to_string(nb_trap_identification) + " trap_identifications, " + std::to_string(nb_trap_deactivation) + " trap_deactivations)\n", true)

  /*
  //flashy

  Int droneCOL (#1ACAFF) 1
  Int agiCOL (#0C2EE8) 2
  Int agiCOL2 (#B500FF) 3
  Int lynxCOL (#B3B100) 4
  Int spotCOL (#0CE820) 5
  Int vabCOL (#00B1E6) 6
  */

  int colors[7] = {0x000000, 0x1ACAFF, 0x0C2EE8, 0xB500FF, 0xB3B100, 0x0CE820, 0x00B1E6}; 
  for (int i = 0; i < nb_total; i++)
  {  
    // ZONE
    /*if (msg.tasks[i].task_type == 1)
    {
      ParentProcess* area_to_add = OldTaskArea(_task_allocated_areas, "", _map);
      for (int j=0 ;j< msg.tasks[i].zone.points.size(); j++){
        auto* task_summit = TaskAreaSummit (area_to_add, std::string("summit_") + std::to_string(j), _map, msg.tasks[i].zone.points[j].latitude, msg.tasks[i].zone.points[j].longitude);
        SET_CHILD_VALUE (Double, task_summit, alt, msg.tasks[i].zone.points[j].altitude, true)
        auto* point = new PolyPoint(area_to_add->find_child("area"), std::string("pt_") + std::to_string(j), 0, 0);

        new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);
        new Connector (area_to_add, "y_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);
    
      }
      SET_CHILD_VALUE (Int, area_to_add, nb_summit, (int) (msg.tasks[i].zone.points.size()), true)
      SET_CHILD_VALUE (Int, area_to_add, color/value, colors[msg.tasks[i].robot_id], true)
    }*/
    // EDGE
    /*else if (msg.tasks[i].task_type == 2)
    {
      ParentProcess* edge_to_add = OldTaskEdge(_task_allocated_edges, "", _map, std::stoi(msg.tasks[i].edge.source) + 1, std::stoi(msg.tasks[i].edge.target) + 1, _nodes);
    
      SET_CHILD_VALUE (Double, edge_to_add, length, msg.tasks[i].edge.length, true)
      SET_CHILD_VALUE (Double, edge_to_add, explored, msg.tasks[i].edge.explored, true)
      SET_CHILD_VALUE (Int, edge_to_add, the_edge/outline_color/value, colors[msg.tasks[i].robot_id], true);
    }*/
    // IDENTIFICATION
    /*else if (msg.tasks[i].task_type == 3)
    {
      ParentProcess* trap_to_add = TaskTrap(_task_allocated_traps, "", _map, msg.tasks[i].identification.id, msg.tasks[i].identification.location.latitude, msg.tasks[i].identification.location.longitude);
      SET_CHILD_VALUE (Bool, trap_to_add, active, msg.tasks[i].identification.active, true)
      SET_CHILD_VALUE (Bool, trap_to_add, identified, msg.tasks[i].identification.identified, true)
      SET_CHILD_VALUE (Text, trap_to_add, trap_id_str, msg.tasks[i].identification.info.id, true)
      SET_CHILD_VALUE (Text, trap_to_add, description, msg.tasks[i].identification.info.description, true)
      SET_CHILD_VALUE (Int, trap_to_add, contact_mode, msg.tasks[i].identification.info.contact_mode, true)
      SET_CHILD_VALUE (Text, trap_to_add, code, msg.tasks[i].identification.info.code, true)
      SET_CHILD_VALUE (Text, trap_to_add, hazard, msg.tasks[i].identification.info.hazard, true)
      SET_CHILD_VALUE (Double, trap_to_add, radius, msg.tasks[i].identification.info.radius, true)
      SET_CHILD_VALUE (Int, trap_to_add, content/red, colors[msg.tasks[i].robot_id], true) 
    }
    // DEACTIVATION
    else if (msg.tasks[i].task_type == 4)
    {
      ParentProcess* trap_to_add = TaskTrap(_task_allocated_traps, "", _map, msg.tasks[i].deactivation.id, msg.tasks[i].deactivation.location.latitude, msg.tasks[i].deactivation.location.longitude);
      SET_CHILD_VALUE (Bool, trap_to_add, active, msg.tasks[i].deactivation.active, true)
      SET_CHILD_VALUE (Bool, trap_to_add, identified, msg.tasks[i].deactivation.identified, true)
      SET_CHILD_VALUE (Text, trap_to_add, trap_id_str, msg.tasks[i].deactivation.info.id, true)
      SET_CHILD_VALUE (Text, trap_to_add, description, msg.tasks[i].deactivation.info.description, true)
      SET_CHILD_VALUE (Int, trap_to_add, contact_mode, msg.tasks[i].deactivation.info.contact_mode, true)
      SET_CHILD_VALUE (Text, trap_to_add, code, msg.tasks[i].deactivation.info.code, true)
      SET_CHILD_VALUE (Text, trap_to_add, hazard, msg.tasks[i].deactivation.info.hazard, true)
      SET_CHILD_VALUE (Double, trap_to_add, radius, msg.tasks[i].deactivation.info.radius, true)
      SET_CHILD_VALUE (Int, trap_to_add, content/red, colors[msg.tasks[i].robot_id], true)
    }*/
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}



void
RosNode::send_msg_lima(int id)
{
  icare_interfaces::msg::LimaCrossed message = icare_interfaces::msg::LimaCrossed();
  message.id = id;
  //cout << "send_msg_lima " << id << endl;

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Validated lima " + std::to_string(id) + "\n", true);
  
  message.header.stamp = _node->get_clock()->now();

  publisher_lima->publish(message);
}


void
RosNode::send_msg_planning_request()
{    
  icare_interfaces::msg::PlanningRequest message = icare_interfaces::msg::PlanningRequest();
  message.id = _current_plan_id_vab.get_string_value();
  //cout << "send_msg_planning_request " << _current_plan_id_vab.get_string_value() << endl;

  for (auto item : ((djnn::List*)_node_models)->children())
  {
    GET_CHILD_VALUE (str_status, Text, item, status)
    GET_CHILD_VALUE (n_id, Int, item, id)

    if (str_status == "start")
      message.start_node = to_string(n_id - 1);
    else if ( str_status == "end")
      message.end_node = to_string(n_id - 1);
    else if (str_status == "forced")
        message.node_contraints.push_back(to_string(n_id -1));
  }

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Asked planification between nodes "+ message.start_node + " and " + message.end_node + " \n", true);

  message.header.stamp = _node->get_clock()->now();

  publisher_planning_request->publish(message);  
}


void 
RosNode::send_msg_navgraph_update(){

  GET_CHILD_VAR (nodes, CoreProcess, _parent, parent/l/map/layers/navgraph/nodes)
  GET_CHILD_VAR (edges, CoreProcess, _parent, parent/l/map/layers/navgraph/edges)

  //std::cerr << "about to generate json" << std::endl;
  nlohmann::json j;
  j["graph"]["directed"] = false;

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Send navgraph update\n", true);

  // Edges
  for (auto item: ((djnn::List*)edges)->children()){

    GET_CHILD_VALUE (ssource_id, Int, item, id_src)
    GET_CHILD_VALUE (starget_id, Int, item, id_dest)
    GET_CHILD_VALUE (dlength, Double, item, length)

    nlohmann::json jn = {
      {"source", std::to_string(ssource_id - 1)},
      {"target", std::to_string(starget_id - 1)},
      {"metadata", { 
       // {"length", dlength}
      }}
    };                
    j["graph"]["edges"].push_back(jn); 
  }

  // FIXME: use models instead of views
  // Nodes
  for (auto item: ((djnn::List*)nodes)->children())
  {
    GET_CHILD_VALUE (iid, Int, item, id)
    GET_CHILD_VALUE (slabel, Text, item, label)
    GET_CHILD_VALUE (dlat, Double, item, wpt/lat)
    GET_CHILD_VALUE (dlon, Double, item, wpt/lon)
    GET_CHILD_VALUE (dalt, Double, item, alt)
    GET_CHILD_VALUE (scompulsory, Text, item, wpt/usage_status)
    GET_CHILD_VALUE (phase, Int, item, phase)
    GET_CHILD_VALUE (compulsory, Bool, item, wpt/isMandatory)
    GET_CHILD_VALUE (locked, Bool, item, islocked)
    
    nlohmann::json jn = {
      {"id", std::to_string(iid - 1)},
      {"label", slabel},
      {"metadata", { 
        {"altitude", dalt},
        {"latitude", dlat},
        {"longitude", dlon},
        {"compulsory", compulsory},
        {"locked", locked},
        {"phase", phase}
      }}
    };                
    j["graph"]["nodes"].push_back(jn);   
  }

  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  message.data = j.dump();
  message.header.stamp = _node->get_clock()->now();

  publisher_navgraph_update->publish(message);
}


// Send Validation PLAN
void 
RosNode::send_validation_plan()
{
  cout << "Send Validation PLAN" << endl;

  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  message.data = _selected_itinerary_id->get_string_value();
  message.header.stamp = _node->get_clock()->now();
  publisher_validation->publish(message);

  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container)
  {
    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--)
    {
        auto *child = _layer_filter_container->children ()[i];
        GET_CHILD_VALUE (layer_name, Text, child, name)
        //cout << "Found layer" << layer_name << std::endl;

        if (layer_name == "Tasks")
        {
          cout << "Found 'Tasks' layer" << endl;
          GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
          if (activation_state == "hidden")
          {
            cout << "Tasks layer is hidden --> Notify activation to cb/press" << endl;
            child->find_child("cb/press")->notify_activation();
          }
        }

        if (layer_name == "Allocation")
        {
          std::cerr << "Found 'Allocation' layer" << std::endl;
          GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
          if (activation_state == "visible")
          {
            cout << "Allocation layer is visible --> Notify activation to cb/press" << endl;
            child->find_child("cb/press")->notify_activation();
          }
        }
    }
  }

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Validate plan #" + message.data + "\n" , true);
}


// Send Selected Tasks
void 
RosNode::send_selected_tasks()
{
  cout << "Send Selected Tasks" << endl;

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Send task selection\n", true);

  icare_interfaces::msg::Tasks message = icare_interfaces::msg::Tasks();

  // TRAPS
  for (auto model : ((djnn::List*)_task_trap_models)->children())
  {
    GET_CHILD_VALUE (trap_is_selected, Bool, model, is_selected)
    if (trap_is_selected)
    {     
      icare_interfaces::msg::Trap trap_to_add = icare_interfaces::msg::Trap();
      GET_CHILD_VALUE2 (trap_to_add.id, Int, model, trap/id)
      GET_CHILD_VALUE2 (trap_to_add.identified, Bool, model, trap/identified)
      GET_CHILD_VALUE2 (trap_to_add.active, Bool, model, trap/active)
      GET_CHILD_VALUE2 (trap_to_add.location.latitude, Double, model, trap/lat)
      GET_CHILD_VALUE2 (trap_to_add.location.longitude, Double, model, trap/lon)

      if (trap_to_add.identified)
        message.trap_deactivations.push_back(trap_to_add);
      else
        message.trap_identifications.push_back(trap_to_add);
    }
  }

  // EDGES
  for (auto model : ((djnn::List*)_task_edge_models)->children())
  {
    GET_CHILD_VALUE (edge_is_selected, Bool, model, is_selected)
    if (edge_is_selected)
    {
      icare_interfaces::msg::GraphEdge edge_to_add = icare_interfaces::msg::GraphEdge();
      GET_CHILD_VALUE (source, Int, model, edge/node1/id)
      edge_to_add.source = std::to_string(source - 1);
      GET_CHILD_VALUE (dest, Int, model, edge/node2/id)
      edge_to_add.target = std::to_string(dest - 1);
      GET_CHILD_VALUE2 (edge_to_add.length, Double, model, edge/length)
      GET_CHILD_VALUE2 (edge_to_add.explored, Double, model, explored)
      message.ugv_edges.push_back(edge_to_add);
    }
  }

  // AREAS
  for (auto task_area : ((djnn::List*)_task_area_models)->children())
  {
    GET_CHILD_VALUE (area_is_selected, Bool, task_area, is_selected)
    if (area_is_selected)
    {
      icare_interfaces::msg::ExplorationPolygon geopolygon_to_add = icare_interfaces::msg::ExplorationPolygon();
      GET_CHILD_VALUE2 (geopolygon_to_add.area, Double, task_area, area)
      GET_CHILD_VALUE2 (geopolygon_to_add.explored, Double, task_area, explored)
      
      GET_CHILD_VALUE (nb_points, Int, task_area, points/size)
      for (int i = 0; i < nb_points; i++)
      {
        CoreProcess* point = dynamic_cast<CoreProcess*>(task_area->find_child("points/" + std::to_string(i + 1)));
        if (point != nullptr)
        {
          geographic_msgs::msg::GeoPoint point_to_add = geographic_msgs::msg::GeoPoint();
          GET_CHILD_VALUE2 (point_to_add.latitude, Double, point, lat)
          GET_CHILD_VALUE2 (point_to_add.longitude, Double, point, lon)
          GET_CHILD_VALUE2 (point_to_add.altitude, Double, point, alt)
          geopolygon_to_add.points.push_back(point_to_add);
        }
        else
          cerr << "Point doesn't exist at index " << i << " for task area " << geopolygon_to_add.area << endl;
      }
      message.uav_zones.push_back(geopolygon_to_add);
    }
  }

  message.header.stamp = _node->get_clock()->now();
  publisher_tasks->publish(message);
}


// **************************************************************************************************
//
//  SITE
//
// **************************************************************************************************

// Receive msg SITE with limits, exclusion zones, and Limas
void 
RosNode::receive_msg_site(const icare_interfaces::msg::Site msg)
{
  get_exclusive_access(DBG_GET);

  cout << "Receive msg SITE" << endl;

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received site data\n", true);

  // LIMITS
  for (int i = 0; i < msg.limits.points.size(); i++)
  {
    PointModel (_limit_models, "", msg.limits.points[i].latitude, msg.limits.points[i].longitude, msg.limits.points[i].altitude);
  }

  // EXCLUSION ZONES
  for (int i = 0; i < msg.zones.size(); i++)
  {
    ParentProcess* zone = ExclusionZoneModel (_zone_models, "", msg.zones[i].type, msg.zones[i].name);
    ParentProcess* points = zone->find_child ("points");
    for (int j = 0; j < msg.zones[i].polygon.points.size(); j++)
    {
      PointModel (points, "", msg.zones[i].polygon.points[j].latitude, msg.zones[i].polygon.points[j].longitude, msg.zones[i].polygon.points[j].altitude);
    }
  }


  /*for (int i=0; i < msg.zones.size(); i++){
    ParentProcess* area_to_add = ExclusionArea(_exclusion_areas,"", _map, "unknown"); 
    SET_CHILD_VALUE (Text, area_to_add, name, msg.zones[i].name, true)
    
    static const string area_status_name[] = {"unknown", "rfa", "nfa", "nfz", "ffa", "roz_all", "roz_ground"};
    SET_CHILD_VALUE (Text, area_to_add, status, area_status_name[msg.zones[i].type], true)

    auto* bary_summit = TaskAreaSummit(area_to_add, "bary_summit", _map, 0, 0);
    int n = msg.zones[i].polygon.points.size();
      
    double above_x = 0;
    double below_x = 0;

    double above_y = 0;
    double below_y = 0;
    
    GET_CHILD (CoreProcess, area_to_add, area)
    for (int j = 0; j < n; j++){
      auto* task_summit = TaskAreaSummit(area_to_add, std::string("summit_") + std::to_string(j), _map, msg.zones[i].polygon.points[j].latitude, msg.zones[i].polygon.points[j].longitude);
      
      SET_CHILD_VALUE (Double, task_summit, alt, msg.zones[i].polygon.points[j].altitude, true)
      auto* point = new PolyPoint(area, std::string("pt_") + std::to_string(j), 0, 0);
      GET_CHILD_VALUE (cur_lat, Double, bary_summit, lat)
      GET_CHILD_VALUE (cur_lon, Double, bary_summit, lon)
        
      SET_CHILD_VALUE (Double, bary_summit, lat, cur_lat + msg.zones[i].polygon.points[j].latitude / n, true)
      SET_CHILD_VALUE (Double, bary_summit, lon, cur_lon + msg.zones[i].polygon.points[j].longitude / n, true)
      
      new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);
      new Connector (area_to_add, "y_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);
    }

    // barycenter of polygon : https://fr-academic.com/dic.nsf/frwiki/263573
    //Compute above x : 
    for (int j = 0; j < n -1; j++){
      above_x = above_x + (msg.zones[i].polygon.points[j].latitude + msg.zones[i].polygon.points[j+ 1].latitude) * (msg.zones[i].polygon.points[j].latitude * msg.zones[i].polygon.points[j+1].longitude - msg.zones[i].polygon.points[j].longitude * msg.zones[i].polygon.points[j+1].latitude);
    }
    //Compute below x :
    for (int j = 0; j < n -1; j++){
      below_x = below_x + 3 * (msg.zones[i].polygon.points[j].latitude * msg.zones[i].polygon.points[j+1].longitude - msg.zones[i].polygon.points[j].longitude * msg.zones[i].polygon.points[j+1].latitude); 
    }
    //Compute above y
    for (int j = 0; j < n -1; j++){
      above_y = above_y + (msg.zones[i].polygon.points[j].longitude + msg.zones[i].polygon.points[j+ 1].longitude) * (msg.zones[i].polygon.points[j].latitude * msg.zones[i].polygon.points[j+1].longitude - msg.zones[i].polygon.points[j].longitude * msg.zones[i].polygon.points[j+1].latitude);
    }
    //Compute below y
    for (int j = 0; j < n -1; j++){
      below_y = below_y + 3 * (msg.zones[i].polygon.points[j].latitude * msg.zones[i].polygon.points[j+1].longitude - msg.zones[i].polygon.points[j].longitude * msg.zones[i].polygon.points[j+1].latitude); 
    }

    double res_lat = above_x/below_x;
    double res_lon = above_y/below_y;
    std::cout << "res latitude = " << res_lat << " -- longitude = " << res_lon << std::endl;

    new Connector (area_to_add, "x_bary_bind", area_to_add->find_child("bary_summit/x"), area_to_add->find_child("barycenterX"), 1);
    new Connector (area_to_add, "y_bary_bind", area_to_add->find_child("bary_summit/y"), area_to_add->find_child("barycenterY"), 1);
  }*/

  // LIMAS
  for (int i = 0; i < msg.limas.size(); i++)
  {
    int index = msg.limas[i].index;
    ParentProcess* lima = LimaModel (_lima_models, std::to_string(index), index, msg.limas[i].name, this);
      
    CoreProcess* points_list = lima->find_child("points");

    for (int j = 0; j < msg.limas[i].points.size(); j++)
    {
      PointModel (points_list, "", msg.limas[i].points[j].latitude, msg.limas[i].points[j].longitude, msg.limas[i].points[j].altitude);
    }
  }
  
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}
  
void 
RosNode::send_validation_tasks(){

  //TODO
  //message.header.stamp = _node->get_clock()->now();

}

static string frame_data;
void 
RosNode::receive_msg_map(const icare_interfaces::msg::EnvironmentMap msg){
  
  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received exploration map update\n", true)

  float lat_center = msg.origin.latitude;
  float lon_center = msg.origin.longitude; 
  float lat_center_map = msg.origin.latitude;
  float lon_center_map = msg.origin.longitude;
  
  int w = msg.width;
  int h = msg.height;

  if (_georef_visibility_map) {
    SET_CHILD_VALUE (Double, _georef_visibility_map, lat, lat_center_map, true)
    SET_CHILD_VALUE (Double, _georef_visibility_map, lon, lon_center_map, true)
  }
  
  if (_visibility_map_resolution)
    _visibility_map_resolution->set_value (msg.resolution, true);

  _visibility_map->width()->set_value (w, true);
  _visibility_map->height()->set_value (h, true);
  _visibility_map->format()->set_value(5 , true);  // DO NOT Change frame is ARGB_32 , QImage::Format_ARGB32 = 5 

  int octect = 4;
  int size_map = w*h*octect;;
      
  frame_data.reserve(size_map);

  // link frame_data to the data_image
  string*& data = _visibility_map->get_data_ref();
  data = &frame_data;

  //color:
  //ugv_camera => yellow ( #f4d03f )
  //uav_camera => purple ( #9b59b6 )
  //uav_camera && ugv_camera => cyan #7fb3d5

  for (int i = 0 ;  i < w*h ; i++ ) {
    int j0 = i*octect;
    int j1 = j0 + 1;
    int j2 = j0 + 2;
    int j3 = j0 + 3;
    if (msg.outside_area_layer[i] == 0){
      if (msg.ugv_camera_layer[i] != 0 ) {
        //yellow
        frame_data[j0] = static_cast<char>(0x3F); //B
        frame_data[j1] = static_cast<char>(0xD0); //G
        frame_data[j2] = static_cast<char>(0xF4); //R
        frame_data[j3] = static_cast<char>(0x6A); //A
      }
      if (msg.uav_camera_layer[i] != 0) {
        //purple
        frame_data[j0] = static_cast<char>(0x9B); //B
        frame_data[j1] = static_cast<char>(0x59); //G
        frame_data[j2] = static_cast<char>(0xB6); //R
        frame_data[j3] = static_cast<char>(0x6A); //A
      }
      if ((msg.ugv_camera_layer[i] != 0) && (msg.uav_camera_layer[i] != 0)) {
        //cyan
        frame_data[j0] = static_cast<char>(0xD5); //B
        frame_data[j1] = static_cast<char>(0xB3); //G
        frame_data[j2] = static_cast<char>(0x7F); //R
        frame_data[j3] = static_cast<char>(0x6A); //A
      }
      if ((msg.uav_camera_layer[i] == 0) && (msg.ugv_camera_layer[i] == 0)) {
        //blank
        frame_data[j0] = static_cast<char>(0xFF); //B
        frame_data[j1] = static_cast<char>(0xFF); //G
        frame_data[j2] = static_cast<char>(0xFF); //R
        frame_data[j3] = static_cast<char>(0x00); //A
      }
    }
  }

  //ask for draw
  _visibility_map->set_invalid_cache (true);
  _visibility_map->get_frame ()->damaged ()->activate (); // ?
      
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void 
RosNode::send_msg_trap_activation(int id, bool new_active_state){

  icare_interfaces::msg::TrapActivation msg = icare_interfaces::msg::TrapActivation();

  msg.active = new_active_state;
  msg.id = id;
  msg.header.stamp = _node->get_clock()->now();
  publisher_trap_activation->publish(msg);
  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  if (new_active_state){
    SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - Trap activation (#" +std::to_string(id) + ")\n", true)
    SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Trap activation (#" +std::to_string(id) + ")\n", true)
  }
  else {
    SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - Trap deactivation (#" +std::to_string(id) + ")\n", true)
    SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Trap deactivation (#" +std::to_string(id) + ")\n", true)
  }
}

void
RosNode::test_draw_visibility_map(){

  float lat_center_map = 44.27432196595285;
  float lon_center_map = 1.729783361205679;

  int w = 10; // for debug
  int h = 10; // for debug
  int ugv_camera_layer[100] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
                                1, 1, 0, 0, 0, 0, 0, 0, 1, 1, \
                                1, 1, 0, 0, 0, 0, 0, 0, 1, 1, \
                                1, 1, 0, 0, 1, 1, 0, 0, 1, 1, \
                                0, 0, 0, 0, 1, 1, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int uav_camera_layer[100] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                                0, 0, 0, 0, 1, 1, 0, 0, 0, 0, \
                                1, 1, 0, 0, 1, 1, 0, 0, 1, 1, \
                                1, 1, 0, 0, 0, 0, 0, 0, 1, 1, \
                                1, 1, 0, 0, 0, 0, 0, 0, 1, 1, \
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
      
  float resolution = 5; //cells are 5 meters large squares 

  if (_georef_visibility_map) {
    dynamic_cast<DoubleProperty*> (_georef_visibility_map->find_child ("lat"))->set_value (lat_center_map, true);
    dynamic_cast<DoubleProperty*> (_georef_visibility_map->find_child ("lon"))->set_value (lon_center_map, true);
  }
  if (_visibility_map_resolution)
      _visibility_map_resolution->set_value (resolution, true);
  
  _visibility_map->width()->set_value (w, true);
  _visibility_map->height()->set_value (h, true);
  _visibility_map->format()->set_value(5 , true);  // frame is ARGB_32 , QImage::Format_ARGB32 = 5 

  int octect = 4;
  int size_map = w*h*octect;;
      
  frame_data.reserve(size_map);

  // link frame_data to the data_image
  string*& data = _visibility_map->get_data_ref();
  data = &frame_data;

  //ugv_camera => yellow ( #f4d03f )
  //uav_camera => purple ( #9b59b6 )
  //uav_camera && ugv_camera => cyan #7fb3d5

  for (int i = 0 ;  i < w*h ; i++ ) {
    int j0 = i*octect;
    int j1 = j0 + 1;
    int j2 = j0 + 2;
    int j3 = j0 + 3;
    if (ugv_camera_layer[i] == 1) {
      //yellow
      frame_data[j0] = static_cast<char>(0x3F); //B
      frame_data[j1] = static_cast<char>(0xD0); //G
      frame_data[j2] = static_cast<char>(0xF4); //R
      frame_data[j3] = static_cast<char>(0xFF); //A
    }
    if (uav_camera_layer[i] == 1) {
      //purple
      frame_data[j0] = static_cast<char>(0x9B); //B
      frame_data[j1] = static_cast<char>(0x59); //G
      frame_data[j2] = static_cast<char>(0xB6); //R
      frame_data[j3] = static_cast<char>(0xFF); //A
    }
    if ((ugv_camera_layer[i] == 1) && (uav_camera_layer[i] == 1)) {
      //cyan
      frame_data[j0] = static_cast<char>(0xD5); //B
      frame_data[j1] = static_cast<char>(0xB3); //G
      frame_data[j2] = static_cast<char>(0x7F); //R
      frame_data[j3] = static_cast<char>(0xFF); //A
    }
    if ((ugv_camera_layer[i] == 0) && (uav_camera_layer[i] == 0)) {
      //blank
      frame_data[j0] = static_cast<char>(0xFF); //B
      frame_data[j1] = static_cast<char>(0xFF); //G
      frame_data[j2] = static_cast<char>(0xFF); //R
      frame_data[j3] = static_cast<char>(0x00); //A
    }
  }

  // ask for draw
  _visibility_map->set_invalid_cache (true);
  _visibility_map->get_frame ()->damaged ()->activate (); // ?
}

void
RosNode::save_console(){

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  std::stringstream ss;
  ss << timestamp + " - Console Content stored\n";

  for (auto item: ((djnn::List*)_console->find_child("ste/lines"))->children()){
    ss << ((SimpleText*)item)->get_content() << "\n";
  }
  SET_CHILD_VALUE (Text, _fw_console_input, , ss.str(), true);        
}


void
RosNode::send_msg_trap_deleted(int trap_id, bool to_delete)
{
  // if to_delete => delete trap with id trap_id

  /*icare_interfaces::msg::.... msg = icare_interfaces::msg::....();

  msg.id = id;
  msg. .... = ....;

  msg.header.stamp = _node->get_clock()->now();
  publisher_trap_activation->publish(msg);
  */

  // we should then receive a msg containing all the traps (not "deleted")
}


void
RosNode::send_msg_update_trap_position(int trap_id, double new_lat, double new_lon)
{
  std::cout << "send msg 'update position' of the trap " << trap_id << " at " << new_lat << " " << new_lon << std::endl;
  // TODO
}

#endif

