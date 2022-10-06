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

//#include <fstream>

#include "Node.h"
#include "NavGraph.h"
#include "Edge.h"
#include "math.h"
#include "trap/Trap.h"
#include "task/TaskTrap.h"
#include "task/TaskAreaSummit.h"
#include "task/TaskArea.h"
#include "task/TaskEdge.h"
#include "ExclusionArea.h"
#include "Lima.h"
using std::placeholders::_1;

using namespace djnn;

//TODO : MP
// remove - find_child
// remove set_value
// remove get_value
// check : exclussive access and release access - RETURN
// check boucle for et utilisation des msg ROS2 


RosNode::RosNode (ParentProcess* parent, const string& n, CoreProcess* my_map, CoreProcess* context) :
FatProcess (n),
ExternalSource (n),
  //arguments
_map (my_map),
_context (context),

  //navgraph fields
navgraph_data(this, "data", ""),

  //robot_state fields
_robot_id(this, "robot_id", 0),
_battery_percentage(this, "battery_percentage", 0),
_battery_voltage(this, "battery_voltage", 0),
_latitude(this, "latitude", 0),
_longitude(this, "longitude", 0),
_altitude_msl(this, "altitude_msl", 0),
_compass_heading(this, "compass_heading", 0),
_emergency_stop(this, "emergency_stop", 0),
_failsafe(this, "failsafe", 0),
_operation_mode(this, "operation_mode", 0),

  //Planif VAB
_current_plan_id_vab(this, "current_plan_id", 0),
_start_plan_vab_id(this, "start_plan_id", 0),
_end_plan_vab_id(this, "end_plan_id", 0)

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
  //subscriptions
  sub_navgraph =_node->create_subscription<icare_interfaces::msg::StringStamped>( 
  "/navgraph", qos_transient, std::bind(&RosNode::receive_msg_navgraph, this, _1)); //Replace 10 with qosbesteffort
  
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>(
    "/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, this, _1));

  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItineraryList>(
    "/itinerary", qos, std::bind(&RosNode::receive_msg_graph_itinerary_loop, this, _1));

  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
    "/plan", qos, std::bind(&RosNode::receive_msg_graph_itinerary_final, this, _1));

  sub_candidate_tasks = _node->create_subscription<icare_interfaces::msg::Tasks>(
    "/candidate_tasks", qos, std::bind(&RosNode::receive_msg_allocated_tasks, this, _1));

  sub_allocation = _node->create_subscription<icare_interfaces::msg::Allocation>(
    "/allocation", qos, std::bind(&RosNode::receive_msg_allocation, this, _1));

  sub_traps = _node->create_subscription<icare_interfaces::msg::TrapList>(
    "/traps", qos_transient, std::bind(&RosNode::receive_msg_trap, this, _1));

  sub_site = _node->create_subscription<icare_interfaces::msg::Site>(
    "/site", qos_transient, std::bind(&RosNode::receive_msg_site, this, _1));

  sub_map = _node->create_subscription<icare_interfaces::msg::EnvironmentMap>(
  "/map", qos_transient, std::bind(&RosNode::receive_msg_map, this, std::placeholders::_1));

  publisher_planning_request =_node->create_publisher<icare_interfaces::msg::PlanningRequest>(
    "/planning_request", qos);
  publisher_validation = _node->create_publisher<icare_interfaces::msg::StringStamped>(
    "/validation", qos);
  publisher_navgraph_update = _node->create_publisher<icare_interfaces::msg::StringStamped>(
    "/navgraph_update", qos_transient);
  publisher_tasks = _node->create_publisher<icare_interfaces::msg::Tasks>(
    "/tasks", qos);
  publisher_validation_tasks = _node->create_publisher<icare_interfaces::msg::StringStamped>(
    "/validate", qos);
  publisher_lima = _node->create_publisher<icare_interfaces::msg::LimaCrossed>(
    "/lima", qos);
  publisher_trap_activation = _node->create_publisher<icare_interfaces::msg::TrapActivation>(
    "/activation", qos);
  #endif

  //activate navgraph fields
  navgraph_data.activate();

  //activate robot_state fields
  _robot_id.activate();
  _battery_percentage.activate();
  _battery_voltage.activate();
  _latitude.activate();
  _longitude.activate();
  _altitude_msl.activate();
  _compass_heading.activate();
  _emergency_stop.activate();
  _failsafe.activate();
  _operation_mode.activate();

  _current_plan_id_vab.activate();
  _start_plan_vab_id.activate();
  _end_plan_vab_id.activate();

  GET_CHILD_VAR2 (_frame, CoreProcess, _parent, parent/f)
  GET_CHILD_VAR2 (_layer_filter, CoreProcess, _parent, parent/menu/ui/cb_left)

  GET_CHILD_VAR2 (_nodes, CoreProcess, _parent, parent/l/map/layers/navgraph/nodes)
  GET_CHILD_VAR2 (_edges, CoreProcess, _parent, parent/l/map/layers/navgraph/edges)
  GET_CHILD_VAR2 (_shadow_edges, CoreProcess, _parent, parent/l/map/layers/navgraph/shadow_edges)
  GET_CHILD_VAR2 (_task_edges, CoreProcess, _parent, parent/l/map/layers/tasks/tasklayer/edges)
  GET_CHILD_VAR2 (_task_areas, CoreProcess, _parent, parent/l/map/layers/tasks/tasklayer/areas)
  GET_CHILD_VAR2 (_task_traps, CoreProcess, _parent, parent/l/map/layers/tasks/tasklayer/traps)
  GET_CHILD_VAR2 (_task_allocated_edges, CoreProcess, _parent, parent/l/map/layers/allocated_tasks/allocated_tasks_layer/edges)
  GET_CHILD_VAR2 (_task_allocated_areas, CoreProcess, _parent, parent/l/map/layers/allocated_tasks/allocated_tasks_layer/areas)
  GET_CHILD_VAR2 (_task_allocated_traps, CoreProcess, _parent, parent/l/map/layers/allocated_tasks/allocated_tasks_layer/traps)
  
  GET_CHILD_VAR2 (_traps, CoreProcess, _parent, parent/l/map/layers/traps/traplayer/traps)
  GET_CHILD_VAR2 (_exclusion_areas, CoreProcess, _parent, parent/l/map/layers/site/sitelayer/exclusion_areas)
  GET_CHILD_VAR2 (_limas, CoreProcess, _parent, parent/l/map/layers/site/sitelayer/limas)
  GET_CHILD_VAR2 (_actor, CoreProcess, _parent, parent/l/map/layers/actors/sfty_pilot_uav)
  GET_CHILD_VAR2 (_actor_ugv, CoreProcess, _parent, parent/l/map/layers/actors/sfty_pilot_ugv)
  GET_CHILD_VAR2 (_clock, CoreProcess, _parent, parent/right_pannel/right_pannel/clock)
  GET_CHILD_VAR2 (_fw_input, CoreProcess, _parent, parent/right_pannel/right_pannel/clock/fw/input)
  GET_CHILD_VAR2 (_fw_console_input, CoreProcess, _parent, parent/right_pannel/right_pannel/clock/fw_console/input)
  GET_CHILD_VAR2 (_console, CoreProcess, _parent, parent/right_pannel/right_pannel/console)
  GET_CHILD_VAR2 (_itineraries_list, Component, _parent, parent/l/map/layers/itineraries/itineraries_list)
  GET_CHILD_VAR2 (_id_curent_itenerary, TextProperty, _parent, parent/l/map/layers/itineraries/id)
  GET_CHILD_VAR2 (_ref_curent_itenerary, RefProperty, _parent, parent/l/map/layers/itineraries/ref_current_itinerary)
  GET_CHILD_VAR2 (_edge_released_na, NativeAction, _parent, parent/l/map/layers/itineraries/edge_released_na)
  GET_CHILD_VAR2 (_vab, CoreProcess, _parent, parent/l/map/layers/satelites/vab)
  GET_CHILD_VAR2 (_agilex1, CoreProcess, _parent, parent/l/map/layers/satelites/agilex1)
  GET_CHILD_VAR2 (_agilex2, CoreProcess, _parent, parent/l/map/layers/satelites/agilex2)
  GET_CHILD_VAR2 (_lynx, CoreProcess, _parent, parent/l/map/layers/satelites/lynx)
  GET_CHILD_VAR2 (_spot, CoreProcess, _parent, parent/l/map/layers/satelites/spot)
  GET_CHILD_VAR2 (_drone, CoreProcess, _parent, parent/l/map/layers/satelites/drone)

  GET_CHILD_VAR2 (_current_wpt, RefProperty, _parent, parent/context/current_wpt)
  GET_CHILD_VAR2 (_entered_wpt, RefProperty, _parent, parent/context/entered_wpt)

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

  //deactivate robot_state fields
  _robot_id.deactivate();
  _battery_percentage.deactivate();
  _battery_voltage.deactivate();
  _latitude.deactivate();
  _longitude.deactivate();
  _altitude_msl.deactivate();
  _compass_heading.deactivate();
  _emergency_stop.deactivate();
  _failsafe.deactivate();
  _operation_mode.deactivate();

  _current_plan_id_vab.deactivate();
  _start_plan_vab_id.deactivate();
  _end_plan_vab_id.deactivate();

  ExternalSource::please_stop ();
}

#ifndef NO_ROS

// callback for navgraph msg (contains the navigation graph)
void 
RosNode::receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg) {
  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received new navgraph\n", true)
  
  _current_wpt->set_value ((CoreProcess*)nullptr, true);
  _entered_wpt->set_value ((CoreProcess*)nullptr, true);

    Container *_task_edge_container = dynamic_cast<Container *> (_task_edges);
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
    }

    Container *_trap_container = dynamic_cast<Container *> (_task_traps);
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

    Container *_task_container = dynamic_cast<Container *> (_task_areas);
    if (_task_container) {
      int _task_container_size = _task_container->children ().size ();
      for (int i = _task_container_size - 1; i >= 0; i--) {
        auto *item = _task_container->children ()[i];
        if (item) {
          item->deactivate ();
          if (item->get_parent ())
            item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
          item->schedule_delete ();
          item = nullptr;
        }
      }
    }

    //schedule delete old content
    int itineraries_list_size =  _itineraries_list->children ().size ();
    for (int i = itineraries_list_size - 1; i >= 0; i--) {
      auto *child = _itineraries_list->children ()[i];
      if (child) {
        child->deactivate ();
        if (child->get_parent ())
          child->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(child));
        child->schedule_delete ();
        child = nullptr;
      }
    }
    _ref_curent_itenerary->set_value ((CoreProcess*)nullptr, true);
    

  Container *_edge_container = dynamic_cast<Container *> (_edges);
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

  Container *_shadow_edges_container = dynamic_cast<Container *>( _shadow_edges);
  if (_shadow_edges_container) {
    int _shadow_edges_container_size = _shadow_edges_container->children ().size ();
    for (int i = _shadow_edges_container_size - 1; i >= 0; i--) {
      auto *item = _shadow_edges_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

  Container *_nodes_container = dynamic_cast<Container *> (_nodes);
  if (_nodes_container) {
    int _nodes_container_size = _nodes_container->children ().size ();
    for (int i = _nodes_container_size - 1; i >= 0; i--) {
      auto *item = _nodes_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

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
  
  // nodes
  for (int i=j_graph["nodes"].size() - 1; i >=0; i--){
    auto& node = j_graph["nodes"][i];
    auto& m = node["metadata"];
    bool locked = m["locked"].get<bool>();
    bool isPPO = m["compulsory"].get<bool>();
    int phase = m["phase"].get<int>();

    ParentProcess* node_ = Node (_nodes, "", _map , _frame, m["latitude"].get<double>(), m["longitude"].get<double>(), m["altitude"].get<double>(),
     isPPO, node["label"], std::stoi(node["id"].get<std::string>()) + 1, _context);
    SET_CHILD_VALUE(Bool, node_, islocked, locked, true);
    //TODO: MP problème entre le nom du child et la variable
    SET_CHILD_VALUE(Int, node_, phase, phase, true);
    SET_CHILD_VALUE(Bool, node_, wpt/isMandatory, isPPO, true)
  }

  for (auto& edge: j_graph["edges"]) {
    std::string source = edge["source"].get<std::string>();
    std::string target = edge["target"].get<std::string>();
    auto& m = edge["metadata"];
    double length =m["length"].get<double>();
    ParentProcess* edge_ = Edge(_edges, "", std::stoi(source) + 1, std::stoi(target) + 1, length, _nodes);
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

#endif

void
RosNode::test_multiple_itineraries(){
  #if 0
  //debug
  ////std::cerr << "in RosNode::test_multiple_itineraries - pointers  " << _itineraries_list  <<std::endl;

  //debug ros_msg
  std::vector<std::pair<string,std::vector<int>>> msg = { \
    {"toto", {2, 1, 0, 5, 6}}, \
    {"titi", {2, 10, 8, 6}}, \
    {"tata", {0, 1, 4, 7}}};

  //Color:
    int unselected = 0x232323;
    int selected = 0x1E90FF;

  ////std::cerr << "in RosNode::test_multiple_itineraries - size before "  << _itineraries_list->children ().size () << " - ref  " << _edge_released_na  <<std::endl;

  //schedule delete old content
    int itineraries_list_size =  _itineraries_list->children ().size ();
    for (int i = itineraries_list_size - 1; i >= 0; i--) {
      auto *child = _itineraries_list->children ()[i];
      if (child) {
        child->deactivate ();
        if (child->get_parent ())
          child->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(child));
        child->schedule_delete ();
        child = nullptr;
      }
    }
    _ref_curent_itenerary->set_value ((CoreProcess*)nullptr, true);

  ////std::cerr << "in RosNode::test_multiple_itineraries - size after "  << _itineraries_list->children ().size () <<std::endl;

  /*
    _itineraries_list {
      id { id;
          edges: liste of edges}
      id { id;
          edges: liste of edges}
      id { id;
           edges: liste of edges}
      ...
    }
  */

    string first_id = "";
    for (auto ros_itinerary : msg) {
    // get first id
      if (first_id == "")
        first_id = ros_itinerary.first;
      Component *new_itinerary = new Component ( _itineraries_list, ros_itinerary.first );
      new TextProperty (new_itinerary, "id", ros_itinerary.first);
      List* new_ite_edges = new List (new_itinerary, "edges");
      int ite_edges_size = ros_itinerary.second.size ();
      if ( ite_edges_size > 0) {
        for (int i = 1; i < ite_edges_size; i++) {
          ParentProcess* edge = Edge( new_ite_edges, "", ros_itinerary.second[i-1] + 1, ros_itinerary.second[i] + 1, 20, _nodes);
          ((AbstractProperty*) edge->find_child("color/value"))->set_value (unselected, true);
          new Binding (edge, "binding_edge_released", edge, "outerEdge/release", _edge_released_na, "");
        }
      }
    }
    _id_curent_itenerary->set_value (first_id, true);

  //debug
  //int itinerary_edges_size = dynamic_cast<IntProperty*> (_itinerary_edges->find_child ("size"))->get_value ();
  ////std::cerr << "in RosNode::test_multiple_itineraries " <<  _itinerary_edges  << " - " << itinerary_edges_size <<std::endl;
  #endif
}

#ifndef NO_ROS
void 
RosNode::receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg) {
 
  std::vector<std::pair<string,std::vector<int>>> msg_struct;
  
  if (msg->itineraries.size () <= 0)
    return;

  get_exclusive_access(DBG_GET);

  //TODO: MP - better algo - pourquoi garer msg_struct ??

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(msg->itineraries.size()) + " itineraries\n", true);
 
  for (int i = 0; i <msg->itineraries.size(); i++){
    std::string id = msg->itineraries[i].id;
    std::vector<int> nodes;

    for (int j = 0; j < msg->itineraries[i].nodes.size(); j++){
      nodes.push_back(std::stoi(msg->itineraries[i].nodes[j]));
    }
    msg_struct.push_back(std::pair<string, std::vector<int>>(id, nodes));
  }

  //Color:
  int unselected = 0x232323;
  int selected = 0x1E90FF;

  //schedule delete for old content
  int itineraries_list_size =  _itineraries_list->children ().size ();
  for (int i = itineraries_list_size - 1; i >= 0; i--) {
    auto *child = _itineraries_list->children ()[i];
    if (child) {
      child->deactivate ();
      if (child->get_parent ())
        child->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(child));
      child->schedule_delete ();
      child = nullptr;
    }
  }
  _ref_curent_itenerary->set_value ((CoreProcess*)nullptr, true);


  string first_id = "";
  for (auto ros_itinerary : msg_struct) {
  // get first id
    if (first_id == "")
      first_id = ros_itinerary.first;
    Component *new_itinerary = new Component ( _itineraries_list, ros_itinerary.first );
    new TextProperty (new_itinerary, "id", ros_itinerary.first);
    List* new_ite_edges = new List (new_itinerary, "edges");
    int ite_edges_size = ros_itinerary.second.size ();
    if ( ite_edges_size > 0) {
      for (int i = 1; i < ite_edges_size; i++) {
        ParentProcess* edge = Edge( new_ite_edges, "", ros_itinerary.second[i-1] + 1, ros_itinerary.second[i] + 1, 20, _nodes);
        SET_CHILD_VALUE (Int, edge, color/value, unselected, true)
        new Binding (edge, "binding_edge_released", edge, "outerEdge/release", _edge_released_na, "");
      }
    }
  }
  SET_CHILD_VALUE (Text, _id_curent_itenerary, , first_id, true)
  SET_CHILD_VALUE (Text, _parent, parent/right_pannel/right_pannel/itineraryPannel/first/description_input, msg->itineraries[0].description, true)
  SET_CHILD_VALUE (Text, _parent, parent/right_pannel/right_pannel/itineraryPannel/second/description_input, msg->itineraries[1].description, true)
  SET_CHILD_VALUE (Text, _parent, parent/right_pannel/right_pannel/itineraryPannel/third/description_input, msg->itineraries[2].description, true)
  SET_CHILD_VALUE (Text, _parent, parent/right_pannel/right_pannel/itineraryPannel/first/itinerary_id, msg->itineraries[0].id, true)
  SET_CHILD_VALUE (Text, _parent, parent/right_pannel/right_pannel/itineraryPannel/second/itinerary_id,msg->itineraries[1].id, true )
  SET_CHILD_VALUE (Text, _parent, parent/right_pannel/right_pannel/itineraryPannel/third/itinerary_id, msg->itineraries[2].id, true)
  
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void 
RosNode::receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {

  get_exclusive_access(DBG_GET);
  
  //schedule delete old content
  int itineraries_list_size =  _itineraries_list->children ().size ();
  for (int i = itineraries_list_size - 1; i >= 0; i--) {
    auto *child = _itineraries_list->children ()[i];
    if (child) {
      child->deactivate ();
      if (child->get_parent ())
        child->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(child));
      child->schedule_delete ();
      child = nullptr;
    }
  }
  _ref_curent_itenerary->set_value ((CoreProcess*)nullptr, true);

  //Color:
  int unselected = 0x232323;
  int selected = 0x1E90FF;

  Component *new_itinerary = new Component ( _itineraries_list, msg->id );
  new TextProperty (new_itinerary, "id", msg->id);
  List* new_ite_edges = new List (new_itinerary, "edges");
  int ite_edges_size = msg->nodes.size ();
  if ( ite_edges_size > 0) {
    for (int i = 1; i < ite_edges_size; i++) {
      ParentProcess* edge = Edge( new_ite_edges, "", std::stoi(msg->nodes[i-1]) + 1,std::stoi(msg->nodes[i]) + 1, 20, _nodes);
      SET_CHILD_VALUE (Int, edge, color/value, selected, true)
      //new Binding (edge, "binding_edge_released", edge, "edge/release", _edge_released_na, "");
    }
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void 
RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState::SharedPtr msg) {
  
  RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);

  djnn::Process * robots[] = {nullptr, _drone, _agilex1, _agilex2, _lynx, _spot, _vab, _actor, _actor_ugv};
  static const string robots_name[] = {"", "drone", "agilex1", "agilex2", "lynx", "spot", "vab", "drone_safety_pilot", "ground_safety_pilot"};
  if (msg->robot_id<1 || msg->robot_id>=sizeof(robots)) {
    RCLCPP_INFO(_node->get_logger(), "incorrect robot_id: '%d'", msg->robot_id);
    return;
  }
    
  djnn::Process * robot = robots[msg->robot_id];
  const string& robot_name = robots_name[msg->robot_id];
  //assert(robot);
  //assert (robot_name);

  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);

  SET_CHILD_VALUE (Double, robot, lat, msg->position.latitude, true);
  SET_CHILD_VALUE (Double, robot, lon, msg->position.longitude, true);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Received robot_state for " + robot_name + "\n", true);
  if(robot != _actor && robot != _actor_ugv ) {
    SET_CHILD_VALUE (Double, robot, altitude_msl, msg->position.altitude, true);
    SET_CHILD_VALUE (Double, robot, heading_rot, msg->compass_heading, true);
    SET_CHILD_VALUE (Int, robot, battery_percentage, msg->battery_percentage, true);
    SET_CHILD_VALUE (Int, robot, operation_mode, msg->operating_mode, true); // FIXME: operation_mode vs operating_mode
    SET_CHILD_VALUE (Bool, robot, emergency_stop, msg->emergency_stop, true);
    SET_CHILD_VALUE (Bool, robot, failsafe, msg->failsafe, true);
  }

  _latitude.set_value (msg -> position.latitude, true);
  _longitude.set_value (msg -> position.longitude, true);
  _robot_id.set_value (msg -> robot_id, true);
  _battery_percentage.set_value ( msg -> battery_percentage, true);
  _battery_voltage.set_value (msg -> battery_voltage, true);
  _compass_heading.set_value (msg -> compass_heading, true);
  _emergency_stop.set_value (msg -> emergency_stop, true);
  _failsafe.set_value (msg -> failsafe, true);
  _operation_mode.set_value (msg -> operating_mode, true);
  _altitude_msl.set_value (msg -> altitude_msl, true);

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void 
RosNode::receive_msg_trap (const icare_interfaces::msg::TrapList msg){
    
  get_exclusive_access(DBG_GET);
  
  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  
  int new_trap = 0;
  int update_trap = 0;
  CoreProcess *current_trap;
  
  for (int k= 0; k < msg.traps.size(); k ++){

    int index_found = -1;
    current_trap = nullptr;

    Container *_traps_container = dynamic_cast<Container *> (_traps);

    for (int i = 0; i < _traps_container->children().size(); i++){
      GET_CHILD_VALUE (id, Int, _traps_container->children()[i], id)
      if (id == msg.traps[k].id){
        index_found = i;
        break;
      }
    }

    if (index_found == -1) {
      new_trap = new_trap + 1;
      
      ParentProcess *new_trap = Trap(_traps, "", _map, msg.traps[k].location.latitude, msg.traps[k].location.longitude, msg.traps[k].id, this);
  
      current_trap = new_trap;

      SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - New trap #" + std::to_string(msg.traps[k].id) + "\n", true);
      
      if (msg.traps[k].identified)
        SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - New trap identified "+ msg.traps[k].info.id +"(#" +std::to_string(msg.traps[k].id) + ")" +  " " + msg.traps[k].info.code  + " " + msg.traps[k].info.hazard + "\n", true);
    }
    else {
      
      update_trap = update_trap + 1;

      if (msg.traps[k].identified && !((BoolProperty*)_traps_container->children()[index_found]->find_child("identified"))->get_value())
        SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp + " - trap identified "+ msg.traps[k].info.id +"(#" +std::to_string(msg.traps[k].id) + ")" +  " " + msg.traps[k].info.code  + " " + msg.traps[k].info.hazard + "\n", true)
      else if(msg.traps[k].identified)
        SET_CHILD_VALUE (Text, _console, ste/string_input, timestamp+ " - trap updated "+ msg.traps[k].info.id +"(#" +std::to_string(msg.traps[k].id) + ")" +  " " + msg.traps[k].info.code  + " " + msg.traps[k].info.hazard + "\n", true)
      
      current_trap = _traps_container->children()[index_found];

    }

    SET_CHILD_VALUE (Bool, current_trap, active, msg.traps[k].active, true)
    SET_CHILD_VALUE (Bool, current_trap, identified, msg.traps[k].identified, true)
    SET_CHILD_VALUE (Text, current_trap, trap_id, msg.traps[k].info.id, true)
    SET_CHILD_VALUE (Text, current_trap, description, msg.traps[k].info.description, true)
    SET_CHILD_VALUE (Int, current_trap, contact_mode, msg.traps[k].info.contact_mode, true)
    SET_CHILD_VALUE (Text, current_trap, code, msg.traps[k].info.code, true)
    SET_CHILD_VALUE (Text, current_trap, hazard, msg.traps[k].info.hazard, true)
    //Todo: MP - remove-  give radius control by trap_sw_state 
    //SET_CHILD_VALUE (Double, current_trap, radius, msg.traps[k].info.radius, true) 
    SET_CHILD_VALUE (Bool, current_trap, remotely_deactivate, msg.traps[k].info.remotely_deactivate, true)
    SET_CHILD_VALUE (Bool, current_trap, contact_deactivate, msg.traps[k].info.contact_deactivate, true)

    GRAPH_EXEC;
  }
  
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(msg.traps.size()) + " traps (" + std::to_string(new_trap) + " new, " + std::to_string(update_trap) + " updated)\n" , true);
      
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void 
RosNode::receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks msg){

  get_exclusive_access(DBG_GET);

  Container *_edge_container = dynamic_cast<Container *> (_task_edges);
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

  Container *_trap_container = dynamic_cast<Container *> (_task_traps);
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

  Container *_task_container = dynamic_cast<Container *> (_task_areas);
  if (_task_container) {
    int _task_container_size = _task_container->children ().size ();
    for (int i = _task_container_size - 1; i >= 0; i--) {
      auto *item = _task_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }
  
  int nb_uav_zone = msg.uav_zones.size();
  int nb_ugv_edges = msg.ugv_edges.size();
  int nb_trap_identification = msg.trap_identifications.size();  
  int nb_trap_deactivation = msg.trap_deactivations.size();
  int nb_total = nb_uav_zone + nb_ugv_edges + nb_trap_deactivation + nb_trap_identification;
  
  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(nb_total) + " tasks ("+ std::to_string(nb_uav_zone) + " uav_zones, " + std::to_string(nb_ugv_edges) + " ugv_edges, " + std::to_string(nb_trap_identification) + " trap_identifications, " + std::to_string(nb_trap_deactivation) + " trap_deactivations)\n", true)
 

  for (int i=0; i < msg.uav_zones.size(); i++){
    ParentProcess* area_to_add = TaskArea(_task_areas , "", _map);
    SET_CHILD_VALUE (Double, area_to_add, area_prop, msg.uav_zones[i].area, true)
    SET_CHILD_VALUE (Double, area_to_add, explored, msg.uav_zones[i].explored, true)
    
    for (int j = 0; j < msg.uav_zones[i].points.size(); j++){
      auto* task_summit = TaskAreaSummit (area_to_add, std::string("summit_") + std::to_string(j), _map, msg.uav_zones[i].points[j].latitude, msg.uav_zones[i].points[j].longitude);
      SET_CHILD_VALUE (Double, task_summit, alt, msg.uav_zones[i].points[j].altitude, true)
      auto* point = new PolyPoint(area_to_add->find_child("area"), std::string("pt_") + std::to_string(j), 0, 0);

      new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);
      new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);
    }
    SET_CHILD_VALUE (Int, area_to_add, nb_summit, (int) (msg.uav_zones[i].points.size()), true)
  }

  for (int i=0; i < msg.ugv_edges.size(); i++){
    //Create ugv_edges
    ParentProcess* edge_to_add = TaskEdge(_task_edges, "", _map, std::stoi(msg.ugv_edges[i].source) + 1, std::stoi(msg.ugv_edges[i].target) + 1, _nodes);
    
    SET_CHILD_VALUE (Double, edge_to_add, length, msg.ugv_edges[i].length, true)
    SET_CHILD_VALUE (Double, edge_to_add, explored, msg.ugv_edges[i].explored, true)
  }
  
  for (int i=0; i <msg.trap_identifications.size(); i++){
    
    //debug
    //std::cerr << "trying to add a trap_identification at " + std::to_string(msg.trap_identifications[i].location.latitude) << std::endl;
    
    ParentProcess* trap_to_add = TaskTrap(_task_traps, "", _map, msg.trap_identifications[i].id, msg.trap_identifications[i].location.latitude, msg.trap_identifications[i].location.longitude);
    SET_CHILD_VALUE (Bool, trap_to_add, active, msg.trap_identifications[i].active, true)
    SET_CHILD_VALUE (Bool, trap_to_add, identified, msg.trap_identifications[i].identified, true)
    SET_CHILD_VALUE (Text, trap_to_add, trap_id_str, msg.trap_identifications[i].info.id, true)
    SET_CHILD_VALUE (Text, trap_to_add, description, msg.trap_identifications[i].info.description, true)
    SET_CHILD_VALUE (Int, trap_to_add, contact_mode, msg.trap_identifications[i].info.contact_mode, true)
    SET_CHILD_VALUE (Text, trap_to_add, code, msg.trap_identifications[i].info.code, true)
    SET_CHILD_VALUE (Text, trap_to_add, hazard, msg.trap_identifications[i].info.hazard, true)
    SET_CHILD_VALUE (Double, trap_to_add, radius, msg.trap_identifications[i].info.radius, true)
  }
  
  for (int i=0; i<msg.trap_deactivations.size(); i++){
    
    //debug
    //std::cerr << "trying to add a trap_deactivation" << std::endl;

    ParentProcess* trap_to_add = TaskTrap(_task_traps, "", _map, msg.trap_deactivations[i].id, msg.trap_deactivations[i].location.latitude, msg.trap_deactivations[i].location.longitude);
    SET_CHILD_VALUE (Bool, trap_to_add, active, msg.trap_deactivations[i].active, true)
    SET_CHILD_VALUE (Bool, trap_to_add, identified, msg.trap_deactivations[i].identified, true)
    SET_CHILD_VALUE (Text, trap_to_add, trap_id_str, msg.trap_deactivations[i].info.id, true)
    SET_CHILD_VALUE (Text, trap_to_add, description, msg.trap_deactivations[i].info.description, true)
    SET_CHILD_VALUE (Int, trap_to_add, contact_mode, msg.trap_deactivations[i].info.contact_mode, true)
    SET_CHILD_VALUE (Text, trap_to_add, code, msg.trap_deactivations[i].info.code, true)
    SET_CHILD_VALUE (Text, trap_to_add, hazard, msg.trap_deactivations[i].info.hazard, true)
    SET_CHILD_VALUE (Double, trap_to_add, radius, msg.trap_deactivations[i].info.radius, true)
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


void
RosNode::activate_layer(std::string layer_to_activate){

  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container){
    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--) {
      auto *child = _layer_filter_container->children ()[i];
      GET_CHILD_VALUE (layer_name, Text, child, name)
      if (layer_name == layer_to_activate){
        GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
        if (activation_state == "hidden"){
          child->find_child("cb/press")->activate();
        }
      }
    }
  }
}

void
RosNode::deactivate_layer(std::string layer_to_deactivate){

  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container){
    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--) {
      auto *child = _layer_filter_container->children ()[i];
      GET_CHILD_VALUE (layer_name, Text, child, name)
      if (layer_name == layer_to_deactivate){
        GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
        if (activation_state == "visible"){
          child->find_child("cb/press")->activate();
        }
      }
    }
  }
}


void 
RosNode::receive_msg_allocation(const icare_interfaces::msg::Allocation msg){
    

  get_exclusive_access(DBG_GET);

  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container){

    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--) {
        auto *child = _layer_filter_container->children ()[i];
        GET_CHILD_VALUE (layer_name, Text, child, name)
        std::cerr << "found layer" << layer_name << std::endl; 
        if (layer_name == "Tasks"){
          std::cerr << "found task layer" << std::endl;
          GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
          if (activation_state == "visible"){
            std::cerr << "tasklayer is visible" << std::endl;
            child->find_child("cb/press")->notify_activation();
            std::cerr << "notified activation to cb/press" << std::endl;
          }
        }

        if (layer_name == "Allocation"){
          std::cerr << "found Allocation layer" << std::endl;
          GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
          if (activation_state == "hidden"){
            std::cerr << "Allocation layer is hidden" << std::endl;
            child->find_child("cb/press")->notify_activation();
            std::cerr << "notified activation to cb/press" << std::endl;
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

  Container *_task_container = dynamic_cast<Container *> (_task_allocated_areas);
  if (_task_container) {
    int _task_container_size = _task_container->children ().size ();
    for (int i = _task_container_size - 1; i >= 0; i--) {
      auto *item = _task_container->children ()[i];
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
  //Allocation = list of Allocated tasks
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
  
  /*GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text)
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received " + std::to_string(nb_total) + " tasks ("+ std::to_string(nb_uav_zone) + " uav_zones, " + std::to_string(nb_ugv_edges) + " ugv_edges, " + std::to_string(nb_trap_identification) + " trap_identifications, " + std::to_string(nb_trap_deactivation) + " trap_deactivations)\n", true)
 */


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
  for (int i=0; i < nb_total; i++){
    
    if (msg.tasks[i].task_type == 1){
      ParentProcess* area_to_add = TaskArea(_task_allocated_areas, "", _map);
      for (int j=0 ;j< msg.tasks[i].zone.points.size(); j++){
        auto* task_summit = TaskAreaSummit (area_to_add, std::string("summit_") + std::to_string(j), _map, msg.tasks[i].zone.points[j].latitude, msg.tasks[i].zone.points[j].longitude);
        SET_CHILD_VALUE (Double, task_summit, alt, msg.tasks[i].zone.points[j].altitude, true)
        auto* point = new PolyPoint(area_to_add->find_child("area"), std::string("pt_") + std::to_string(j), 0, 0);

        new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);
        new Connector (area_to_add, "y_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);
    
      }
      SET_CHILD_VALUE (Int, area_to_add, nb_summit, (int) (msg.tasks[i].zone.points.size()), true)
      SET_CHILD_VALUE (Int, area_to_add, color/value, colors[msg.tasks[i].robot_id], true)
    } else if (msg.tasks[i].task_type == 2){

      ParentProcess* edge_to_add = TaskEdge(_task_allocated_edges, "", _map, std::stoi(msg.tasks[i].edge.source) + 1, std::stoi(msg.tasks[i].edge.target) + 1, _nodes);
    
      SET_CHILD_VALUE (Double, edge_to_add, length, msg.tasks[i].edge.length, true)
      SET_CHILD_VALUE (Double, edge_to_add, explored, msg.tasks[i].edge.explored, true)
      SET_CHILD_VALUE (Int, edge_to_add, the_edge/color/value, colors[msg.tasks[i].robot_id], true);

    } /*else if (msg.tasks[i].task_type == 3){
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


    } else if (msg.tasks[i].task_type == 4){
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
    }
*/






  }
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void
RosNode::send_msg_lima(int id){

  icare_interfaces::msg::LimaCrossed message = icare_interfaces::msg::LimaCrossed();
  message.id = id;

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Validated lima " + std::to_string(id) + "\n", true);
  
  message.header.stamp = _node->get_clock()->now();

  publisher_lima->publish(message);
}


void
RosNode::send_msg_planning_request(){
    
  icare_interfaces::msg::PlanningRequest message = icare_interfaces::msg::PlanningRequest();
  message.id = _current_plan_id_vab.get_string_value();

  for (auto item: ((djnn::List*)_nodes)->children()){

    GET_CHILD_VALUE (status, Text, item, status)
    GET_CHILD_VALUE (iid, Int, item, id)

    if (status == "start" )
      message.start_node = std::to_string(iid - 1);
    else if ( status == "end")
      message.end_node = std::to_string(iid - 1);
    else if (status == "forced")
        message.node_contraints.push_back(std::to_string(iid -1));
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

  //Edges
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

  //Nodes
  for (auto item: ((djnn::List*)nodes)->children()){

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

void 
RosNode::send_validation_plan(){

  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  message.data = _id_curent_itenerary->get_string_value();
  message.header.stamp = _node->get_clock()->now();
  publisher_validation->publish(message);


  Container *_layer_filter_container = dynamic_cast<Container *> (_layer_filter);
  if (_layer_filter_container){

    int layer_size = _layer_filter_container->children ().size ();
    for (int i = layer_size - 1; i >= 0; i--) {
        auto *child = _layer_filter_container->children ()[i];
        GET_CHILD_VALUE (layer_name, Text, child, name)
        std::cerr << "found layer" << layer_name << std::endl; 
        if (layer_name == "Tasks"){
          std::cerr << "found task layer" << std::endl;
          GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
          if (activation_state == "hidden"){
            std::cerr << "tasklayer is hidden" << std::endl;
            child->find_child("cb/press")->notify_activation();
            std::cerr << "notified activation to cb/press" << std::endl;
          }
        }
        if (layer_name == "Allocation"){
          std::cerr << "found Allocation layer" << std::endl;
          GET_CHILD_VALUE (activation_state, Text, child, cb/fsm/state)
          if (activation_state == "visible"){
            std::cerr << "Allocation layer is visible" << std::endl;
            child->find_child("cb/press")->notify_activation();
            std::cerr << "notified activation to cb/press" << std::endl;
          }
        }

    }

  }

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Validate plan #" + message.data + "\n" , true);

}

void 
RosNode::send_selected_tasks(){

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Send task selection\n", true);

  icare_interfaces::msg::Tasks message = icare_interfaces::msg::Tasks();

  for (auto trap: ((djnn::List*)_task_traps)->children()){
    GET_CHILD_VALUE (trap_selected, Bool, trap, selected)
    if (trap_selected){     
      icare_interfaces::msg::Trap trap_to_add = icare_interfaces::msg::Trap();
      GET_CHILD_VALUE2 (trap_to_add.id, Int, trap, trap_id)
      GET_CHILD_VALUE2 (trap_to_add.identified, Bool, trap, identified)
      GET_CHILD_VALUE2 (trap_to_add.active, Bool, trap, active)
      GET_CHILD_VALUE2 (trap_to_add.location.latitude, Double, trap, lat)
      GET_CHILD_VALUE2 (trap_to_add.location.longitude, Double, trap, lon)
      
      if(trap_to_add.identified)
        message.trap_deactivations.push_back(trap_to_add);
      else
        message.trap_identifications.push_back(trap_to_add);
    }
  }

  for (auto edge: ((djnn::List*)_task_edges)->children()){
    GET_CHILD_VALUE (edge_selected, Bool, edge, selected)
    if (edge_selected){
      icare_interfaces::msg::GraphEdge edge_to_add = icare_interfaces::msg::GraphEdge();
      GET_CHILD_VALUE (source, Int, edge, id_source)
      edge_to_add.source = std::to_string(source - 1);
      GET_CHILD_VALUE (dest, Int, edge, id_dest)
      edge_to_add.target = std::to_string(dest - 1);
      GET_CHILD_VALUE2 (edge_to_add.length, Double, edge, length)
      GET_CHILD_VALUE2 (edge_to_add.explored, Double, edge, explored)
      message.ugv_edges.push_back(edge_to_add);
    }
  }

  for (auto area: ((djnn::List*)_task_areas)->children()){
    GET_CHILD_VALUE (area_selected, Bool, area, selected)
    if (area_selected){
      icare_interfaces::msg::ExplorationPolygon geopolygon_to_add = icare_interfaces::msg::ExplorationPolygon();
      GET_CHILD_VALUE2 (geopolygon_to_add.area, Double, area, area_prop)
      GET_CHILD_VALUE2 (geopolygon_to_add.explored, Double, area, explored)
      
      GET_CHILD_VALUE (nb_summit, Int, area, nb_summit)
      for (int i = 0; i < nb_summit; i++){
        geographic_msgs::msg::GeoPoint point_to_add = geographic_msgs::msg::GeoPoint();
        CoreProcess* summit = dynamic_cast<CoreProcess*>(area->find_child(std::string("summit_") + std::to_string(i)));
        GET_CHILD_VALUE2 (point_to_add.latitude, Double, summit, lat)
        GET_CHILD_VALUE2 (point_to_add.longitude, Double, summit, lon)
        GET_CHILD_VALUE2 (point_to_add.altitude, Double, summit, alt)
        geopolygon_to_add.points.push_back(point_to_add);
      }

      message.uav_zones.push_back(geopolygon_to_add);
    }
  }

  message.header.stamp = _node->get_clock()->now();
  publisher_tasks->publish(message);
}

void 
RosNode::receive_msg_site(const icare_interfaces::msg::Site msg){

  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _clock, wc/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received site data\n", true);

  ParentProcess *limits_to_add = ExclusionArea(_exclusion_areas, "", _map, "limits");
    
  for (int i=0; i<msg.limits.points.size(); i++){
    auto* limit_summit = TaskAreaSummit(limits_to_add, std::string("summit_") + std::to_string(i), _map, msg.limits.points[i].latitude, msg.limits.points[i].longitude);
    SET_CHILD_VALUE (Double, limit_summit, alt, msg.limits.points[i].altitude, true)
    auto* point = new PolyPoint(limits_to_add->find_child("area"), std::string("pt_") + std::to_string(i), 0, 0);
    new Connector (limits_to_add, "x_bind", limits_to_add->find_child(std::string("summit_") + std::to_string(i) + std::string("/x")), limits_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(i) + std::string("/x")), 1);
    new Connector (limits_to_add, "y_bind", limits_to_add->find_child(std::string("summit_") + std::to_string(i) + std::string("/y")), limits_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(i) + std::string("/y")), 1);
  }

  for (int i=0; i < msg.zones.size(); i++){
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
    //Compute abovex : 
    for (int j = 0; j < n -1; j++){
      above_x = above_x + (msg.zones[i].polygon.points[j].latitude + msg.zones[i].polygon.points[j+ 1].latitude) * (msg.zones[i].polygon.points[j].latitude * msg.zones[i].polygon.points[j+1].longitude - msg.zones[i].polygon.points[j].longitude * msg.zones[i].polygon.points[j+1].latitude);
    }
    //Compute belowx :
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
    std::cerr << "res_latitude = " << res_lat << std::endl;
    std::cerr << "res_longitude = " << res_lon << std::endl;

    new Connector (area_to_add, "x_bary_bind", area_to_add->find_child("bary_summit/x"), area_to_add->find_child("barycenterX"), 1);
    new Connector (area_to_add, "y_bary_bind", area_to_add->find_child("bary_summit/y"), area_to_add->find_child("barycenterY"), 1);
  }

  for (int i=0; i < msg.limas.size(); i++){
    ParentProcess *lima_to_add = Lima(_limas, "", _map, this);
    SET_CHILD_VALUE (Int, lima_to_add, id, msg.limas[i].index, true)
    SET_CHILD_VALUE (Text, lima_to_add, name, msg.limas[i].name, true)
  
    for (int j = 0; j < msg.limas[i].points.size(); j++){
      auto* task_summit = TaskAreaSummit(lima_to_add, std::string("summit_") + std::to_string(j), _map, msg.limas[i].points[j].latitude, msg.limas[i].points[j].longitude);
      SET_CHILD_VALUE (Double, task_summit, alt, msg.limas[i].points[j].altitude, true)
      auto* point = new PolyPoint(lima_to_add->find_child("lima"), std::string("pt_") + std::to_string(j), 0, 0);
      new Connector (lima_to_add, "x_bind", lima_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), lima_to_add->find_child(std::string("lima/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);
      new Connector (lima_to_add, "y_bind", lima_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), lima_to_add->find_child(std::string("lima/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);
       
      /*  
        new NativeAction (lima_to_add, "send_lima_id_na", RosNode::send_msg_lima, msg.limas[i].index, true);
        new Binding (lima_to_add, "lima_pressed", lima_to_add, "pressed", lima_to_add, "send_lima_id_na");
      */

      //TODO : 
      //binding : lima.press -> send correct lima id
      /*   
        new Binding (edge, "binding_edge_released", edge, "edge/release", _edge_released_na, "");
        new Binding (lima_to_add, "binding_edge_released", lima, "press", _lima_released_na, "");
      */
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
RosNode::send_msg_trap_deleted(int trap_id, bool to_delete){
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
RosNode::send_msg_update_trap_position(int trap_id, double new_lat, double new_lon){
//TODO

  std::cerr << new_lat << " " << new_lon << std::endl;



}

#endif

void
RosNode::run () {
#ifndef NO_ROS
  rclcpp::spin(_node);
  rclcpp::shutdown();
#endif
}
