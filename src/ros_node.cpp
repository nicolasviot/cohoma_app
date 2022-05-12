#include "ros_node.h"

#include "exec_env/global_mutex.h"
//#include "core/execution/graph.h"
#include "core/control/binding.h"
#include "core/ontology/coupling.h"
#include "base/connector.h"
#include "core/core-dev.h"
#include "core/tree/list.h"
#include "gui/shape/poly.h"

#include <nlohmann/json.hpp>


//#include <iostream>

//#include <fstream>

#include "Node.h"
#include "NavGraph.h"
#include "Edge.h"
#include "math.h"
#include "Trap.h"
#include "TaskTrap.h"
#include "TaskAreaSummit.h"
#include "TaskArea.h"
#include "TaskEdge.h"
#include "ExclusionArea.h"
#include "Lima.h"
using std::placeholders::_1;

using namespace djnn;



RosNode::RosNode (ParentProcess* parent, const string& n, CoreProcess* my_map, CoreProcess* manager) :
FatProcess (n),
ExternalSource (n),
  //arguments
_map (my_map),
_manager (manager),

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
  "map", qos_transient, std::bind(&RosNode::receive_msg_map, this, std::placeholders::_1));


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

  _nodes = _parent->find_child ("parent/l/map/layers/navgraph/nodes");
  _edges = _parent->find_child ("parent/l/map/layers/navgraph/edges");
  _shadow_edges = _parent->find_child ("parent/l/map/layers/navgraph/shadow_edges");
  _task_edges = _parent->find_child("parent/l/map/layers/tasks/tasklayer/edges");
  _task_areas = _parent->find_child("parent/l/map/layers/tasks/tasklayer/areas");
  _task_traps = _parent->find_child("parent/l/map/layers/tasks/tasklayer/traps");
  _traps = _parent->find_child("parent/l/map/layers/traps/traplayer/traps");
  _exclusion_areas = _parent->find_child("parent/l/map/layers/site/sitelayer/exclusion_areas");
  _limas = _parent->find_child("parent/l/map/layers/site/sitelayer/limas");
  _frame = _parent->find_child("parent/f");
  _actor = _parent->find_child("parent/l/map/layers/actors/sfty_pilot");
  _clock = _parent->find_child("parent/right_pannel/right_pannel/clock");
  _console = _parent->find_child("parent/right_pannel/right_pannel/console");
  _itineraries_list = dynamic_cast<Component*> (_parent->find_child("parent/l/map/layers/itineraries/itineraries_list"));
  _id_curent_itenerary  = dynamic_cast<TextProperty*> (_parent->find_child ("parent/l/map/layers/itineraries/id"));
  _ref_curent_itenerary = dynamic_cast<RefProperty*> (_parent->find_child ("parent/l/map/layers/itineraries/ref_current_itinerary"));
  _edge_released_na = dynamic_cast<NativeAction*> (_parent->find_child ("parent/l/map/layers/itineraries/edge_released_na"));
  //_send_lima_na = dynamic_cast<NativeAction*> (_parent->find_child("parent/ros_manager/send_lima_na"))
  _vab = _parent->find_child("parent/l/map/layers/satelites/vab");
  _agilex1 = _parent->find_child("parent/l/map/layers/satelites/agilex1");
  _agilex2 = _parent->find_child("parent/l/map/layers/satelites/agilex2");
  _lynx = _parent->find_child("parent/l/map/layers/satelites/lynx");
  _spot = _parent->find_child("parent/l/map/layers/satelites/spot");
  _drone = _parent->find_child("parent/l/map/layers/satelites/drone");

  _current_wpt = dynamic_cast<RefProperty*> (_parent->find_child ("parent/l/map/layers/navgraph/manager/current_wpt"));
  _entered_wpt = dynamic_cast<RefProperty*> (_parent->find_child ("parent/l/map/layers/navgraph/manager/entered_wpt"));



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

  _current_wpt->set_value ((CoreProcess*)nullptr, true);
  _entered_wpt->set_value ((CoreProcess*)nullptr, true);

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
    if (j.size() > 1) {
      std::cerr << "Several graphs defined in the JSON structure! loading the first one..." << std::endl;
      j_graph = j["graphs"][0];
    }
    else if (j.size() == 0) {
      std::cerr << "No graph defined in the JSON structure!" << std::endl;
      return;
    }
  }
    //std::cerr << "about to get graph attributes" << std::endl;
    // graph attributes
  if (j_graph.contains("directed") && j_graph["directed"].get<bool>()) {
    std::cerr << "graph is said to be directed! NavGraph are only undirected: the results graph may not be what expected!" << std::endl;
  }
    //std::cerr << "about to parse nodes" << std::endl;
    // nodes
  for (int i=j_graph["nodes"].size() - 1; i >=0; i--){
    //for (auto& node: j_graph["nodes"]) {
    auto& node = j_graph["nodes"][i];
        //std::cerr << "in from json parsing nodes" << std::endl;
    auto& m = node["metadata"];
    bool locked = m["locked"].get<bool>();
    bool isPPO = m["compulsory"].get<bool>();
    if (isPPO){
      std::cerr << "one more PPO imported" << std::endl;
    }
    ParentProcess* node_ = Node(_nodes, "", _map , _frame, m["latitude"].get<double>(), m["longitude"].get<double>(), m["altitude"].get<double>(),
     isPPO, node["label"], std::stoi(node["id"].get<std::string>()) + 1, _manager);
    ((BoolProperty*)node_->find_child("islocked"))->set_value(locked, true);

  }
  std::cerr << "about to parse edges" << std::endl;
    // edges
  for (auto& edge: j_graph["edges"]) {


    std::string source = edge["source"].get<std::string>();
    std::string target = edge["target"].get<std::string>();
    auto& m = edge["metadata"];
    double length =m["length"].get<double>();
    ParentProcess* edge_ = Edge(_edges, "", std::stoi(source) + 1, 
      std::stoi(target) + 1,length, _nodes);

  }
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

#endif
void
RosNode::test_multiple_itineraries(){
  #if 0
  //debug
  //std::cerr << "in RosNode::test_multiple_itineraries - pointers  " << _itineraries_list  <<std::endl;

  //debug ros_msg
  std::vector<std::pair<string,std::vector<int>>> msg = { \
    {"toto", {2, 1, 0, 5, 6}}, \
    {"titi", {2, 10, 8, 6}}, \
    {"tata", {0, 1, 4, 7}}};

  //Color:
    int unselected = 0x232323;
    int selected = 0x1E90FF;

  //std::cerr << "in RosNode::test_multiple_itineraries - size before "  << _itineraries_list->children ().size () << " - ref  " << _edge_released_na  <<std::endl;

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

  //std::cerr << "in RosNode::test_multiple_itineraries - size after "  << _itineraries_list->children ().size () <<std::endl;

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
  //std::cerr << "in RosNode::test_multiple_itineraries " <<  _itinerary_edges  << " - " << itinerary_edges_size <<std::endl;
  #endif
  }
#ifndef NO_ROS
  void 
  RosNode::receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg) {
 //debug
  //std::cerr << "in RosNode::test_multiple_itineraries - pointers  " << _itineraries_list  <<std::endl;

  get_exclusive_access(DBG_GET);
  //debug ros_msg
  std::vector<std::pair<string,std::vector<int>>> msg_struct; /*= { \
    {msg->itineraries[0]->id, {}, \
    {msg->itineraries[1]->id, {}}, \
    {msg->itineraries[2]->id, {}}};
*/
    if (msg->itineraries.size()<= 0)
      return;

    for (int i = 0; i <msg->itineraries.size(); i++){
      std::string id = msg->itineraries[i].id;
  //  std::string description = msg->itineraries[i]->description;
      std::vector<int> nodes;

      for (int j = 0; j < msg->itineraries[i].nodes.size(); j++){
        nodes.push_back(std::stoi(msg->itineraries[i].nodes[j]));
      }
      msg_struct.push_back(std::pair<string, std::vector<int>>(id, nodes));

    }

  //Color:
    int unselected = 0x232323;
    int selected = 0x1E90FF;

  //std::cerr << "in RosNode::test_multiple_itineraries - size before "  << _itineraries_list->children ().size () << " - ref  " << _edge_released_na  <<std::endl;

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
          ((AbstractProperty*) edge->find_child("color/value"))->set_value (unselected, true);
          new Binding (edge, "binding_edge_released", edge, "outerEdge/release", _edge_released_na, "");
        }
      }
    }
    _id_curent_itenerary->set_value (first_id, true);
    ((AbstractProperty*)_parent->find_child("parent/right_pannel/right_pannel/itineraryPannel/first/description_input"))->set_value(msg->itineraries[0].description, true);
    ((AbstractProperty*)_parent->find_child("parent/right_pannel/right_pannel/itineraryPannel/second/description_input"))->set_value(msg->itineraries[1].description, true);
    ((AbstractProperty*)_parent->find_child("parent/right_pannel/right_pannel/itineraryPannel/third/description_input"))->set_value(msg->itineraries[2].description, true);
    ((AbstractProperty*)_parent->find_child("parent/right_pannel/right_pannel/itineraryPannel/first/itinerary_id"))->set_value(msg->itineraries[0].id, true);
    ((AbstractProperty*)_parent->find_child("parent/right_pannel/right_pannel/itineraryPannel/second/itinerary_id"))->set_value(msg->itineraries[1].id, true);
    ((AbstractProperty*)_parent->find_child("parent/right_pannel/right_pannel/itineraryPannel/third/itinerary_id"))->set_value(msg->itineraries[2].id, true);
  //"humanize the label ? "
    GRAPH_EXEC;
  release_exclusive_access(DBG_REL);

  }

  void 
  RosNode::receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {
  
  get_exclusive_access(DBG_GET);
  // export to result layer

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
        ((AbstractProperty*) edge->find_child("color/value"))->set_value (selected, true);
        //new Binding (edge, "binding_edge_released", edge, "edge/release", _edge_released_na, "");
      }
    }



  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
  }



//callback for robot_state msg (contains data on one robot)
  void 
  RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState::SharedPtr msg) {
    RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);
    get_exclusive_access(DBG_GET);

    if (msg->robot_id == 1){
      //Drone
      ((DoubleProperty*)_drone->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_drone->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_drone->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_drone->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_drone->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_drone->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_drone->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_drone->find_child("failsafe"))->set_value(msg->failsafe, true);
    }
    if (msg->robot_id == 2){
      //Agilex 1
      ((DoubleProperty*)_agilex1->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_agilex1->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_agilex1->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_agilex1->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_agilex1->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_agilex1->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_agilex1->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_agilex1->find_child("failsafe"))->set_value(msg->failsafe, true);

    }
    if (msg->robot_id == 3){
      //Agilex 2
      ((DoubleProperty*)_agilex2->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_agilex2->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_agilex2->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_agilex2->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_agilex2->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_agilex2->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_agilex2->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_agilex2->find_child("failsafe"))->set_value(msg->failsafe, true);

    }
    if (msg->robot_id == 4){
      //Lynx
      ((DoubleProperty*)_lynx->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_lynx->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_lynx->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_lynx->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_lynx->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_lynx->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_lynx->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_lynx->find_child("failsafe"))->set_value(msg->failsafe, true);

    }
    if (msg->robot_id == 5){
      //Spot
      ((DoubleProperty*)_spot->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_spot->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_spot->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_spot->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_spot->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_spot->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_spot->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_spot->find_child("failsafe"))->set_value(msg->failsafe, true);

    }
    if (msg->robot_id == 6){
      //VAB
      ((DoubleProperty*)_vab->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_vab->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_vab->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_vab->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_vab->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_vab->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_vab->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_vab->find_child("failsafe"))->set_value(msg->failsafe, true);

    }
    if (msg->robot_id == 7){
      //Safety pilot
      ((DoubleProperty*)_actor->find_child("lat"))->set_value(msg->position.latitude,true);
      ((DoubleProperty*)_actor->find_child("lon"))->set_value(msg->position.longitude, true);
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

  //Todo, use header.
  }



  void 
  RosNode::receive_msg_trap (const icare_interfaces::msg::TrapList msg){
    std::cerr << "received some traps to display" << std::endl;
    get_exclusive_access(DBG_GET);
    for (int k= 0; k < msg.traps.size(); k ++){
      int index_found = -1;

      Container *_traps_container = dynamic_cast<Container *> (_traps);

      for (int i = 0; i < _traps_container->children().size(); i++){
        if (((IntProperty*)_traps_container->children()[i]->find_child("id"))->get_value() == msg.traps[k].id){
          index_found = i;
        }
      }
      if (index_found == -1){

        std::cerr << "new trap !" << std::endl;
/*int16 id                            # Manager trap id
geographic_msgs/GeoPoint location   # location
bool identified false               # whether the trap has been identified (i.e. QRCode read)
bool active true                    # whether the trap is active

icare_interfaces/TrapIdentification info
uint8[] detected_by                 # list of robots having detected this trap
uint32[] local_ids                   # locals ids of the detection per robot*/
        ParentProcess *new_trap = Trap(_traps, "", _map, msg.traps[k].location.latitude, msg.traps[k].location.longitude, msg.traps[k].id);
        std::cerr << msg.traps[k].location.latitude << std::endl;
        std::cerr << msg.traps[k].location.longitude << std::endl;
        ((BoolProperty*)new_trap->find_child("active"))->set_value(msg.traps[k].active, true);
        ((BoolProperty*)new_trap->find_child("identified"))->set_value(msg.traps[k].identified, true);
        ((TextProperty*)new_trap->find_child("trap_id_text/text"))->set_value(msg.traps[k].info.id, true);
        ((TextProperty*)new_trap->find_child("description"))->set_value(msg.traps[k].info.description, true);
        ((IntProperty*)new_trap->find_child("contact_mode"))->set_value(msg.traps[k].info.contact_mode, true);
        ((TextProperty*)new_trap->find_child("code"))->set_value(msg.traps[k].info.code, true);
        ((TextProperty*)new_trap->find_child("hazard"))->set_value(msg.traps[k].info.hazard, true);
        ((DoubleProperty*)new_trap->find_child("radius"))->set_value(msg.traps[k].info.radius, true);

        std::string timestamp = ((TextProperty*)_clock->find_child("wc/state_text"))->get_value();
        ((TextProperty*)_console->find_child("ste/string_input"))->set_value(timestamp + " - New trap at ["+ std::to_string(msg.traps[k].location.latitude) + "," + std::to_string(msg.traps[k].location.longitude) + "]", true);
        if (msg.traps[k].identified){
          ((TextProperty*)_console->find_child("ste/string_input"))->set_value(timestamp + " - Trap identified at ["+ std::to_string(msg.traps[k].location.latitude) + "," + std::to_string(msg.traps[k].location.longitude) + "]" + " code =" + msg.traps[k].info.code, true);
        }

//rajouter radius
      }
      if (index_found != -1){

        std::string timestamp = ((TextProperty*)_clock->find_child("wc/state_text"))->get_value();

        if (msg.traps[k].identified && !((BoolProperty*)_traps_container->children()[index_found]->find_child("identified"))->get_value()){
          ((TextProperty*)_console->find_child("ste/string_input"))->set_value(timestamp + " - Trap identified at ["+ std::to_string(msg.traps[k].location.latitude) + "," + std::to_string(msg.traps[k].location.longitude) + "]" + " code =" + msg.traps[k].info.code, true);
        }

        std::cerr << "old trap to update!" << std::endl;
        ((BoolProperty*)_traps_container->children()[index_found]->find_child("active"))->set_value(msg.traps[k].active, true);
        ((BoolProperty*)_traps_container->children()[index_found]->find_child("identified"))->set_value(msg.traps[k].identified, true);
        ((TextProperty*)_traps_container->children()[index_found]->find_child("trap_id"))->set_value(msg.traps[k].info.id, true);
        ((TextProperty*)_traps_container->children()[index_found]->find_child("description"))->set_value(msg.traps[k].info.description, true);
        ((IntProperty*)_traps_container->children()[index_found]->find_child("contact_mode"))->set_value(msg.traps[k].info.contact_mode, true);
        ((TextProperty*)_traps_container->children()[index_found]->find_child("code"))->set_value(msg.traps[k].info.code, true);
        ((TextProperty*)_traps_container->children()[index_found]->find_child("hazard"))->set_value(msg.traps[k].info.hazard, true);

        ((DoubleProperty*)_traps_container->children()[index_found]->find_child("radius"))->set_value(msg.traps[k].info.radius, true);

      }



    }
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

    for (int i=0; i < msg.uav_zones.size(); i++){
      ParentProcess* area_to_add = TaskArea(_task_areas , "", _map);
      ((DoubleProperty*)area_to_add->find_child("area_prop"))->set_value(msg.uav_zones[i].area, true);
      ((DoubleProperty*)area_to_add->find_child("explored"))->set_value(msg.uav_zones[i].explored, true);
      for (int j = 0; j < msg.uav_zones[i].points.size(); j++){
        auto* task_summit = TaskAreaSummit(area_to_add, std::string("summit_") + std::to_string(j), _map, msg.uav_zones[i].points[j].latitude, msg.uav_zones[i].points[j].longitude);
        ((DoubleProperty*)task_summit->find_child("alt"))->set_value(msg.uav_zones[i].points[j].altitude, true);
      //auto* cpnt_23 = new PolyPoint (cpnt_21, "pt2", - 20, 20);
        auto* point = new PolyPoint(area_to_add->find_child("area"), std::string("pt_") + std::to_string(j), 0, 0);
     //new Connector (cpnt_1, "", cpnt_26->find_child ("x"), cpnt_21->find_child ("pt1/x"), 1);

        new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);

        new Connector (area_to_add, "x_bind", area_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), area_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);


      }
      ((IntProperty*)area_to_add->find_child("nb_summit"))->set_value(((int)msg.uav_zones[i].points.size()), true);
      //Area {
      // Point pt1
      // Point pt2
      // Point pt3
      //}

      //TaskAreaSummit sum1
      //TaskAreaSummit sum2
      //TaskAreaSummit sum3
      //sum1.x =:> area.pt1.x
      //sum2.y =:> area.pt1.y



    }
    for (int i=0; i < msg.ugv_edges.size(); i++){
    //Create ugv_edges
      ParentProcess* edge_to_add = TaskEdge(_task_edges, "", _map, std::stoi(msg.ugv_edges[i].source) + 1, std::stoi(msg.ugv_edges[i].target) + 1, _nodes);
      ((DoubleProperty*)edge_to_add->find_child("length"))->set_value(msg.ugv_edges[i].length, true);
      ((DoubleProperty*)edge_to_add->find_child("explored"))->set_value(msg.ugv_edges[i].explored, true);
    }
    for (int i=0; i <msg.trap_identifications.size(); i++){
    //Create trap_tasks
      std::cerr << "trying to add a trap_identification at " + std::to_string(msg.trap_identifications[i].location.latitude) << std::endl;
      ParentProcess* trap_to_add = TaskTrap(_task_traps, "", _map, msg.trap_identifications[i].id, msg.trap_identifications[i].location.latitude, msg.trap_identifications[i].location.longitude);
      ((BoolProperty*)trap_to_add->find_child("active"))->set_value(msg.trap_identifications[i].active, true);
      ((BoolProperty*)trap_to_add->find_child("identified"))->set_value(msg.trap_identifications[i].identified, true);
      ((TextProperty*)trap_to_add->find_child("trap_id"))->set_value(msg.trap_identifications[i].info.id, true);
      ((TextProperty*)trap_to_add->find_child("description"))->set_value(msg.trap_identifications[i].info.description, true);
      ((IntProperty*)trap_to_add->find_child("contact_mode"))->set_value(msg.trap_identifications[i].info.contact_mode, true);
      ((TextProperty*)trap_to_add->find_child("code"))->set_value(msg.trap_identifications[i].info.code, true);
      ((TextProperty*)trap_to_add->find_child("hazard"))->set_value(msg.trap_identifications[i].info.hazard, true);
      ((DoubleProperty*)trap_to_add->find_child("radius"))->set_value(msg.trap_identifications[i].info.radius, true);

    }
    for (int i=0; i<msg.trap_deactivations.size(); i++){

      std::cerr << "trying to add a trap_deactivation" << std::endl;
      ParentProcess* trap_to_add = TaskTrap(_task_traps, "", _map, msg.trap_deactivations[i].id, msg.trap_deactivations[i].location.latitude, msg.trap_deactivations[i].location.longitude);
      ((BoolProperty*)trap_to_add->find_child("active"))->set_value(msg.trap_deactivations[i].active, true);
      ((BoolProperty*)trap_to_add->find_child("identified"))->set_value(msg.trap_deactivations[i].identified, true);
      ((TextProperty*)trap_to_add->find_child("trap_id"))->set_value(msg.trap_deactivations[i].info.id, true);
      ((TextProperty*)trap_to_add->find_child("description"))->set_value(msg.trap_deactivations[i].info.description, true);
      ((IntProperty*)trap_to_add->find_child("contact_mode"))->set_value(msg.trap_deactivations[i].info.contact_mode, true);
      ((TextProperty*)trap_to_add->find_child("code"))->set_value(msg.trap_deactivations[i].info.code, true);
      ((TextProperty*)trap_to_add->find_child("hazard"))->set_value(msg.trap_deactivations[i].info.hazard, true);
      ((DoubleProperty*)trap_to_add->find_child("radius"))->set_value(msg.trap_deactivations[i].info.radius, true);
    }

    GRAPH_EXEC;
    release_exclusive_access(DBG_REL);
  }



  void 
  RosNode::receive_msg_allocation(const icare_interfaces::msg::Allocation msg){
    
    //TODO
    //get_exclusive_access(DBG_GET);
    /* .. Traitement .. */
    //GRAPH_EXEC;
    //release_exclusive_access(DBG_REL);
  

  }

  void
  RosNode::send_msg_lima(int id){

    icare_interfaces::msg::LimaCrossed message = icare_interfaces::msg::LimaCrossed();
//message.id = id;
    message.id = id;

    publisher_lima->publish(message);
  }


  void
  RosNode::send_msg_planning_request(){
    std::cerr << "in send planning request" << std::endl;

    /*get_exclusive_access(DBG_GET);
*/
    icare_interfaces::msg::PlanningRequest message = icare_interfaces::msg::PlanningRequest();
    std::cerr << _parent << std::endl;
    message.id = _current_plan_id_vab.get_string_value();
    int iid;

    for (auto item: ((djnn::List*)_nodes)->children()){
      std::cerr << "debug : " << ((TextProperty*)item->find_child("status"))->get_value() << std::endl;
      if (((TextProperty*)item->find_child("status"))->get_value() == "start"){
        std::cerr << "start" << std::endl;
        iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
        message.start_node = std::to_string(iid - 1);
      }
      if (((TextProperty*)item->find_child("status"))->get_value() == "end"){
        std::cerr << "end" << std::endl;
        iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();

        message.end_node = std::to_string(iid - 1);

      }
      if (((TextProperty*)item->find_child("status"))->get_value()=="forced"){
        std::cerr << "forced" <<std::endl;
        iid = dynamic_cast<IntProperty*> (item->find_child("id"))->get_value();
        message.node_contraints.push_back(std::to_string(iid -1));
      }
    }
    /*GRAPH_EXEC;
    release_exclusive_access(DBG_REL);
    */message.header.stamp = _node->get_clock()->now();

    publisher_planning_request->publish(message);  
    
  }

  void 
  RosNode::send_msg_navgraph_update(){

/*    get_exclusive_access(DBG_GET);
*/    CoreProcess* nodes = _parent->find_child ("parent/l/map/layers/navgraph/nodes");
    CoreProcess* edges = _parent->find_child ("parent/l/map/layers/navgraph/edges");

    std::cerr << "about to generate json" << std::endl;
    nlohmann::json j;
    j["graph"]["directed"] = false;


  //Edges
    for (auto item: ((djnn::List*)edges)->children()){

      int ssource_id = dynamic_cast<IntProperty*> (item->find_child ("id_src"))->get_value ();
      int starget_id = dynamic_cast<IntProperty*> (item->find_child ("id_dest"))->get_value ();
      double dlength = dynamic_cast<DoubleProperty*> (item->find_child ("length"))->get_value ();

    //Rarccos(sin(a)sin(b) + cos(a)cos(b)cos(c-d)) a = lata, b=lona, c =latb, d =lonb
  /*  double lata = dynamic_cast<DoubleProperty*> (nodes[ssource_id].find_child("lat"))->get_value() * 3.14259/180;
    double lona = dynamic_cast<DoubleProperty*> (nodes[ssource_id].find_child("lon"))->get_value() * 3.14259/180;
    double latb = dynamic_cast<DoubleProperty*> (nodes[starget_id].find_child("lat"))->get_value() * 3.14259/180;
    double lonb = dynamic_cast<DoubleProperty*> (nodes[starget_id].find_child("lon"))->get_value() * 3.14259/180;
    double dlength = 6371000 * acos(sin(lata)*sin(lona) + cos(lata)*cos(lona)*cos(latb-lonb));
   */ nlohmann::json jn = {
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
    int iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
    string slabel = dynamic_cast<TextProperty*> (item->find_child ("label"))->get_value ();
    double dlat = dynamic_cast<DoubleProperty*> (item->find_child ("wpt/lat"))->get_value ();
    double dlon = dynamic_cast<DoubleProperty*> (item->find_child ("wpt/lon"))->get_value ();
    double dalt = dynamic_cast<DoubleProperty*> (item->find_child ("alt"))->get_value ();
    string scompulsory = dynamic_cast<TextProperty*>(item->find_child("wpt/usage_status"))->get_value();
    
    bool compulsory = (scompulsory == "mandatory");   
    if (compulsory){
      std::cerr << "one more PPO exported"<< std::endl;
    }
    bool locked = dynamic_cast<BoolProperty*>(item->find_child("islocked"))->get_value();
    

    nlohmann::json jn = {
      {"id", std::to_string(iid - 1)},
      {"label", slabel},
      {"metadata", { 
        {"altitude", dalt},
        {"latitude", dlat},
        {"longitude", dlon},
        {"compulsory", compulsory},
        {"locked", locked}
      }}
    };                
    j["graph"]["nodes"].push_back(jn);   
  }


  //TODO: remove - only for debug
  std::cerr << "finished generating JSON" << std::endl;
  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  std::cerr << j.dump() << std::endl;
  message.data = j.dump();
  std::cerr << "about to publish on publisher_navgraph_update" << std::endl;

  message.header.stamp = _node->get_clock()->now();
  publisher_navgraph_update->publish(message);
  /*GRAPH_EXEC;
  */std::cerr << "finished publishing" << std::endl;
/*  release_exclusive_access(DBG_REL);
*/

}

void 
RosNode::send_validation_plan(){

  std::cerr << "in validation plan" << std::endl;
  std::cerr << _parent << std::endl;

  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  message.data = _id_curent_itenerary->get_string_value();
  message.header.stamp = _node->get_clock()->now();
  publisher_validation->publish(message);
  GRAPH_EXEC;
  
}

void 
RosNode::send_selected_tasks(){
//TODO

 /*get_exclusive_access(DBG_GET);
*/
 icare_interfaces::msg::Tasks message= icare_interfaces::msg::Tasks();
 for (auto trap: ((djnn::List*)_task_traps)->children()){
  if (((BoolProperty*)trap->find_child("selected"))->get_value() == true){

      /*int16 id                            # Manager trap id
geographic_msgs/GeoPoint location   # location
bool identified false               # whether the trap has been identified (i.e. QRCode read)
bool active true                    # whether the trap is active

icare_interfaces/TrapIdentification info
uint8[] detected_by                 # list of robots having detected this trap
uint32[] local_ids                   # locals ids of the detection per robot
*/
/*
((BoolProperty*)new_trap->find_child("active"))->set_value(msg.active, true);
  ((BoolProperty*)new_trap->find_child("identified"))->set_value(msg.identified, true);
    ((TextProperty*)new_trap->find_child("trap_id"))->set_value(msg.info.id, true);
    ((TextProperty*)new_trap->find_child("description"))->set_value(msg.info.description, true);
    ((IntProperty*)new_trap->find_child("contact_mode"))->set_value(msg.info.contact_mode, true);
    ((TextProperty*)new_trap->find_child("code"))->set_value(msg.info.code, true);
    ((TextProperty*)new_trap->find_child("hazard"))->set_value(msg.info.hazard, true);

*/

    icare_interfaces::msg::Trap trap_to_add = icare_interfaces::msg::Trap();
    trap_to_add.id = dynamic_cast<IntProperty*> (trap->find_child ("trap_id"))->get_value ();
    trap_to_add.identified = dynamic_cast<BoolProperty*>(trap->find_child("identified"))->get_value();
    trap_to_add.active = dynamic_cast<BoolProperty*>(trap->find_child("active"))->get_value();
   /* trap_to_add.info.id = dynamic_cast<TextProperty*>(trap->find_child("trap_id"))->get_value();
    trap_to_add.info.description = dynamic_cast<TextProperty*>(trap->find_child("description"))->get_value();
    trap_to_add.info.contact_mode = dynamic_cast<IntProperty*>(trap->find_child("contact_mode"))->get_value();
    trap_to_add.info.code = dynamic_cast<TextProperty*>(trap->find_child("code"))->get_value();
    trap_to_add.info.hazard = dynamic_cast<TextProperty*>(trap->find_child("hazard"))->get_value();
   */ 
    trap_to_add.location.latitude = dynamic_cast<DoubleProperty*>(trap->find_child("lat"))->get_value();
    trap_to_add.location.longitude = dynamic_cast<DoubleProperty*>(trap->find_child("lon"))->get_value();
    
    if(trap_to_add.identified){
      message.trap_deactivations.push_back(trap_to_add);

    } 
    if(!trap_to_add.identified){
      message.trap_identifications.push_back(trap_to_add);
    }
  }

}
for (auto edge: ((djnn::List*)_task_edges)->children()){
  if (dynamic_cast<BoolProperty*> (edge->find_child ("selected"))->get_value ()){
    icare_interfaces::msg::GraphEdge edge_to_add = icare_interfaces::msg::GraphEdge();
    edge_to_add.source = std::to_string(dynamic_cast<IntProperty*> (edge->find_child ("id_source"))->get_value () - 1);

    edge_to_add.target = std::to_string(dynamic_cast<IntProperty*> (edge->find_child ("id_dest"))->get_value () - 1);
    edge_to_add.length = dynamic_cast<DoubleProperty*> (edge->find_child("length"))->get_value();
    edge_to_add.explored = dynamic_cast<DoubleProperty*> (edge->find_child("explored"))->get_value();
    message.ugv_edges.push_back(edge_to_add);
  }
}

for (auto area: ((djnn::List*)_task_areas)->children()){
 if (dynamic_cast<BoolProperty*> (area->find_child("selected"))->get_value()){

  icare_interfaces::msg::ExplorationPolygon geopolygon_to_add = icare_interfaces::msg::ExplorationPolygon();
  geopolygon_to_add.area = dynamic_cast<DoubleProperty*>(area->find_child("area_prop"))->get_value();
  geopolygon_to_add.explored = dynamic_cast<DoubleProperty*>(area->find_child("explored"))->get_value();
  for (int i = 0; i < dynamic_cast<IntProperty*>(area->find_child("nb_summit"))->get_value();i++){
    geographic_msgs::msg::GeoPoint point_to_add = geographic_msgs::msg::GeoPoint();
    point_to_add.latitude =  dynamic_cast<DoubleProperty*>(area->find_child(std::string("summit_") + std::to_string(i) + std::string("/lat")))->get_value();
    point_to_add.longitude =  dynamic_cast<DoubleProperty*>(area->find_child(std::string("summit_") + std::to_string(i) + std::string("/lon")))->get_value();
    point_to_add.altitude =  dynamic_cast<DoubleProperty*>(area->find_child(std::string("summit_") + std::to_string(i) + std::string("/alt")))->get_value();
    geopolygon_to_add.points.push_back(point_to_add);
  }


  message.uav_zones.push_back(geopolygon_to_add);
}
}
message.header.stamp = _node->get_clock()->now();
publisher_tasks->publish(message);
/*GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
*/}
void 
RosNode::receive_msg_site(const icare_interfaces::msg::Site msg){


  get_exclusive_access(DBG_GET);
/*
# Describe the mission site
geographic_msgs/GeoPoint start_point
icare_interfaces/GeoPolygon limits
icare_interfaces/RestrictedZone[] zones
icare_interfaces/Lima[] limas


*/
  /*# Restricted Zones
icare_interfaces/GeoPolygon polygon
string name
uint8 type

uint8 TYPE_UNKNOWN    = 0 # Unknown zone type
uint8 TYPE_RFA        = 1 # Restricted Fire Area (deactivation only on clearance)
uint8 TYPE_NFA        = 2 # No Fire Area (deactivation forbidden)
uint8 TYPE_NFZ        = 3 # No Fly Zone
uint8 TYPE_FFA        = 4 # Free Fire Area (deactivation allowed)
uint8 TYPE_ROZ_ALL    = 5 # Restricted Operation Zone (forbidden to all vehicles)
uint8 TYPE_ROZ_GROUND = 6 # Restricted Operation Zone (forbidden to ground vehicles)*/

//TODO refactor (merge) ExclusionArea + lima
  for (int i=0; i < msg.zones.size(); i++){
    ParentProcess* area_to_add = ExclusionArea(_exclusion_areas,"", _map, "unknown"); 
    ((TextProperty*)area_to_add->find_child("name"))->set_value(msg.zones[i].name, true);
    std::cerr << std::to_string(msg.zones[i].type) << std::endl;
    if(msg.zones[i].type == 0){
      ((TextProperty*)area_to_add->find_child("status"))->set_value("unknown", true);
    }
    if(msg.zones[i].type == 1){


      ((TextProperty*)area_to_add->find_child("status"))->set_value("rfa", true);}

      if(msg.zones[i].type == 2){

        ((TextProperty*)area_to_add->find_child("status"))->set_value("nfa", true);
      }
      if(msg.zones[i].type == 3){
        ((TextProperty*)area_to_add->find_child("status"))->set_value("nfz", true);
      }
      if(msg.zones[i].type == 4){
        ((TextProperty*)area_to_add->find_child("status"))->set_value("ffa", true);
      }

      if(msg.zones[i].type == 5){
        ((TextProperty*)area_to_add->find_child("status"))->set_value("roz_all", true);
      }

      if(msg.zones[i].type == 6){
        ((TextProperty*)area_to_add->find_child("status"))->set_value("roz_ground", true);
      }


      auto* bary_summit = TaskAreaSummit(area_to_add, "bary_summit", _map, 0, 0);
      int n = msg.zones[i].polygon.points.size();
      
      double above_x = 0;
      double below_x = 0;

      double above_y = 0;
      double below_y = 0;
      for (int j = 0; j < n; j++){
        auto* task_summit = TaskAreaSummit(area_to_add, std::string("summit_") + std::to_string(j), _map, msg.zones[i].polygon.points[j].latitude, msg.zones[i].polygon.points[j].longitude);
        ((DoubleProperty*)task_summit->find_child("alt"))->set_value(msg.zones[i].polygon.points[j].altitude, true);
        auto* point = new PolyPoint(area_to_add->find_child("area"), std::string("pt_") + std::to_string(j), 0, 0);
        /*double cur_lat =  dynamic_cast<DoubleProperty*>(bary_summit->find_child("lat"))->get_value();
        double cur_lon =  dynamic_cast<DoubleProperty*>(bary_summit->find_child("lon"))->get_value();
        
        ((DoubleProperty*)bary_summit->find_child("lat"))->set_value(cur_lat + msg.zones[i].polygon.points[j].latitude / n, true);
        ((DoubleProperty*)bary_summit->find_child("lon"))->set_value(cur_lon + msg.zones[i].polygon.points[j].longitude / n, true);
       */
        ((TextProperty*)area_to_add->find_child("name"))->set_value(msg.zones[i].name, true);
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

      ((DoubleProperty*)bary_summit->find_child("lat"))->set_value(above_x/ below_x, true);
      ((DoubleProperty*)bary_summit->find_child("lon"))->set_value(above_y / below_y, true);
     
     new Connector (area_to_add, "x_bary_bind", area_to_add->find_child("bary_summit/x"), area_to_add->find_child("barycenterX"), 1);
     new Connector (area_to_add, "y_bary_bind", area_to_add->find_child("bary_summit/y"), area_to_add->find_child("barycenterY"), 1);
        


  }
    for (int i=0; i < msg.limas.size(); i++){
      ParentProcess *lima_to_add = Lima(_limas, "", _map, this);
      ((IntProperty*)lima_to_add->find_child("id"))->set_value(msg.limas[i].index, true);
      ((TextProperty*)lima_to_add->find_child("name"))->set_value(msg.limas[i].name, true);
      for (int j = 0; j < msg.limas[i].points.size(); j++){
        auto* task_summit = TaskAreaSummit(lima_to_add, std::string("summit_") + std::to_string(j), _map, msg.limas[i].points[j].latitude, msg.limas[i].points[j].longitude);
        ((DoubleProperty*)task_summit->find_child("alt"))->set_value(msg.limas[i].points[j].altitude, true);
        auto* point = new PolyPoint(lima_to_add->find_child("lima"), std::string("pt_") + std::to_string(j), 0, 0);

        new Connector (lima_to_add, "x_bind", lima_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/x")), lima_to_add->find_child(std::string("lima/") + std::string("pt_") + std::to_string(j) + std::string("/x")), 1);

        new Connector (lima_to_add, "y_bind", lima_to_add->find_child(std::string("summit_") + std::to_string(j) + std::string("/y")), lima_to_add->find_child(std::string("lima/") + std::string("pt_") + std::to_string(j) + std::string("/y")), 1);
       
      /*  new NativeAction (lima_to_add, "send_lima_id_na", RosNode::send_msg_lima, msg.limas[i].index, true);

        new Binding (lima_to_add, "lima_pressed", lima_to_add, "pressed", lima_to_add, "send_lima_id_na");
      */  
      //TODO : 
      // binding : lima.press -> send correct lima id
     /*   new Binding (edge, "binding_edge_released", edge, "edge/release", _edge_released_na, "");
        new Binding (lima_to_add, "binding_edge_released", lima, "press", _lima_released_na, "");*/

      }
    }
    ParentProcess *limits_to_add = ExclusionArea(_exclusion_areas, "", _map, "limits");
      
    for (int i=0; i<msg.limits.points.size(); i++){
      auto* limit_summit = TaskAreaSummit(limits_to_add, std::string("summit_") + std::to_string(i), _map, msg.limits.points[i].latitude, msg.limits.points[i].longitude);
      ((DoubleProperty*)limit_summit->find_child("alt"))->set_value(msg.limits.points[i].altitude, true);
      auto* point = new PolyPoint(limits_to_add->find_child("area"), std::string("pt_") + std::to_string(i), 0, 0);

      new Connector (limits_to_add, "x_bind", limits_to_add->find_child(std::string("summit_") + std::to_string(i) + std::string("/x")), limits_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(i) + std::string("/x")), 1);

      new Connector (limits_to_add, "y_bind", limits_to_add->find_child(std::string("summit_") + std::to_string(i) + std::string("/y")), limits_to_add->find_child(std::string("area/") + std::string("pt_") + std::to_string(i) + std::string("/y")), 1);

    }
    GRAPH_EXEC;
   release_exclusive_access(DBG_REL);
  }
  void 
  RosNode::send_validation_tasks(){
//TODO

//  message.header.stamp = _node->get_clock()->now();

  }

void 
RosNode::receive_msg_map(const icare_interfaces::msg::EnvironmentMap msg){
std::cerr << "received exploration map" << std::endl;

 float lat_center = msg.origin.latitude;
 float lon_center = msg.origin.longitude; 
std::cerr << lat_center << std::endl;
std::cerr << lon_center << std::endl;




}



#endif

/*
//Util : 
  void write_to_log (string _filename, string s) {

    string filename(_filename);

    std::fstream file_out;



    file_out.open(filename, std::ios_base::app);

    if (!file_out.is_open()) {

      std::cout << "failed to open " << filename << '\n';

    } else {

      file_out << s << std::endl;

    }
    file_out.close();

  }
*/


  void RosNode::test_draw_visibility_map(){
    float lat_center_map = 44.27432196595285;
    float lon_center_map = 1.729783361205679;

    int width = 20; //20 collumns
    int height = 20; //20 rows
    int ugv_camera_layer[width * height]; //1 if said pixel is explored, 0 if not
    int uav_camera_layer[width * height]; //1 if said pixel is explored, 0 if not
    //uint8[] ugv_lidar_layer; //dispo mais pas utilisé
    


    float resolution = 5; //cells are 5 meters large squares 
    std::cerr << "debug draw_visbility map" << " lattiude " << lat_center_map << " longitude " << lon_center_map << " resolution " << resolution << std::endl;



    //@Mathieu ; 
    //Pour faire en sorte que l'image que tu crée suive le pan, tu peux essayer d'instancier un TaskAreaSummit 
    //(c'est un waypoint sans aspect) et créer un binding ayant pour source ses x et y

    //Fusion rules (colors subject to change )

    //ugv_camera => yellow ( #f4d03f )
    //uav_camera => purple ( #9b59b6 )
    //uav_camera && ugv_camera => cyan #7fb3d5 *



  }
  void
  RosNode::run () {
  #ifndef NO_ROS
    rclcpp::spin(_node);
    rclcpp::shutdown();
  #endif
  }
