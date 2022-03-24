#include "ros_node.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"
#include "core/tree/list.h"

#include <nlohmann/json.hpp>

#ifndef NO_LEMON
#include "include/navgraph/navgraph.hpp"
#include "Node.h"
#include "NavGraph.h"
#include "Edge.h"
#endif

using std::placeholders::_1;

using namespace djnn;

#ifndef NO_LEMON
using navgraph::to_json;
using navgraph::from_json;
#endif

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
  qos(1)
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

#endif
  finalize_construction (parent, n);
}

void
RosNode::impl_activate ()
{ 
#ifndef NO_ROS
  //subscriptions
  sub_navgraph =_node->create_subscription<icare_interfaces::msg::StringStamped>( 
  "/navgraph", qos, std::bind(&RosNode::receive_msg_navgraph, this, _1)); //Replace 10 with qosbesteffort
  
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>(
    "/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, this, _1));

  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
    "/itinerary", qos, std::bind(&RosNode::receive_msg_graph_itinerary_loop, this, _1));

  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
    "/plan", qos, std::bind(&RosNode::receive_msg_graph_itinerary_final, this, _1));


  publisher_planning_request =_node->create_publisher<icare_interfaces::msg::PlanningRequest>("/planning_request", qos);
  publisher_validation = _node->create_publisher<icare_interfaces::msg::StringStamped>("/validation", qos);
  publisher_navgraph_update = _node->create_publisher<icare_interfaces::msg::StringStamped>("/navgraph_update", qos);



#endif
#ifndef NO_LEMON
  //activate navgraph fields
  navgraph_data.activate();
#endif

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
  _itinerary_edges = _parent->find_child("parent/l/map/layers/navgraph/itinerary_edges");

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
  
#ifndef NO_LEMON
  sub_navgraph.reset ();
  sub_robot_state.reset ();
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

  navgraph::NavGraph _navgraph(msg->data);

/*  CoreProcess* nodes = _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("nodes");
  CoreProcess* edges = _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("edges");
  CoreProcess* shadow_edges = _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("shadow_edges");*/
 

for (auto item: ((djnn::List*)_edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
  }


  for (auto item: ((djnn::List*)_shadow_edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
    }

 
  for (auto item: ((djnn::List*)_nodes)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
      
    }


  for (lemon::ListGraph::NodeIt n(_navgraph.nodes()); n != lemon::INVALID; ++n) {
      navgraph::GeoPoint p = _navgraph.get_geopoint(n);
      ParentProcess* node = Node(_nodes, "", _map , p.latitude, p.longitude, p.altitude,
       0, _navgraph.get_label(n), std::stoi(_navgraph.get_id(n)), _manager);
      //_parent->find_child ("nodes")->add_child(node, "");     
      std::cerr << "fin boucle"  << std::endl;
  }
  for (lemon::ListGraph::EdgeIt n(_navgraph.edges()); n!= lemon::INVALID; ++n){
      ParentProcess* edge = Edge(_edges, "", std::stoi(_navgraph.get_id(_navgraph.source(n))), 
        std::stoi(_navgraph.get_id(_navgraph.target(n))), _navgraph.get_length(n), _nodes);
     // _parent->find_child("edges")->add_child(edge, "");
  }
  //_parent->find_child("graph_pub")->navgraph = _navgraph;
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

#ifndef NO_ROS
void 
RosNode::receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {

/*  for (auto item: ((djnn::List*)_itinerary_edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
      
    }*/

  int size = msg->nodes.size();

  for (int i = 0; i <  size - 1; ++i) {
      std::cout << "trying to draw arc between " << i << " and " << i+1 << std::endl;
      ParentProcess* edge = Edge(_itinerary_edges, "", std::stoi(msg->nodes[i]), std::stoi(msg->nodes[i+1]), 20, _nodes);
      ((AbstractProperty*)edge->find_child("color/r"))->set_value(30, true);
      ((AbstractProperty*)edge->find_child("color/g"))->set_value(144, true);
      ((AbstractProperty*)edge->find_child("color/b"))->set_value(255, true);
    }
}

void 
RosNode::receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {

}
#endif


//callback for robot_state msg (contains data on one robot)
void 
RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState::SharedPtr msg) {
  RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);
  get_exclusive_access(DBG_GET);
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
#endif

void
RosNode::send_msg_planning_request(){
  std::cerr << "in send planning request" << std::endl;
  #ifndef NO_ROS
  icare_interfaces::msg::PlanningRequest message = icare_interfaces::msg::PlanningRequest();
  std::cerr << _parent << std::endl;
  message.id = _current_plan_id_vab.get_string_value();
  int iid;

  for (auto item: ((djnn::List*)_nodes)->children()){
    std::cerr << "debug : " << ((TextProperty*)item->find_child("status"))->get_value() << std::endl;
    if (((TextProperty*)item->find_child("status"))->get_value() == "start"){
      std::cerr << "start" << std::endl;
      iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
      message.start_node = std::to_string(iid);
    }
    if (((TextProperty*)item->find_child("status"))->get_value() == "end"){
      std::cerr << "end" << std::endl;
      iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
    
      message.end_node = std::to_string(iid);
      
    }
  }

  
  publisher_planning_request->publish(message);  
  #endif

}

void 
RosNode::send_msg_navgraph_update(){

#ifndef NO_ROS
  CoreProcess* nodes = _parent->find_child ("parent/l/map/layers/navgraph/nodes");
  CoreProcess* edges = _parent->find_child ("parent/l/map/layers/navgraph/edges");

  nlohmann::json j;
  j["graph"]["directed"] = false;


  //Edges
  for (auto item: ((djnn::List*)edges)->children()){

    int ssource_id = dynamic_cast<IntProperty*> (item->find_child ("id_src"))->get_value ();
    int starget_id = dynamic_cast<IntProperty*> (item->find_child ("id_dest"))->get_value ();
    double dlength = dynamic_cast<DoubleProperty*> (item->find_child ("length"))->get_value ();

    nlohmann::json jn = {
      {"source", ssource_id},
      {"target", starget_id},
      {"metadata", { 
        {"length", dlength}
      }}
    };                
    j["graph"]["edges"].push_back(jn); 
  }

  //Nodes
  for (auto item: ((djnn::List*)nodes)->children()){
    int iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
    string slabel = dynamic_cast<TextProperty*> (item->find_child ("label"))->get_value ();
    double dlat = dynamic_cast<DoubleProperty*> (item->find_child ("lat"))->get_value ();
    double dlon = dynamic_cast<DoubleProperty*> (item->find_child ("lon"))->get_value ();
    double dalt = dynamic_cast<DoubleProperty*> (item->find_child ("alt"))->get_value ();
       
    nlohmann::json jn = {
      {"id", iid},
      {"label", slabel},
      {"metadata", { 
        {"altitude", dalt},
        {"latitude", dlat},
        {"longitude", dlon}
      }}
    };                
    j["graph"]["nodes"].push_back(jn);   
  }

  //TODO: remove - only for debug
  //std::cerr << j << std::endl;
  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  message.data = j.dump();
  publisher_navgraph_update->publish(message);
#endif
}

void 
RosNode::send_validation_plan(){
 
  std::cerr << "in validation plan" << std::endl;
  std::cerr << _parent << std::endl;
   #ifndef NO_ROS
    icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
    message.data = std::to_string(_current_plan_id_vab.get_value());
    publisher_validation->publish(message);
  #endif
}




void
RosNode::run () {
  #ifndef NO_ROS
  rclcpp::spin(_node);
  rclcpp::shutdown();
  #endif
}
