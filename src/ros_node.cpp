#include "ros_node.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"
#include "core/core.h"

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
  _operation_mode(this, "operation_mode", 0)

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

  ExternalSource::please_stop ();
}


#ifndef NO_ROS
// callback for navgraph msg (contains the navigation graph)
void 
RosNode::receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg) {
  get_exclusive_access(DBG_GET);

  navgraph::NavGraph _navgraph(msg->data);

  CoreProcess* nodes = _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("nodes");
  CoreProcess* edges = _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("edges");
  CoreProcess* shadow_edges = _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("shadow_edges");
 

for (auto item: ((djnn::List*)edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
  }


  for (auto item: ((djnn::List*)shadow_edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
    }

 
  for (auto item: ((djnn::List*)nodes)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
      
    }


  for (lemon::ListGraph::NodeIt n(_navgraph.nodes()); n != lemon::INVALID; ++n) {
      navgraph::GeoPoint p = _navgraph.get_geopoint(n);
      ParentProcess* node = Node(_parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("nodes"), "", _map , p.latitude, p.longitude, p.altitude,
       0, _navgraph.get_label(n), std::stoi(_navgraph.get_id(n)), _manager);
      //_parent->find_child ("nodes")->add_child(node, "");     
      std::cerr << "fin boucle"  << std::endl;
  }
  for (lemon::ListGraph::EdgeIt n(_navgraph.edges()); n!= lemon::INVALID; ++n){
      ParentProcess* edge = Edge(_parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("edges"), "", std::stoi(_navgraph.get_id(_navgraph.source(n))), 
        std::stoi(_navgraph.get_id(_navgraph.target(n))), _navgraph.get_length(n), _parent->find_child ("parent")->find_child("l")->find_child("map")->find_child("layers")->find_child("navgraph")->find_child("nodes"));
     // _parent->find_child("edges")->add_child(edge, "");
  }
  //_parent->find_child("graph_pub")->navgraph = _navgraph;
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}



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
RosNode::run () {
  #ifndef NO_ROS
  rclcpp::spin(_node);
  rclcpp::shutdown();
  #endif
}
