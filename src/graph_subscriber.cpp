#include "graph_subscriber.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"

#include "include/navgraph/navgraph.hpp"
#include "Node.h"
#include "NavGraph.h"
#include "Edge.h"

using std::placeholders::_1;

using namespace djnn;
using navgraph::to_json;
using navgraph::from_json;

GraphSubscriber::GraphSubscriber (ParentProcess* parent, const string& n, const string& topic_name, CoreProcess* my_map, CoreProcess* manager) :
  FatProcess (n),
  ExternalSource (n),
  _topic_name (topic_name),
  _string_data (""),
  _data(this, "data", ""),
  _map (my_map),
  _manager (manager),
 // qosbesteffort(10)
  qos(1)
//  _navgraph()
{
  _node = std::make_shared<rclcpp::Node>(n);
//  qosbesteffort.best_effort();


  qos.reliable();
  qos.durability_volatile();
  finalize_construction (parent, n);



}

void
GraphSubscriber::impl_activate ()
{ 
  subscription_ =_node->create_subscription<icare_interfaces::msg::StringStamped>( 
  _topic_name, qos, std::bind(&GraphSubscriber::receive_msg, this, _1)); //Replace 10 with qosbesteffort
  _data.activate();
  ExternalSource::start ();  
  

}

void
GraphSubscriber::impl_deactivate ()
{
  // Here we should disable the subcription but it 
  // seems there is no way to do it properly
  // some insights here:
  // https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/
  subscription_.reset ();
  _data.deactivate();
  ExternalSource::please_stop ();
}

void 
GraphSubscriber::receive_msg (const icare_interfaces::msg::StringStamped::SharedPtr msg) {
  get_exclusive_access(DBG_GET);
  //_navgraph(msg->data);
  navgraph::NavGraph _navgraph(msg->data);
  for (lemon::ListGraph::NodeIt n(_navgraph.nodes()); n != lemon::INVALID; ++n) {
      navgraph::GeoPoint p = _navgraph.get_geopoint(n);
      ParentProcess* node = Node(_parent, "", _map , p.latitude, p.longitude, p.altitude,
       0, _navgraph.get_label(n), std::stoi(_navgraph.get_id(n)), _manager);
      _parent->find_child ("nodes")->add_child(node, "");     
      std::cerr << "fin boucle"  << std::endl;
  }
  for (lemon::ListGraph::EdgeIt n(_navgraph.edges()); n!= lemon::INVALID; ++n){
      ParentProcess* edge = Edge(_parent, "", std::stoi(_navgraph.get_id(_navgraph.source(n))), 
        std::stoi(_navgraph.get_id(_navgraph.target(n))), _navgraph.get_length(n), _parent->find_child("nodes"));
      _parent->find_child("edges")->add_child(edge, "");
  }
  //_parent->find_child("graph_pub")->navgraph = _navgraph;
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void
GraphSubscriber::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}
