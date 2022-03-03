#include "graph_subscriber.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"

#include "include/navgraph/navgraph.hpp"
#include "Node.h"
#include "NavGraph.h"

using std::placeholders::_1;

using namespace djnn;
using navgraph::to_json;
using navgraph::from_json;

GraphSubscriber::GraphSubscriber (ParentProcess* parent, const string& n, const string& topic_name, FatProcess* map, FatProcess* manager) :
  FatProcess (n),
  ExternalSource (n),
  _topic_name (topic_name),
  _string_data (""),
  _data(this, "data", ""),
  _map (map),
  _manager (manager)
 // qosbesteffort(10)


{
  _node = std::make_shared<rclcpp::Node>(n);
//  qosbesteffort.best_effort();

  finalize_construction (parent, n);
}

void
GraphSubscriber::impl_activate ()
{ 
  subscription_ =_node->create_subscription<icare_interfaces::msg::StringStamped>( _topic_name, 10, std::bind(&GraphSubscriber::receive_msg, this, _1)); //Replace 10 with qosbesteffort
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
  _data.set_value(msg -> data, true);
  /*navgraph::NavGraph g;
  from_json(msg -> data, g);
  if (not_found_id) {
    FarProcess* ng = NavGraph (root, "id", map);
    FatProcess* nodes = ng->find_child ("nodes");
    for (auto node : g.nodes) {
      FatProcess* new_node = Node (nodes, "", _map, lat, lon, _manager);
    }
    navgraph_list.add (ng);
  }*/
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void
GraphSubscriber::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}
