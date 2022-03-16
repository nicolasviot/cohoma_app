#include "graph_publisher.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"

#include "include/navgraph/navgraph.hpp"
#include "Node.h"
#include "NavGraph.h"
#include "Edge.h"

using namespace djnn;
using navgraph::to_json;
using navgraph::from_json;


GraphPublisher::GraphPublisher (ParentProcess* parent, const string& n, const string& topic_name, CoreProcess* my_map, CoreProcess* manager) :
  FatProcess (n),
  ExternalSource (n),
  _msg (this, "message", ""),
  _action (this, "send"),
  _c_msg (&_msg, ACTIVATION, &_action, ACTIVATION)
 // qos(1)
{
  _node = std::make_shared<rclcpp::Node>(n);
 /* qos.reliable();
  qos.durability_volatile();
 */ publisher_ =_node->create_publisher<icare_interfaces::msg::StringStamped>(topic_name, 10);
  finalize_construction (parent, n);
}

void
GraphPublisher::impl_activate ()
{
  _c_msg.enable ();
  ExternalSource::start ();  
}

void
GraphPublisher::impl_deactivate ()
{
  _c_msg.disable ();
  ExternalSource::please_stop ();
}

void 
GraphPublisher::send_msg () {
  auto message = icare_interfaces::msg::StringStamped();;
  publisher_->publish(message);
}

void
GraphPublisher::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}
