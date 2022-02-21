#include "ros_subscriber.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"

using std::placeholders::_1;

using namespace djnn;

RosSubscriber::RosSubscriber (ParentProcess* parent, const string& n, const string& topic_name) :
  FatProcess (n),
  ExternalSource (n),
  _topic_name (topic_name),
  _msg (this, "message", ""),
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


{
  _node = std::make_shared<rclcpp::Node>(n);

  finalize_construction (parent, n);
}

void
RosSubscriber::impl_activate ()
{ 
  subscription_ =_node->create_subscription<icare_interfaces::msg::RobotState>(
      _topic_name, 10, std::bind(&RosSubscriber::receive_msg, this, _1));
  _msg.activate();
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
  ExternalSource::start ();  
  

}

void
RosSubscriber::impl_deactivate ()
{
  // Here we should disable the subcription but it 
  // seems there is no way to do it properly
  // some insights here:
  // https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/
  subscription_.reset ();
  _msg.deactivate();
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

void 
RosSubscriber::receive_msg (const icare_interfaces::msg::RobotState::SharedPtr msg) {
  RCLCPP_INFO(_node->get_logger(), "I heard: '%f'", msg->position.latitude);
  get_exclusive_access(DBG_GET);
  _msg.set_value (msg->position.latitude, true);
  _latitude.set_value (msg -> position.latitude, true);
  _longitude.set_value (msg -> position.longitude, true);
  _robot_id.set_value (msg -> robot_id, true);
  _battery_percentage.set_value ( msg -> battery_percentage, true);
  _battery_voltage.set_value (msg -> battery_voltage, true);
  _compass_heading.set_value (msg -> compass_heading, true);
  _emergency_stop.set_value (msg -> emergency_stop, true);
  _failsafe.set_value (msg -> failsafe, true);
  _operation_mode.set_value (msg -> operating_mode, true);
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void
RosSubscriber::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}
