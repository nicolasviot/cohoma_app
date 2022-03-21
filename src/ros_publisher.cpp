#include "ros_publisher.h"
using namespace djnn;

RosPublisher::RosPublisher (ParentProcess* parent, const string& n, const string& topic_name) :
  FatProcess (n),
  ExternalSource (n),
  _msg (this, "message", ""),
  _action (this, "send"),
  _c_msg (&_msg, ACTIVATION, &_action, ACTIVATION)
{
  _node = std::make_shared<rclcpp::Node>(n);
  publisher_ =_node->create_publisher<icare_interfaces::msg::RobotState>(topic_name, 10);
  finalize_construction (parent, n);
}

void
RosPublisher::impl_activate ()
{
  _c_msg.enable ();
  ExternalSource::start ();  
}

void
RosPublisher::impl_deactivate ()
{
  _c_msg.disable ();
  ExternalSource::please_stop ();
}

void 
RosPublisher::send_msg () {
  auto message = icare_interfaces::msg::RobotState();
  message.position.latitude = 1.5/*_msg.get_value ()*/;
  RCLCPP_INFO(_node->get_logger(), "Publishing: '%f'", message.position.latitude);
  publisher_->publish(message);
}


void
RosPublisher::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}
