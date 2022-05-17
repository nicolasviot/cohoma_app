#include "ros_node_instantiate.h"

template class rclcpp::Subscription<icare_interfaces::msg::StringStamped>;
template class rclcpp::Subscription<icare_interfaces::msg::RobotState>;
template class rclcpp::Subscription<icare_interfaces::msg::GraphItineraryList>;
template class rclcpp::Subscription<icare_interfaces::msg::GraphItinerary>;
template class rclcpp::Subscription<icare_interfaces::msg::Tasks>;
template class rclcpp::Subscription<icare_interfaces::msg::Allocation>;
template class rclcpp::Subscription<icare_interfaces::msg::TrapList>;
template class rclcpp::Subscription<icare_interfaces::msg::Site>;
template class rclcpp::Subscription<icare_interfaces::msg::EnvironmentMap>;

template class rclcpp::Publisher<icare_interfaces::msg::PlanningRequest>;
template class rclcpp::Publisher<icare_interfaces::msg::StringStamped>;
template class rclcpp::Publisher<icare_interfaces::msg::Tasks>;
template class rclcpp::Publisher<icare_interfaces::msg::LimaCrossed>;
