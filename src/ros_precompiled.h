//#include <iostream>

#include "core/utils/build/precompiled.hpp"

#ifndef NO_ROS
//Ros Stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
//robot_state msg interface : subscriber
#include "icare_interfaces/msg/robot_state.hpp"
//navgraph/general string  msg interface : publisher/subscriber
#include "icare_interfaces/msg/string_stamped.hpp"
//VAB planif request : publisher
//#include "icare_interfaces/msg/planning_request.hpp"
//VAB planif output : subscriber
#include "icare_interfaces/msg/graph_itinerary.hpp"
#include "icare_interfaces/msg/graph_itinerary_list.hpp"
//Traps : subscriber 
#include "icare_interfaces/msg/trap.hpp"
#include "icare_interfaces/msg/trap_list.hpp"
//Tasks : subscriber / publisher 
#include "icare_interfaces/msg/tasks.hpp"
//Allocation : subscriber
#include "icare_interfaces/msg/allocation.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "icare_interfaces/msg/lima_crossed.hpp"
#include "icare_interfaces/msg/site.hpp"
#include "icare_interfaces/msg/exploration_polygon.hpp"
#include "icare_interfaces/msg/environment_map.hpp"

//#include <nlohmann/json.hpp>

#include "ros_node_instantiate.h" // will explicitely instantiate stuff

#endif

/*template class icare_interfaces::msg::StringStamped_<std::allocator<void>>;


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
*/
//class RosNode;

/*template
rclcpp::Subscription<icare_interfaces::msg::StringStamped_<std::allocator<void>>>::SharedPtr
rclcpp::Node::create_subscription<icare_interfaces::msg::StringStamped_<std::allocator<void>>, std::_Bind<void (RosNode::*(RosNode *, std::_Placeholder<1>))(std::shared_ptr<icare_interfaces::msg::StringStamped_<std::allocator<void>>>)>, std::allocator<void>, icare_interfaces::msg::StringStamped_<std::allocator<void>>, rclcpp::Subscription<icare_interfaces::msg::StringStamped_<std::allocator<void>>>, rclcpp::message_memory_strategy::MessageMemoryStrategy<icare_interfaces::msg::StringStamped_<std::allocator<void>>>>;*/

