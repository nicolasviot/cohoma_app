#pragma once

#include <functional>
#include <memory>
#include <string>

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
#include "icare_interfaces/msg/planning_request.hpp"
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
#endif


//C++ stuff



using namespace djnn;

struct RosNodeProxy
  {
  public:
    RosNodeProxy (const string& name);

    void impl_activate ();
    void impl_deactivate ();

  	
  #ifndef NO_ROS
    virtual void receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg);
    virtual void receive_msg_robot_state (const icare_interfaces::msg::RobotState::SharedPtr msg);
    virtual void receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg);
    virtual void receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg);
    virtual void receive_msg_trap (const icare_interfaces::msg::TrapList msg);
    virtual void receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks);
    virtual void receive_msg_allocation(const icare_interfaces::msg::Allocation);
    virtual void receive_msg_site(const icare_interfaces::msg::Site) = 0;
    virtual void receive_msg_map(const icare_interfaces::msg::EnvironmentMap);


  #endif

 
   protected:
    const std::string _topic_name;
  
#ifndef NO_ROS
    //Ros
    rclcpp::Node::SharedPtr _node;
    rclcpp::QoS qos_best_effort;
    rclcpp::QoS qos;
    rclcpp::QoS qos_transient;


    rclcpp::Subscription<icare_interfaces::msg::StringStamped>::SharedPtr sub_navgraph;
	  rclcpp::Subscription<icare_interfaces::msg::RobotState>::SharedPtr sub_robot_state;
    rclcpp::Subscription<icare_interfaces::msg::GraphItineraryList>::SharedPtr sub_graph_itinerary_loop;
    rclcpp::Subscription<icare_interfaces::msg::GraphItinerary>::SharedPtr sub_graph_itinerary_final;
    rclcpp::Subscription<icare_interfaces::msg::Tasks>::SharedPtr sub_candidate_tasks;
    rclcpp::Subscription<icare_interfaces::msg::Allocation>::SharedPtr sub_allocation;
    rclcpp::Subscription<icare_interfaces::msg::TrapList>::SharedPtr sub_traps;
    rclcpp::Subscription<icare_interfaces::msg::Site>::SharedPtr sub_site;
    rclcpp::Subscription<icare_interfaces::msg::EnvironmentMap>::SharedPtr sub_map;

    rclcpp::Publisher<icare_interfaces::msg::PlanningRequest>::SharedPtr publisher_planning_request;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_navgraph_update;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_validation;
    rclcpp::Publisher<icare_interfaces::msg::Tasks>::SharedPtr publisher_tasks;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_validation_tasks;
    rclcpp::Publisher<icare_interfaces::msg::LimaCrossed>::SharedPtr publisher_lima;
#endif
    
  };
