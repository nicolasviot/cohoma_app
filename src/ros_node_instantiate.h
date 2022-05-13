#pragma once

#include "ros_node.h"

using std::placeholders::_1;

struct ros_node_instantiate {
#ifndef NO_ROS
    void receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg);
    void receive_msg_robot_state (const icare_interfaces::msg::RobotState::SharedPtr msg);
    void receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg);
    void receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg);
    void receive_msg_trap (const icare_interfaces::msg::TrapList msg);
    void receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks);
    void receive_msg_allocation(const icare_interfaces::msg::Allocation);
    void receive_msg_site(const icare_interfaces::msg::Site);
    void receive_msg_map(const icare_interfaces::msg::EnvironmentMap);


    void send_msg_planning_request();
    void send_msg_navgraph_update();
    void send_validation_plan();
    void send_selected_tasks();
    void send_validation_tasks();
    void send_msg_lima(int);
  #endif

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

    void activate(RosNode* rosnode) {
    	#ifndef NO_ROS
		  //subscriptions
		  sub_navgraph =_node->create_subscription<icare_interfaces::msg::StringStamped>( 
		  "/navgraph", qos_transient, std::bind(&RosNode::receive_msg_navgraph, rosnode, _1)); //Replace 10 with qosbesteffort
		  
		  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>(
		    "/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, rosnode, _1));

		  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItineraryList>(
		    "/itinerary", qos, std::bind(&RosNode::receive_msg_graph_itinerary_loop, rosnode, _1));

		  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
		    "/plan", qos, std::bind(&RosNode::receive_msg_graph_itinerary_final, rosnode, _1));

		  sub_candidate_tasks = _node->create_subscription<icare_interfaces::msg::Tasks>(
		    "/candidate_tasks", qos, std::bind(&RosNode::receive_msg_allocated_tasks, rosnode, _1));

		  sub_allocation = _node->create_subscription<icare_interfaces::msg::Allocation>(
		    "/allocation", qos, std::bind(&RosNode::receive_msg_allocation, rosnode, _1));

		  sub_traps = _node->create_subscription<icare_interfaces::msg::TrapList>(
		    "/traps", qos_transient, std::bind(&RosNode::receive_msg_trap, rosnode, _1));

		  sub_site = _node->create_subscription<icare_interfaces::msg::Site>(
		    "/site", qos_transient, std::bind(&RosNode::receive_msg_site, rosnode, _1));

		  sub_map = _node->create_subscription<icare_interfaces::msg::EnvironmentMap>(
		  "map", qos_transient, std::bind(&RosNode::receive_msg_map, rosnode, std::placeholders::_1));


		  publisher_planning_request =_node->create_publisher<icare_interfaces::msg::PlanningRequest>(
		    "/planning_request", qos);
		  publisher_validation = _node->create_publisher<icare_interfaces::msg::StringStamped>(
		    "/validation", qos);
		  publisher_navgraph_update = _node->create_publisher<icare_interfaces::msg::StringStamped>(
		    "/navgraph_update", qos_transient);
		  publisher_tasks = _node->create_publisher<icare_interfaces::msg::Tasks>(
		    "/tasks", qos);
		  publisher_validation_tasks = _node->create_publisher<icare_interfaces::msg::StringStamped>(
		    "/validate", qos);
		  publisher_lima = _node->create_publisher<icare_interfaces::msg::LimaCrossed>(
		    "/lima", qos);
		  #endif

    }


    void deactivate() {
     // Here we should disable the subcription but it 
	  // seems there is no way to do it properly
	  // some insights here:
	  // https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/

	#ifndef NO_ROS
	  sub_navgraph.reset ();
	  sub_robot_state.reset ();
	  sub_graph_itinerary_loop.reset ();
	  sub_graph_itinerary_final.reset ();
	  sub_candidate_tasks.reset ();
	  sub_allocation.reset ();

	#endif
	}

};
