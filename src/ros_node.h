#pragma once

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
#include "icare_interfaces/msg/trap_activation.hpp"
#endif

//Djnn-smala stuff
#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/property/text_property.h"
#include "core/property/double_property.h"
#include "core/property/int_property.h"
#include "core/property/bool_property.h"
#include "core/property/ref_property.h"
#include "core/tree/component.h"
#include "exec_env/external_source.h"
#include "core/control/native_action.h"
#include "gui/shape/image.h"
#include "gui/transformation/scaling.h"

//C++ stuff



using namespace djnn;

class RosNode : public FatProcess, public ExternalSource
  {
  public:
    RosNode (ParentProcess* parent, const string& n, CoreProcess* map, CoreProcess* context, CoreProcess* model_manager);
    ~RosNode() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  	
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
    void send_msg_trap_activation(int, bool);
    void send_msg_trap_deleted(int, bool);
    void send_msg_update_trap_position(int, double, double);
    void save_console();
  #endif
    void test_multiple_itineraries();
    void test_draw_visibility_map();
    void write_to_log (string, string);
    void deactivate_layer(string);
    void activate_layer(string);
 
   private:
    const std::string _topic_name;

    //Arguments
	  CoreProcess *_map, *_context, *_model_manager;
	  CoreProcess *_frame;

    //navgraph fields
    TextProperty navgraph_data;
    CoreProcess *_nodes, *_edges, *_shadow_edges;
    CoreProcess *_trap_models, *_node_models;

    CoreProcess *_task_edges, *_task_areas, *_task_traps;
    CoreProcess *_task_allocated_edges, *_task_allocated_areas, *_task_allocated_traps;
    CoreProcess *_exclusion_areas, *_limas; 
    CoreProcess *_layer_filter;

    // Models of itineraries
    CoreProcess *_shortest_itinerary, *_safest_itinerary, *_tradeoff_itinerary;
    std::vector<CoreProcess*> _itineraries;
    //Component *_itineraries_list;
    CoreProcess *_itineraries_list;
    RefProperty *_ref_curent_itinerary;
    NativeAction* _edge_released_na;

    // Models of vehicles, safety pilots
    CoreProcess *_vab, *_agilex1, *_agilex2, *_lynx, *_spot, *_drone;
    CoreProcess *_drone_safety_pilot, *_ground_safety_pilot;

    // fw = file writer
    CoreProcess *_clock, *_console, *_fw_input, *_fw_console_input;

    // In context
    RefProperty* _ref_NULL;
    RefProperty* _ref_current_node;
    RefProperty* _ref_current_trap;
    TextProperty* _selected_itinerary_id;

    //Planif VAB
    IntProperty _current_plan_id_vab;
    IntProperty _start_plan_vab_id;
    IntProperty _end_plan_vab_id;

    //Visibility_map
    CoreProcess* _georef_visibility_map;
    DoubleProperty* _visibility_map_resolution;
    DataImage* _visibility_map;
    

	  std::vector<ParentProcess*> navgraph_list;
  
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
    rclcpp::Publisher<icare_interfaces::msg::TrapActivation>::SharedPtr publisher_trap_activation;
    rclcpp::Publisher<icare_interfaces::msg::Trap>::SharedPtr publisher_trap_edit;
#endif
    
  };
