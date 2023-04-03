#pragma once

#ifndef NO_ROS
//Ros Stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

//robot_state msg interface : subscriber
#include "icare_interfaces/msg/robot_state.hpp"
//robot_config msg subscriber
#include "icare_interfaces/msg/robot_config.hpp"

// group config publisher
#include "icare_interfaces/msg/group_config.hpp"

//navgraph/general string  msg interface : publisher/subscriber
#include "icare_interfaces/msg/string_stamped.hpp"

//VAB itinerary request : publisher
#include "icare_interfaces/msg/itinerary_request.hpp"
//VAB itinerary output : subscriber
#include "icare_interfaces/msg/itinerary.hpp"

//Traps : subscriber 
#include "icare_interfaces/msg/trap.hpp"
#include "icare_interfaces/msg/trap_activation.hpp"
#include "icare_interfaces/msg/trap_cluster.hpp"
#include "icare_interfaces/msg/trap_detection.hpp"
#include "icare_interfaces/msg/trap_identification.hpp"
#include "icare_interfaces/msg/trap_set.hpp"
#include "icare_interfaces/msg/trap_set_deactivation_action.hpp"
#include "icare_interfaces/msg/trap_set_identification_mode.hpp"

//Tasks : subscriber / publisher 
#include "icare_interfaces/msg/tasks.hpp"

//subscriber
#include "icare_interfaces/msg/allocation.hpp"
#include "icare_interfaces/msg/allocated_task.hpp"
#include "icare_interfaces/msg/site.hpp"
#include "icare_interfaces/msg/exploration_polygon.hpp"
#include "icare_interfaces/msg/geo_polygon.hpp"
#include "icare_interfaces/msg/environment_map.hpp"
#include "icare_interfaces/msg/graph_edge.hpp"
#include "icare_interfaces/msg/restricted_zone.hpp"


//chat
#include "icare_interfaces/msg/chat_message.hpp"
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
    RosNode (CoreProcess* parent, const string& n, CoreProcess* map, CoreProcess* context, CoreProcess* model_manager);
    ~RosNode() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;

  
#ifndef NO_ROS
    //Ros
    rclcpp::Node::SharedPtr _node;
    rclcpp::QoS qos_best_effort;
    rclcpp::QoS qos;
    rclcpp::QoS qos_transient;

    rclcpp::Subscription<icare_interfaces::msg::StringStamped>::SharedPtr sub_navgraph;
    rclcpp::Subscription<icare_interfaces::msg::Site>::SharedPtr sub_site;
    rclcpp::Subscription<icare_interfaces::msg::EnvironmentMap>::SharedPtr sub_map;
    rclcpp::Publisher<icare_interfaces::msg::Lima>::SharedPtr publisher_lima;
    rclcpp::Publisher<icare_interfaces::msg::RestrictedZone>::SharedPtr publisher_restricted_zone;

    rclcpp::Subscription<icare_interfaces::msg::RobotState>::SharedPtr sub_robot_state;
    rclcpp::Subscription<icare_interfaces::msg::RobotConfig>::SharedPtr sub_robot_config;

    rclcpp::Subscription<icare_interfaces::msg::ChatMessage>::SharedPtr sub_chat;
    rclcpp::Publisher<icare_interfaces::msg::ChatMessage>::SharedPtr publisher_chat;

    rclcpp::Subscription<icare_interfaces::msg::Itinerary>::SharedPtr sub_itinerary;
    rclcpp::Publisher<icare_interfaces::msg::ItineraryRequest>::SharedPtr publisher_itinerary_request;
    
    rclcpp::Publisher<icare_interfaces::msg::GroupConfig>::SharedPtr publisher_group_config;

    rclcpp::Subscription<icare_interfaces::msg::TrapDetection>::SharedPtr sub_trap_detection;
    rclcpp::Subscription<icare_interfaces::msg::Trap>::SharedPtr sub_trap_update;
    
    rclcpp::Publisher<icare_interfaces::msg::TrapActivation>::SharedPtr publisher_trap_activation;
    rclcpp::Publisher<icare_interfaces::msg::Trap>::SharedPtr publisher_trap_delete;
    rclcpp::Publisher<icare_interfaces::msg::Trap>::SharedPtr publisher_trap_add;
    
    rclcpp::Publisher<icare_interfaces::msg::TrapSetIdentificationMode>::SharedPtr publisher_trap_set_identification_mode;
    rclcpp::Publisher<icare_interfaces::msg::TrapSetIdentificationMode>::SharedPtr publisher_trap_set_confirmation_mode;
    
    rclcpp::Publisher<icare_interfaces::msg::TrapSetDeactivationAction>::SharedPtr publisher_trap_set_deactivation_action;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_trap_set_clustering_distance;
    
    /*

    
	  clcpp::Subscription<icare_interfaces::msg::GraphItineraryList>::SharedPtr sub_graph_itinerary_loop;
    rclcpp::Subscription<icare_interfaces::msg::GraphItinerary>::SharedPtr sub_graph_itinerary_final;
    rclcpp::Subscription<icare_interfaces::msg::Tasks>::SharedPtr sub_candidate_tasks;
    rclcpp::Subscription<icare_interfaces::msg::Allocation>::SharedPtr sub_allocation;
    rclcpp::Subscription<icare_interfaces::msg::TrapList>::SharedPtr sub_traps;
    
    rclcpp::Publisher<icare_interfaces::msg::PlanningRequest>::SharedPtr publisher_planning_request;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_navgraph_update;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_validation;
    rclcpp::Publisher<icare_interfaces::msg::Tasks>::SharedPtr publisher_tasks;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_validation_tasks;
    rclcpp::Publisher<icare_interfaces::msg::TrapActivation>::SharedPtr publisher_trap_activation;
    rclcpp::Publisher<icare_interfaces::msg::Trap>::SharedPtr publisher_trap_edit;
    */

  //navgraph map and Site
  void receive_msg_navgraph (const icare_interfaces::msg::StringStamped& msg);
  void receive_msg_site(const icare_interfaces::msg::Site& msg);
  void receive_msg_map(const icare_interfaces::msg::EnvironmentMap& msg);
  void send_msg_lima_clearance(string name, bool crossing_allowed);
  void send_msg_zone_activation(string name, bool is_active);
    
  //robots
  void receive_msg_robot_state (const icare_interfaces::msg::RobotState& msg);
  void receive_msg_robot_config (const icare_interfaces::msg::RobotConfig& msg);

  //chat
  void receive_msg_chat(const icare_interfaces::msg::ChatMessage& msg);
  void send_msg_chat(string _text, int _type, double _lat, double _lng, double _alt);

  //itineraryqos_best_effort
  void send_msg_itinerary_request();
  void receive_msg_itinerary (const icare_interfaces::msg::Itinerary& msg);

  //groupe
  void send_msg_group_config();

  //traps
  void receive_msg_trap_detection(const icare_interfaces::msg::TrapDetection& msg);
  void receive_msg_trap_update(const icare_interfaces::msg::Trap& msg);
  void send_msg_trap_activation(int uid , bool is_active);
  void send_msg_trap_delete(int uid);
  void send_msg_trap_add(int uit, double _lat, double _lng);
  void send_msg_trap_set_identification_mode(int uid, int ident_mode);
  void send_msg_trap_set_confirmation_mode(int uid, int confirmation_mode);
  void send_msg_trap_set_deactivation_action(int uid, int action);
  void send_msg_trap_set_clustering_distance(double clustering_distance);
  

   /*
   void receive_msg_robot_state (const icare_interfaces::msg::RobotState::SharedPtr msg);
    void receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg);
    void receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg);
    void receive_msg_trap (const icare_interfaces::msg::TrapList msg);
    void receive_msg_candidate_tasks (const icare_interfaces::msg::Tasks);
    void receive_msg_allocation(const icare_interfaces::msg::Allocation);
    void receive_msg_site(const icare_interfaces::msg::Site);
    void receive_msg_map(const icare_interfaces::msg::EnvironmentMap);

    void send_msg_navgraph_update();
    void send_validation_plan();
    void send_selected_tasks();
    void send_validation_tasks();
    void send_msg_lima(int);
    void send_msg_trap_activation(int, bool);
    void send_msg_trap_deleted(int);
    void send_msg_update_trap_position(int, double, double);
    void save_console();
    */
  #endif
 
   private:
    const std::string _topic_name;

    // Arguments
	  CoreProcess *_map, *_context, *_model_manager;
	  CoreProcess *_frame;

    // (sub) Layers
    CoreProcess *_layer_models;

    // SITE
    CoreProcess *_limit_models, *_zone_models, *_lima_models;
    
    // Navigation graph
    CoreProcess *_node_ids, *_edge_ids;
    CoreProcess *_node_models, *_edge_models;

    //Chat
    CoreProcess *_chat_models;

    // TASKS
    CoreProcess *_task_edge_models, *_task_area_models, *_task_trap_models;

    // TRAPS
    CoreProcess *_trap_models;
    
    // Models of itineraries
    std::vector<CoreProcess*> _itineraries;

    //Models of Operators
    CoreProcess *_ot, *_og1, *_og2, *_og3;
    
    // Models of vehicles, safety pilots
    CoreProcess *_vab, *_bnx8, *_agilex1, *_agilex2, *_lynx, *_agilex3, *_minnie, *_m600, *_spot, *_long_eye, *_pprz;
    CoreProcess *_drone_safety_pilot, *_ground_safety_pilot;

    // fw = file writer
    CoreProcess *_console, *_fw_input, *_fw_console_input;

    // In context
    RefProperty* _ref_NULL;
    RefProperty *_ref_node_graph_edition, *_ref_node_status_edition;
    RefProperty* _ref_current_trap;
    TextProperty* _selected_itinerary_id;

    // Planif VAB
    IntProperty _current_plan_id_vab;
    IntProperty _start_plan_vab_id;
    IntProperty _end_plan_vab_id;

    // Visibility Map
    CoreProcess* _result_layer;
    DataImage* _image;
    
  };
