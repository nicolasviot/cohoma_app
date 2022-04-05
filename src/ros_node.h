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
//Traps : subscriber 
#include "icare_interfaces/msg/trap.hpp"
//Tasks : subscriber / publisher 
#include "icare_interfaces/msg/tasks.hpp"
//Allocation : subscriber
#include "icare_interfaces/msg/allocation.hpp"
#endif

//Djnn-smala stuff
#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "core/tree/double_property.h"
#include "core/tree/int_property.h"
#include "core/tree/bool_property.h"
#include "core/tree/ref_property.h"
#include "core/tree/component.h"
#include "exec_env/external_source.h"
#include "core/control/native_action.h"


//C++ stuff



using namespace djnn;

class RosNode : public FatProcess, public ExternalSource
  {
  public:
    RosNode (ParentProcess* parent, const string& n, CoreProcess* map, CoreProcess* manager);
    ~RosNode() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  	
  #ifndef NO_ROS
    void receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg);
    void receive_msg_robot_state (const icare_interfaces::msg::RobotState::SharedPtr msg);
    void receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItinerary::SharedPtr msg);
    void receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg);
    void receive_msg_trap (const icare_interfaces::msg::Trap msg);
    void receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks);
    void receive_msg_allocation(const icare_interfaces::msg::Allocation);


    void send_msg_planning_request();
    void send_msg_navgraph_update();
    void send_validation_plan();
    void send_selected_tasks();
    void send_validation_tasks();
    #endif
    void test_multiple_itineraries();

 
   private:
    const std::string _topic_name;

    //Arguments
	  CoreProcess* _map, *_manager;
	

    //navgraph fields
    TextProperty navgraph_data;
    CoreProcess *_nodes, *_edges, *_shadow_edges; 
    Component *_itineraries_list;
    RefProperty *_ref_curent_itenerary;
    NativeAction* _edge_released_na;

    //robot
    CoreProcess* _vab, *_agilex1, *_agilex2, *_lynx, *_spot, *_drone;

    RefProperty *_current_wpt, *_entered_wpt;
    IntProperty *_id_curent_itenerary;
    
    //robot_state fields
    IntProperty _robot_id;
    DoubleProperty _latitude;
    DoubleProperty _longitude;
    IntProperty _battery_percentage;
    DoubleProperty _battery_voltage;
    DoubleProperty _altitude_msl;
    DoubleProperty _compass_heading;
    BoolProperty _emergency_stop;
    BoolProperty _failsafe;
    IntProperty _operation_mode;

    //Planif VAB
    IntProperty _current_plan_id_vab;
    IntProperty _start_plan_vab_id;
    IntProperty _end_plan_vab_id;
    

	  std::vector<ParentProcess*> navgraph_list;
  
#ifndef NO_ROS
    //Ros
    rclcpp::Node::SharedPtr _node;
    rclcpp::QoS qos_best_effort;
    rclcpp::QoS qos;
    rclcpp::Subscription<icare_interfaces::msg::StringStamped>::SharedPtr sub_navgraph;
	  rclcpp::Subscription<icare_interfaces::msg::RobotState>::SharedPtr sub_robot_state;
    rclcpp::Subscription<icare_interfaces::msg::GraphItinerary>::SharedPtr sub_graph_itinerary_loop;
    rclcpp::Subscription<icare_interfaces::msg::GraphItinerary>::SharedPtr sub_graph_itinerary_final;
    rclcpp::Subscription<icare_interfaces::msg::Tasks>::SharedPtr sub_candidate_tasks;
    rclcpp::Subscription<icare_interfaces::msg::Allocation>::SharedPtr sub_allocation;

    rclcpp::Publisher<icare_interfaces::msg::PlanningRequest>::SharedPtr publisher_planning_request;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_navgraph_update;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_validation;
    rclcpp::Publisher<icare_interfaces::msg::Tasks>::SharedPtr publisher_tasks;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_validation_tasks;
#endif
    
  };
