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
#endif

//Djnn-smala stuff
#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "core/tree/double_property.h"
#include "core/tree/int_property.h"
#include "core/tree/bool_property.h"
#include "exec_env/external_source.h"


//C++ stuff
#ifndef NO_LEMON
#include "include/navgraph/navgraph.hpp"
#endif


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
    
    //TODOS
    void receive_msg_graph_itinerary (const icare_interfaces::msg::GraphItinerary msg);
    void receive_msg_trap (const icare_interfaces::msg::Trap msg);
  #endif
  
    void send_msg_planning_request();
    //send navgraph update
    void send_msg_navgraph_update();
    void send_validation_plan();

 
   private:
    const std::string _topic_name;

    //Arguments
	  CoreProcess* _map, *_manager;
	

    //navgraph fields
    TextProperty navgraph_data;
    
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
    rclcpp::Publisher<icare_interfaces::msg::PlanningRequest>::SharedPtr publisher_planning_request;
#endif
    
  };
