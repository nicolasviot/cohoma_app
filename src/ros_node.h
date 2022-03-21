#include <functional>
#include <memory>
#include <string>
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
#include "include/navgraph/navgraph.hpp"


using namespace djnn;

class RosNode : public FatProcess, public ExternalSource
  {
  public:
    RosNode (ParentProcess* parent, const string& n, CoreProcess* map, CoreProcess* manager);
    ~RosNode() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  	
    void receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg);
    void receive_msg_robot_state (const icare_interfaces::msg::RobotState::SharedPtr msg);
    
    //TODOS
    void receive_msg_graph_itinerary (const icare_interfaces::msg::GraphItinerary msg);
    void receive_msg_trap (const icare_interfaces::msg::Trap msg);
  
    void send_msg_planning_request();
    void send_msg_navgraph_update();

 
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
    

	std::vector<ParentProcess*> navgraph_list;
    
    //Ros
    rclcpp::Node::SharedPtr _node;
    rclcpp::QoS qos_best_effort;
    rclcpp::QoS qos;
    rclcpp::Subscription<icare_interfaces::msg::StringStamped>::SharedPtr sub_navgraph;
	rclcpp::Subscription<icare_interfaces::msg::RobotState>::SharedPtr sub_robot_state;
    
    
  };
