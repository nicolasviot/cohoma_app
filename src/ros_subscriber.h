#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "icare_interfaces/msg/robot_state.hpp"
#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "core/tree/double_property.h"
#include "core/tree/int_property.h"
#include "core/tree/bool_property.h"
#include "exec_env/external_source.h"

using namespace djnn;

class RosSubscriber : public FatProcess, public ExternalSource
  {
  public:
    RosSubscriber (ParentProcess* parent, const string& n, const string& topic_name);
    ~RosSubscriber() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  
    void receive_msg (const icare_interfaces::msg::RobotState::SharedPtr msg);
 
   private:
    const std::string _topic_name;
    TextProperty _msg;
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
    rclcpp::Node::SharedPtr _node;
    rclcpp::QoS qosbesteffort;
    rclcpp::Subscription<icare_interfaces::msg::RobotState>::SharedPtr subscription_;
  };
