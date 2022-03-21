#include <functional>
#include <memory>
#include <string>

#ifndef NO_ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "icare_interfaces/msg/robot_state.hpp"
/*#include "icare_interfaces/msg/rosidl_generator_c__visibility_control.h"
#include "icare_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "icare_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
*/
#endif

#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "exec_env/external_source.h"

using namespace djnn;

class RosPublisher : public FatProcess, public ExternalSource
  {
  private:

    /* SEND ACTION */
    class SendMsgAction : public Action
    {
    public:
      SendMsgAction (ParentProcess* parent, const string& name) :
        Action (parent, name) {};
    
      virtual ~SendMsgAction () {}
      void impl_activate () override { ((RosPublisher*)get_parent())->send_msg (); }
    };
  public:
    RosPublisher (ParentProcess* parent, const string& n, const string& topic_name);
    ~RosPublisher() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  
    void send_msg ();

  private:
    TextProperty _msg;
    SendMsgAction _action;
    Coupling _c_msg;
#ifndef NO_ROS
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<icare_interfaces::msg::RobotState>::SharedPtr publisher_;
#endif
  };
