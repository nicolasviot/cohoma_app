#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "icare_interfaces/msg/string_stamped.hpp"

#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "exec_env/external_source.h"
#include "include/navgraph/navgraph.hpp"



using namespace djnn;

class GraphPublisher : public FatProcess, public ExternalSource
  {
  private:

    /* SEND ACTION */
    class SendMsgAction : public Action
    {
    public:
      SendMsgAction (ParentProcess* parent, const string& name) :
        Action (parent, name) {};
    
      virtual ~SendMsgAction () {}
      void impl_activate () override { ((GraphPublisher*)get_parent())->send_msg (); }
    };
  public:
    GraphPublisher (ParentProcess* parent, const string& n, const string& topic_name, CoreProcess* my_map, CoreProcess* manager);
    ~GraphPublisher() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  
    void send_msg ();

  private:
    TextProperty _msg;
    SendMsgAction _action;
    Coupling _c_msg;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<icare_interfaces::msg::StringStamped>::SharedPtr publisher_;
   // rclcpp::QoS qos;

  };
