#include <functional>
#include <memory>
#include <string>

#ifndef NO_ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "icare_interfaces/msg/string_stamped.hpp"
#endif

//#include "include/navgraph/navgraph.hpp"


#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "core/tree/double_property.h"
#include "core/tree/int_property.h"
#include "core/tree/bool_property.h"
#include "exec_env/external_source.h"



using namespace djnn;

class GraphSubscriber : public FatProcess, public ExternalSource
  {
  public:
    GraphSubscriber (ParentProcess* parent, const string& n, const string& topic_name, CoreProcess* map, CoreProcess* manager);
    ~GraphSubscriber() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;

#ifndef NO_ROS  
    void receive_msg (const icare_interfaces::msg::StringStamped::SharedPtr msg);
#endif

   private:
    const std::string _topic_name;
    std::string _string_data;
    TextProperty _data;
    CoreProcess* _map, *_manager;

#ifndef NO_ROS
    rclcpp::Node::SharedPtr _node;
   // rclcpp::QoS qosbesteffort;
    rclcpp::Subscription<icare_interfaces::msg::StringStamped>::SharedPtr subscription_;
    std::vector<ParentProcess*> navgraph_list;
    rclcpp::QoS qos;
#endif
 //   navgraph::NavGraph _navgraph;

  };
