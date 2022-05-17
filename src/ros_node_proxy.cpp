#include "ros_node_proxy.h"


#include <iostream>

//#include <fstream>



using std::placeholders::_1;



RosNodeProxy::RosNodeProxy (const string& n)


  #ifndef NO_ROS
  //ROS
: qos_best_effort(10),
qos(1),
qos_transient(1)
  #endif

{

  #ifndef NO_ROS
  _node = std::make_shared<rclcpp::Node>(n);
  // reliable ~= TCP connections => for the navgraph msgs
  // best_effort allows to drop some pacquets => for robot_state msgs

  
  qos.reliable();
  qos.durability_volatile();
  qos_best_effort.best_effort();
  qos_best_effort.durability_volatile();
  qos_transient.reliable();
  qos_transient.transient_local();
  #endif

}

void
RosNodeProxy::impl_activate ()
{ 

  #ifndef NO_ROS
  //subscriptions
  sub_navgraph =_node->create_subscription<icare_interfaces::msg::StringStamped>( 
  "/navgraph", qos_transient, std::bind(&RosNodeProxy::receive_msg_navgraph, this, _1)); //Replace 10 with qosbesteffort
  
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>(
    "/robot_state", qos_best_effort, std::bind(&RosNodeProxy::receive_msg_robot_state, this, _1));

  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItineraryList>(
    "/itinerary", qos, std::bind(&RosNodeProxy::receive_msg_graph_itinerary_loop, this, _1));

  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
    "/plan", qos, std::bind(&RosNodeProxy::receive_msg_graph_itinerary_final, this, _1));

  sub_candidate_tasks = _node->create_subscription<icare_interfaces::msg::Tasks>(
    "/candidate_tasks", qos, std::bind(&RosNodeProxy::receive_msg_allocated_tasks, this, _1));

  sub_allocation = _node->create_subscription<icare_interfaces::msg::Allocation>(
    "/allocation", qos, std::bind(&RosNodeProxy::receive_msg_allocation, this, _1));

  sub_traps = _node->create_subscription<icare_interfaces::msg::TrapList>(
    "/traps", qos_transient, std::bind(&RosNodeProxy::receive_msg_trap, this, _1));

  sub_site = _node->create_subscription<icare_interfaces::msg::Site>(
    "/site", qos_transient, std::bind(&RosNodeProxy::receive_msg_site, this, _1));

  sub_map = _node->create_subscription<icare_interfaces::msg::EnvironmentMap>(
  "map", qos_transient, std::bind(&RosNodeProxy::receive_msg_map, this, std::placeholders::_1));


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

void
RosNodeProxy::impl_deactivate ()
{
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

#ifndef NO_ROS

// callback for navgraph msg (contains the navigation graph)
void 
RosNodeProxy::receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg) {
  
}

#endif

  #if 0

void
RosNodeProxy::test_multiple_itineraries(){
  //debug
  //std::cerr << "in RosNode::test_multiple_itineraries - pointers  " << _itineraries_list  <<std::endl;

  //debug ros_msg
  std::vector<std::pair<string,std::vector<int>>> msg = { \
    {"toto", {2, 1, 0, 5, 6}}, \
    {"titi", {2, 10, 8, 6}}, \
    {"tata", {0, 1, 4, 7}}};

  //Color:
    int unselected = 0x232323;
    int selected = 0x1E90FF;

  //std::cerr << "in RosNode::test_multiple_itineraries - size before "  << _itineraries_list->children ().size () << " - ref  " << _edge_released_na  <<std::endl;

  //schedule delete old content
    int itineraries_list_size =  _itineraries_list->children ().size ();
    for (int i = itineraries_list_size - 1; i >= 0; i--) {
      auto *child = _itineraries_list->children ()[i];
      if (child) {
        child->deactivate ();
        if (child->get_parent ())
          child->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(child));
        child->schedule_delete ();
        child = nullptr;
      }
    }
    _ref_curent_itenerary->set_value ((CoreProcess*)nullptr, true);

  //std::cerr << "in RosNode::test_multiple_itineraries - size after "  << _itineraries_list->children ().size () <<std::endl;

  /*
    _itineraries_list {
      id { id;
          edges: liste of edges}
      id { id;
          edges: liste of edges}
      id { id;
           edges: liste of edges}
      ...
    }
  */

    string first_id = "";
    for (auto ros_itinerary : msg) {
    // get first id
      if (first_id == "")
        first_id = ros_itinerary.first;
      Component *new_itinerary = new Component ( _itineraries_list, ros_itinerary.first );
      new TextProperty (new_itinerary, "id", ros_itinerary.first);
      List* new_ite_edges = new List (new_itinerary, "edges");
      int ite_edges_size = ros_itinerary.second.size ();
      if ( ite_edges_size > 0) {
        for (int i = 1; i < ite_edges_size; i++) {
          ParentProcess* edge = Edge( new_ite_edges, "", ros_itinerary.second[i-1] + 1, ros_itinerary.second[i] + 1, 20, _nodes);
          ((AbstractProperty*) edge->find_child("color/value"))->set_value (unselected, true);
          new Binding (edge, "binding_edge_released", edge, "outerEdge/release", _edge_released_na, "");
        }
      }
    }
    _id_curent_itenerary->set_value (first_id, true);

  //debug
  //int itinerary_edges_size = dynamic_cast<IntProperty*> (_itinerary_edges->find_child ("size"))->get_value ();
  //std::cerr << "in RosNode::test_multiple_itineraries " <<  _itinerary_edges  << " - " << itinerary_edges_size <<std::endl;

  }
    #endif

#ifndef NO_ROS
  void 
  RosNodeProxy::receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg) {
 

  }

  void 
  RosNodeProxy::receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {
  
  
  }



//callback for robot_state msg (contains data on one robot)
  void 
  RosNodeProxy::receive_msg_robot_state(const icare_interfaces::msg::RobotState::SharedPtr msg) {
    RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);
  //Todo, use header.
  }



  void 
  RosNodeProxy::receive_msg_trap (const icare_interfaces::msg::TrapList msg){
    
  }

  void 
  RosNodeProxy::receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks msg){

    
  }



  void 
  RosNodeProxy::receive_msg_allocation(const icare_interfaces::msg::Allocation msg){
    
    //TODO
    //get_exclusive_access(DBG_GET);
    /* .. Traitement .. */
    //GRAPH_EXEC;
    //release_exclusive_access(DBG_REL);
  

  }

  

void 
RosNodeProxy::receive_msg_map(const icare_interfaces::msg::EnvironmentMap msg){
std::cerr << "received exploration map" << std::endl;

 float lat_center = msg.origin.latitude;
 float lon_center = msg.origin.longitude; 
std::cerr << lat_center << std::endl;
std::cerr << lon_center << std::endl;




}

//void 
//RosNodeProxy::receive_msg_site(const icare_interfaces::msg::Site msg){
//  }



#endif
