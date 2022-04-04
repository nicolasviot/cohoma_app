#include "ros_node.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"
#include "core/tree/list.h"

#include <nlohmann/json.hpp>


#include "Node.h"
#include "NavGraph.h"
#include "Edge.h"
#include "math.h"


using std::placeholders::_1;

using namespace djnn;



RosNode::RosNode (ParentProcess* parent, const string& n, CoreProcess* my_map, CoreProcess* manager) :
  FatProcess (n),
  ExternalSource (n),
  //arguments
  _map (my_map),
  _manager (manager),

  //navgraph fields
  navgraph_data(this, "data", ""),

  //robot_state fields
   _robot_id(this, "robot_id", 0),
  _battery_percentage(this, "battery_percentage", 0),
  _battery_voltage(this, "battery_voltage", 0),
  _latitude(this, "latitude", 0),
  _longitude(this, "longitude", 0),
  _altitude_msl(this, "altitude_msl", 0),
  _compass_heading(this, "compass_heading", 0),
  _emergency_stop(this, "emergency_stop", 0),
  _failsafe(this, "failsafe", 0),
  _operation_mode(this, "operation_mode", 0),

  //Planif VAB
  _current_plan_id_vab(this, "current_plan_id", 0),
  _start_plan_vab_id(this, "start_plan_id", 0),
  _end_plan_vab_id(this, "end_plan_id", 0)

  #ifndef NO_ROS
  //ROS
  ,qos_best_effort(10),
  qos(1)
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
  #endif

  finalize_construction (parent, n);
}

void
RosNode::impl_activate ()
{ 

  #ifndef NO_ROS
  //subscriptions
  sub_navgraph =_node->create_subscription<icare_interfaces::msg::StringStamped>( 
  "/navgraph", qos, std::bind(&RosNode::receive_msg_navgraph, this, _1)); //Replace 10 with qosbesteffort
  
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>(
    "/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, this, _1));

  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
    "/itinerary", qos, std::bind(&RosNode::receive_msg_graph_itinerary_loop, this, _1));

  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>(
    "/plan", qos, std::bind(&RosNode::receive_msg_graph_itinerary_final, this, _1));

  sub_candidate_tasks = _node->create_subscription<icare_interfaces::msg::Tasks>(
    "/candidate_tasks", qos, std::bind(&RosNode::receive_msg_allocated_tasks, this, _1));

  sub_allocation = _node->create_subscription<icare_interfaces::msg::Allocation>(
    "/allocation", qos, std::bind(&RosNode::receive_msg_allocation, this, _1));


  publisher_planning_request =_node->create_publisher<icare_interfaces::msg::PlanningRequest>(
    "/planning_request", qos);
  publisher_validation = _node->create_publisher<icare_interfaces::msg::StringStamped>(
    "/validation", qos);
  publisher_navgraph_update = _node->create_publisher<icare_interfaces::msg::StringStamped>(
    "/navgraph_update", qos);
  publisher_tasks = _node->create_publisher<icare_interfaces::msg::Tasks>(
    "/tasks", qos);
  publisher_validation_tasks = _node->create_publisher<icare_interfaces::msg::StringStamped>(
    "/validate", qos);
  #endif


  //activate navgraph fields
  navgraph_data.activate();


  //activate robot_state fields
  _robot_id.activate();
  _battery_percentage.activate();
  _battery_voltage.activate();
  _latitude.activate();
  _longitude.activate();
  _altitude_msl.activate();
  _compass_heading.activate();
  _emergency_stop.activate();
  _failsafe.activate();
  _operation_mode.activate();

  _current_plan_id_vab.activate();
  _start_plan_vab_id.activate();
  _end_plan_vab_id.activate();

  _nodes = _parent->find_child ("parent/l/map/layers/navgraph/nodes");
  _edges = _parent->find_child ("parent/l/map/layers/navgraph/edges");
  _shadow_edges = _parent->find_child ("parent/l/map/layers/navgraph/shadow_edges");
  _itinerary_edges = _parent->find_child("parent/l/map/layers/itineraries/itinerary_unique");
  
  _vab = _parent->find_child("parent/l/map/layers/satelites/vab");
  _agilex1 = _parent->find_child("parent/l/map/layers/satelites/agilex1");
  _agilex2 = _parent->find_child("parent/l/map/layers/satelites/agilex2");
  _lynx = _parent->find_child("parent/l/map/layers/satelites/lynx");
  _spot = _parent->find_child("parent/l/map/layers/satelites/spot");
  _drone = _parent->find_child("parent/l/map/layers/satelites/drone");

  _current_wpt = dynamic_cast<RefProperty*> (_parent->find_child ("parent/l/map/layers/navgraph/manager/current_wpt"));
  _entered_wpt = dynamic_cast<RefProperty*> (_parent->find_child ("parent/l/map/layers/navgraph/manager/entered_wpt"));

  _curent_itenerary = dynamic_cast<IntProperty*> (_parent->find_child ("parent/l/map/layers/navgraph/manager/current_itenerary"));

  //start the thread
  ExternalSource::start ();  
}

void
RosNode::impl_deactivate ()
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
  //deactivate navgraph fields
  navgraph_data.deactivate();

  //deactivate robot_state fields
    _robot_id.deactivate();
  _battery_percentage.deactivate();
  _battery_voltage.deactivate();
  _latitude.deactivate();
  _longitude.deactivate();
  _altitude_msl.deactivate();
  _compass_heading.deactivate();
  _emergency_stop.deactivate();
  _failsafe.deactivate();
  _operation_mode.deactivate();

  _current_plan_id_vab.deactivate();
  _start_plan_vab_id.deactivate();
  _end_plan_vab_id.deactivate();

  ExternalSource::please_stop ();
}

#ifndef NO_ROS

// callback for navgraph msg (contains the navigation graph)
void 
RosNode::receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg) {
  get_exclusive_access(DBG_GET);

  _current_wpt->set_value ((CoreProcess*)nullptr, true);
	_entered_wpt->set_value ((CoreProcess*)nullptr, true);

  Container *_edge_container = dynamic_cast<Container *> (_edges);
  if (_edge_container) {
    int _edge_container_size = _edge_container->children ().size ();
    for (int i = _edge_container_size - 1; i >= 0; i--) {
      auto *item = _edge_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

  Container *_shadow_edges_container = dynamic_cast<Container *>( _shadow_edges);
  if (_shadow_edges_container) {
    int _shadow_edges_container_size = _shadow_edges_container->children ().size ();
    for (int i = _shadow_edges_container_size - 1; i >= 0; i--) {
      auto *item = _shadow_edges_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

  Container *_nodes_container = dynamic_cast<Container *> (_nodes);
  if (_nodes_container) {
    int _nodes_container_size = _nodes_container->children ().size ();
    for (int i = _nodes_container_size - 1; i >= 0; i--) {
      auto *item = _nodes_container->children ()[i];
      if (item) {
        item->deactivate ();
        if (item->get_parent ())
          item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));
        item->schedule_delete ();
        item = nullptr;
      }
    }
  }

    nlohmann::json j = nlohmann::json::parse(msg->data);
    nlohmann::json j_graph;
    if (j.contains("graph"))
        j_graph = j["graph"];
    else if (j.contains("graphs")) {
        if (j.size() > 1) {
            std::cerr << "Several graphs defined in the JSON structure! loading the first one..." << std::endl;
            j_graph = j["graphs"][0];
        }
        else if (j.size() == 0) {
            std::cerr << "No graph defined in the JSON structure!" << std::endl;
            return;
        }
    }
    std::cerr << "about to get graph attributes" << std::endl;
    // graph attributes
    if (j_graph.contains("directed") && j_graph["directed"].get<bool>()) {
        std::cerr << "graph is said to be directed! NavGraph are only undirected: the results graph may not be what expected!" << std::endl;
    }
    std::cerr << "about to parse nodes" << std::endl;
    // nodes
    for (auto& node: j_graph["nodes"]) {
        std::cerr << "in from json parsing nodes" << std::endl;
        auto& m = node["metadata"];
        bool isPPO = m["compulsory"].get<bool>();
        ParentProcess* node_ = Node(_nodes, "", _map , m["latitude"].get<double>(), m["longitude"].get<double>(), m["altitude"].get<double>(),
       0, node["label"], std::stoi(node["id"].get<std::string>()) + 1, _manager);

  
    }
    std::cerr << "about to parse edges" << std::endl;
    // edges
    for (auto& edge: j_graph["edges"]) {


      std::string source = edge["source"].get<std::string>();
      std::string target = edge["target"].get<std::string>();
      auto& m = edge["metadata"];
      double length =m["length"].get<double>();
      ParentProcess* edge_ = Edge(_edges, "", std::stoi(source) + 1, 
        std::stoi(target) + 1,length, _nodes);
       
    }
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

#endif
void
RosNode::test_multiple_itineraries(){


  std::vector<std::vector<int>> msg = {{2, 1, 0, 5, 6}, {2, 10, 8, 6}, {0, 1, 4, 7}};

  //Color:
  int unselected = 0x232323;
  int selected = 0x1E90FF;
  
  int id = 0;
  for (auto itenerary : msg) {
    List* new_ite = new List (_itinerary_edges, "");
    int ite_size = itenerary.size ();
    if ( ite_size > 0) {
      for (int i = 1; i < ite_size; i++) {
        ParentProcess* edge = Edge( new_ite, "", itenerary[i-1] + 1, itenerary[i] + 1, 20, _nodes);
        if (id == 0)
          ((AbstractProperty*) edge->find_child("color/value"))->set_value (selected, true);
        else
          ((AbstractProperty*) edge->find_child("color/value"))->set_value (unselected, true);
      }
    }
    id++;
  }
  // set current to 0
  _curent_itenerary->set_value (0, true);

  //debug
  int itinerary_edges_size = dynamic_cast<IntProperty*> (_itinerary_edges->find_child ("size"))->get_value ();
  std::cerr << "in RosNode::test_multiple_itineraries " <<  _itinerary_edges  << " - " << itinerary_edges_size <<std::endl;

}
#ifndef NO_ROS
void 
RosNode::receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {


/*  for (auto item: ((djnn::List*)_itinerary_edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
      
    }*/

  int size = msg->nodes.size();

  for (int i = 0; i <  size - 1; ++i) {
      std::cout << "trying to draw arc between " << i << " and " << i+1 << std::endl;
      ParentProcess* edge = Edge(_itinerary_edges, "", std::stoi(msg->nodes[i]) + 1, std::stoi(msg->nodes[i+1]) + 1, 20, _nodes);
      ((AbstractProperty*)edge->find_child("color/r"))->set_value(30, true);
      ((AbstractProperty*)edge->find_child("color/g"))->set_value(144, true);
      ((AbstractProperty*)edge->find_child("color/b"))->set_value(255, true);
    } 
}

void 
RosNode::receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg) {
/*  for (auto item: ((djnn::List*)_itinerary_edges)->children()){
       item->deactivate ();

      if (item->get_parent ())

        item->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(item));

        item->schedule_delete ();

        item = nullptr;
      
    }*/

  int size = msg->nodes.size();

  for (int i = 0; i <  size - 1; ++i) {
      std::cout << "trying to draw arc between " << i << " and " << i+1 << std::endl;
      ParentProcess* edge = Edge(_itinerary_edges, "", std::stoi(msg->nodes[i])+1, std::stoi(msg->nodes[i+1])+1, 20, _nodes);
      ((AbstractProperty*)edge->find_child("color/r"))->set_value(30, true);
      ((AbstractProperty*)edge->find_child("color/g"))->set_value(144, true);
      ((AbstractProperty*)edge->find_child("color/b"))->set_value(255, true);
    }

}



//callback for robot_state msg (contains data on one robot)
void 
RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState::SharedPtr msg) {
  RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);
  get_exclusive_access(DBG_GET);

    if (msg->robot_id == 1){
      //Drone
      ((DoubleProperty*)_drone->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_drone->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_drone->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_drone->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_drone->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_drone->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_drone->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_drone->find_child("failsafe"))->set_value(msg->failsafe, true);
    }
    if (msg->robot_id == 2){
      //Agilex 1
      ((DoubleProperty*)_agilex1->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_agilex1->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_agilex1->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_agilex1->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_agilex1->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_agilex1->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_agilex1->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_agilex1->find_child("failsafe"))->set_value(msg->failsafe, true);
    
    }
    if (msg->robot_id == 3){
      //Agilex 2
      ((DoubleProperty*)_agilex2->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_agilex2->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_agilex2->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_agilex2->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_agilex2->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_agilex2->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_agilex2->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_agilex2->find_child("failsafe"))->set_value(msg->failsafe, true);
    
    }
    if (msg->robot_id == 4){
      //Lynx
      ((DoubleProperty*)_lynx->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_lynx->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_lynx->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_lynx->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_lynx->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_lynx->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_lynx->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_lynx->find_child("failsafe"))->set_value(msg->failsafe, true);
    
    }
    if (msg->robot_id == 5){
      //Spot
      ((DoubleProperty*)_spot->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_spot->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_spot->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_spot->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_spot->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_spot->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_spot->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_spot->find_child("failsafe"))->set_value(msg->failsafe, true);
    
    }
    if (msg->robot_id == 6){
      //VAB
      ((DoubleProperty*)_vab->find_child("lat"))->set_value(msg->position.latitude, true);
      ((DoubleProperty*)_vab->find_child("lon"))->set_value(msg->position.longitude, true);
      ((DoubleProperty*)_vab->find_child("altitude_msl"))->set_value(msg->position.altitude, true);
      ((DoubleProperty*)_vab->find_child("heading_rot"))->set_value(msg->compass_heading, true);
      ((IntProperty*)_vab->find_child("battery_percentage"))->set_value(msg->battery_percentage, true);
      ((IntProperty*)_vab->find_child("operation_mode"))->set_value(msg->operating_mode, true);
      ((BoolProperty*)_vab->find_child("emergency_stop"))->set_value(msg->emergency_stop, true);
      ((BoolProperty*)_vab->find_child("failsafe"))->set_value(msg->failsafe, true);
    
    }



  _latitude.set_value (msg -> position.latitude, true);
  _longitude.set_value (msg -> position.longitude, true);
  _robot_id.set_value (msg -> robot_id, true);
  _battery_percentage.set_value ( msg -> battery_percentage, true);
  _battery_voltage.set_value (msg -> battery_voltage, true);
  _compass_heading.set_value (msg -> compass_heading, true);
  _emergency_stop.set_value (msg -> emergency_stop, true);
  _failsafe.set_value (msg -> failsafe, true);
  _operation_mode.set_value (msg -> operating_mode, true);
  _altitude_msl.set_value (msg -> altitude_msl, true);
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);

  //Todo, use header.
}



void 
RosNode::receive_msg_trap (const icare_interfaces::msg::Trap msg){
//En attente de mise au points des échanges 
}

void 
RosNode::receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks){
//TODO
}
void 
RosNode::receive_msg_allocation(const icare_interfaces::msg::Allocation){
//TODO
}


void
RosNode::send_msg_planning_request(){
  std::cerr << "in send planning request" << std::endl;
  
  icare_interfaces::msg::PlanningRequest message = icare_interfaces::msg::PlanningRequest();
  std::cerr << _parent << std::endl;
  message.id = _current_plan_id_vab.get_string_value();
  int iid;

  for (auto item: ((djnn::List*)_nodes)->children()){
    std::cerr << "debug : " << ((TextProperty*)item->find_child("status"))->get_value() << std::endl;
    if (((TextProperty*)item->find_child("status"))->get_value() == "start"){
      std::cerr << "start" << std::endl;
      iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
      message.start_node = std::to_string(iid - 1);
    }
    if (((TextProperty*)item->find_child("status"))->get_value() == "end"){
      std::cerr << "end" << std::endl;
      iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
    
      message.end_node = std::to_string(iid - 1);
      
    }
  }

  message.header.stamp = _node->get_clock()->now();

  publisher_planning_request->publish(message);  
  GRAPH_EXEC;
  

}

void 
RosNode::send_msg_navgraph_update(){


  CoreProcess* nodes = _parent->find_child ("parent/l/map/layers/navgraph/nodes");
  CoreProcess* edges = _parent->find_child ("parent/l/map/layers/navgraph/edges");

  std::cerr << "about to generate json" << std::endl;
  nlohmann::json j;
  j["graph"]["directed"] = false;


  //Edges
  for (auto item: ((djnn::List*)edges)->children()){

    int ssource_id = dynamic_cast<IntProperty*> (item->find_child ("id_src"))->get_value ();
    int starget_id = dynamic_cast<IntProperty*> (item->find_child ("id_dest"))->get_value ();
    double dlength = dynamic_cast<DoubleProperty*> (item->find_child ("length"))->get_value ();

    //Rarccos(sin(a)sin(b) + cos(a)cos(b)cos(c-d)) a = lata, b=lona, c =latb, d =lonb
  /*  double lata = dynamic_cast<DoubleProperty*> (nodes[ssource_id].find_child("lat"))->get_value() * 3.14259/180;
    double lona = dynamic_cast<DoubleProperty*> (nodes[ssource_id].find_child("lon"))->get_value() * 3.14259/180;
    double latb = dynamic_cast<DoubleProperty*> (nodes[starget_id].find_child("lat"))->get_value() * 3.14259/180;
    double lonb = dynamic_cast<DoubleProperty*> (nodes[starget_id].find_child("lon"))->get_value() * 3.14259/180;
    double dlength = 6371000 * acos(sin(lata)*sin(lona) + cos(lata)*cos(lona)*cos(latb-lonb));
   */ nlohmann::json jn = {
      {"source", std::to_string(ssource_id - 1)},
      {"target", std::to_string(starget_id - 1)},
      {"metadata", { 
       // {"length", dlength}
      }}
    };                
    j["graph"]["edges"].push_back(jn); 
  }

  //Nodes
  for (auto item: ((djnn::List*)nodes)->children()){
    int iid = dynamic_cast<IntProperty*> (item->find_child ("id"))->get_value ();
    string slabel = dynamic_cast<TextProperty*> (item->find_child ("label"))->get_value ();
    double dlat = dynamic_cast<DoubleProperty*> (item->find_child ("wpt/lat"))->get_value ();
    double dlon = dynamic_cast<DoubleProperty*> (item->find_child ("wpt/lon"))->get_value ();
    double dalt = dynamic_cast<DoubleProperty*> (item->find_child ("alt"))->get_value ();
    string scompulsory = dynamic_cast<TextProperty*>(item->find_child("status"))->get_value();
    
    bool compulsory = (scompulsory == "compulsory");   

    

    nlohmann::json jn = {
      {"id", std::to_string(iid - 1)},
      {"label", slabel},
      {"metadata", { 
        {"altitude", dalt},
        {"latitude", dlat},
        {"longitude", dlon},
        {"compulsory", compulsory}
      }}
    };                
    j["graph"]["nodes"].push_back(jn);   
  }


  //TODO: remove - only for debug
  std::cerr << "finished generating JSON" << std::endl;
  icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
  message.data = j.dump();
  std::cerr << "about to publish on publisher_navgraph_update" << std::endl;

  message.header.stamp = _node->get_clock()->now();
  publisher_navgraph_update->publish(message);
  GRAPH_EXEC;
  std::cerr << "finished publishing" << std::endl;


}

void 
RosNode::send_validation_plan(){
 
  std::cerr << "in validation plan" << std::endl;
  std::cerr << _parent << std::endl;
   
    icare_interfaces::msg::StringStamped message = icare_interfaces::msg::StringStamped();
    message.data = std::to_string(_current_plan_id_vab.get_value());
    message.header.stamp = _node->get_clock()->now();
    publisher_validation->publish(message);
    GRAPH_EXEC;
  
}

void 
RosNode::send_selected_tasks(){
//TODO



//  message.header.stamp = _node->get_clock()->now();

}

void 
RosNode::send_validation_tasks(){
//TODO

//  message.header.stamp = _node->get_clock()->now();

}


#endif



void
RosNode::run () {
  #ifndef NO_ROS
  rclcpp::spin(_node);
  rclcpp::shutdown();
  #endif
}
