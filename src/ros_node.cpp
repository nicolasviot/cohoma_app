#include "ros_node.h"

#include "exec_env/global_mutex.h"
#include "core/control/binding.h"
#include "core/ontology/coupling.h"
#include "base/connector.h"
#include "base/process_handler.h"
#include "core/core-dev.h"
#include "core/tree/list.h"
#include "gui/shape/poly.h"
#include "gui/widgets/multiline_edit.h"
#include "core/utils/getset.h"

#include <nlohmann/json.hpp>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <iterator>
#include "math.h"
#include <vector>

#include "model/LimaModel.h"
#include "model/PointModel.h"
#include "model/ExclusionZoneModel.h"
#include "model/NodeModel.h"
#include "model/EdgeModel.h"
#include "model/TrapModel.h"

#include "model/task/TaskEdgeModel.h"
#include "model/task/TaskAreaModel.h"
#include "model/task/TaskTrapModel.h"

using std::placeholders::_1;

using namespace djnn;
using namespace std;


RosNode::RosNode (CoreProcess* parent, const string& n, CoreProcess* my_map, CoreProcess* context, CoreProcess* model_manager) :
  FatProcess (n),
  ExternalSource (n),
  //arguments
  _map (my_map),
  _context (context),
  _model_manager (model_manager),

  // Planif VAB
  _current_plan_id_vab (this, "current_plan_id", 0),
  _start_plan_vab_id (this, "start_plan_id", 0),
  _end_plan_vab_id (this, "end_plan_id", 0)

    #ifndef NO_ROS
    //ROS
  ,qos_best_effort(10),
  qos(1),
  qos_transient(1)
    #endif
{

  #ifndef NO_ROS
  _node = std::make_shared<rclcpp::Node>("IHM_Tactique");
  // reliable ~= TCP connections => for the navgraph msgs
  // best_effort allows to drop some pacquets => for robot_state msgs

  
  qos.reliable();
  qos.durability_volatile();
  qos_best_effort.best_effort();
  qos_best_effort.durability_volatile();
  qos_transient.reliable();
  qos_transient.transient_local();
  #endif

  finalize_construction (parent, n);
}


void
RosNode::impl_activate ()
{ 
  #ifndef NO_ROS

  //nav Graph (TO TEST)
  sub_navgraph = _node->create_subscription<icare_interfaces::msg::StringStamped>("/navgraph_manager/navgraph", qos, std::bind(&RosNode::receive_msg_navgraph, this, _1));

  // Robots
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>("/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, this, _1));
  sub_robot_config = _node->create_subscription<icare_interfaces::msg::RobotConfig>("/robot_config", qos_best_effort, std::bind(&RosNode::receive_msg_robot_config, this, _1));

  //CHAT
  sub_chat = _node->create_subscription<icare_interfaces::msg::ChatMessage>("/chat", qos, std::bind(&RosNode::receive_msg_chat, this, _1));
  publisher_chat =_node->create_publisher<icare_interfaces::msg::ChatMessage>("/chat", qos);
  
  //Itinerary
  sub_itinerary = _node->create_subscription<icare_interfaces::msg::Itinerary>("/navgraph_manager/itinerary", qos, std::bind(&RosNode::receive_msg_itinerary, this, _1));
  publisher_itinerary_request =_node->create_publisher<icare_interfaces::msg::ItineraryRequest>("/navgraph_manager/itinerary_request", qos);

  //group config
  publisher_group_config =_node->create_publisher<icare_interfaces::msg::GroupConfig>("/group_config", qos);
  
  //Traps
  sub_trap_detection = _node->create_subscription<icare_interfaces::msg::TrapDetection>("/trap__manager/detection", qos, std::bind(&RosNode::receive_msg_trap_detection, this, _1));
  sub_trap_update = _node->create_subscription<icare_interfaces::msg::Trap>("/trap_manager/trap_update", qos, std::bind(&RosNode::receive_msg_trap_update, this, _1));;
  
  publisher_trap_activation =_node->create_publisher<icare_interfaces::msg::TrapActivation>("/trap_manager/activate_trap", qos);
  publisher_trap_delete =_node->create_publisher<icare_interfaces::msg::Trap>("/trap_manager/del_trap", qos);
  publisher_trap_add =_node->create_publisher<icare_interfaces::msg::Trap>("/trap_manager/add_trap", qos);
  publisher_trap_set_identification_mode = _node->create_publisher<icare_interfaces::msg::TrapSetIdentificationMode>("/trap_manager/set_identification_mode", qos);
  publisher_trap_set_confirmation_mode =_node->create_publisher<icare_interfaces::msg::TrapSetIdentificationMode>("/trap_manager/set_confirmation_mode", qos);
  publisher_trap_set_deactivation_action =_node->create_publisher<icare_interfaces::msg::TrapSetDeactivationAction>("/trap_manager/set_deactivation_action", qos);
  publisher_trap_set_clustering_distance =_node->create_publisher<std_msgs::msg::Float32>("/trap_manager/set_clustering_distance", qos);
  
  //PUBLISH
  /*
  sub_robot_state = _node->create_subscription<icare_interfaces::msg::RobotState>("/robot_state", qos_best_effort, std::bind(&RosNode::receive_msg_robot_state, this, _1));
  sub_graph_itinerary_loop = _node->create_subscription<icare_interfaces::msg::GraphItineraryList>("/itinerary", qos, std::bind(&RosNode::receive_msg_graph_itinerary_loop, this, _1));
  sub_graph_itinerary_final = _node->create_subscription<icare_interfaces::msg::GraphItinerary>("/plan", qos, std::bind(&RosNode::receive_msg_graph_itinerary_final, this, _1));
  sub_candidate_tasks = _node->create_subscription<icare_interfaces::msg::Tasks>("/candidate_tasks", qos, std::bind(&RosNode::receive_msg_candidate_tasks, this, _1));
  sub_allocation = _node->create_subscription<icare_interfaces::msg::Allocation>("/allocation", qos, std::bind(&RosNode::receive_msg_allocation, this, _1));
  sub_traps = _node->create_subscription<icare_interfaces::msg::TrapList>("/traps", qos_transient, std::bind(&RosNode::receive_msg_trap, this, _1));
  sub_site = _node->create_subscription<icare_interfaces::msg::Site>("/site", qos_transient, std::bind(&RosNode::receive_msg_site, this, _1));
  sub_map = _node->create_subscription<icare_interfaces::msg::EnvironmentMap>("/map", qos_transient, std::bind(&RosNode::receive_msg_map, this, _1));

  // PUBLISH
  //publisher_planning_request =_node->create_publisher<icare_interfaces::msg::PlanningRequest>("/planning_request", qos);
  publisher_validation = _node->create_publisher<icare_interfaces::msg::StringStamped>("/validation", qos);
  publisher_navgraph_update = _node->create_publisher<icare_interfaces::msg::StringStamped>("/navgraph_update", qos_transient);
  publisher_tasks = _node->create_publisher<icare_interfaces::msg::Tasks>("/tasks", qos);
  publisher_validation_tasks = _node->create_publisher<icare_interfaces::msg::StringStamped>("/validate", qos);
  publisher_lima = _node->create_publisher<icare_interfaces::msg::LimaCrossed>("/lima", qos);
  publisher_trap_activation = _node->create_publisher<icare_interfaces::msg::TrapActivation>("/activation", qos);
  
  */
  #endif

  // Activate
  _current_plan_id_vab.activate();
  _start_plan_vab_id.activate();
  _end_plan_vab_id.activate();

  GET_CHILD_VAR2 (_frame, CoreProcess, _parent, parent/f)


  // ---------------------------
  // MODEL

  GET_CHILD_VAR2 (_layer_models, CoreProcess, _model_manager, layers)

  // SITE
  GET_CHILD_VAR2 (_limit_models, CoreProcess, _model_manager, limits)
  GET_CHILD_VAR2 (_zone_models, CoreProcess, _model_manager, zones)
  GET_CHILD_VAR2 (_lima_models, CoreProcess, _model_manager, limas)

  GET_CHILD_VAR2 (_node_ids, CoreProcess, _model_manager, node_ids)
  GET_CHILD_VAR2 (_node_models, CoreProcess, _model_manager, nodes)
  GET_CHILD_VAR2 (_edge_ids, CoreProcess, _model_manager, edge_ids)
  GET_CHILD_VAR2 (_edge_models, CoreProcess, _model_manager, edges)

  //OPERATOR
  GET_CHILD_VAR2 (_ot, CoreProcess, _model_manager, operators/ot)
  GET_CHILD_VAR2 (_og1, CoreProcess, _model_manager, operators/og1)
  GET_CHILD_VAR2 (_og2, CoreProcess, _model_manager, operators/og2)
  GET_CHILD_VAR2 (_og3, CoreProcess, _model_manager, operators/og3)
  
  // Itineraries
  CoreProcess *shortest, *safest, *tradeoff;
  GET_CHILD_VAR2 (shortest, CoreProcess, _model_manager, shortest_itinerary)
  GET_CHILD_VAR2 (safest, CoreProcess, _model_manager, safest_itinerary)
  GET_CHILD_VAR2 (tradeoff, CoreProcess, _model_manager, tradeoff_itinerary)
  _itineraries.push_back(shortest);
  _itineraries.push_back(safest);
  _itineraries.push_back(tradeoff);
  
  // Vehicles 

  //OT
  GET_CHILD_VAR2 (_vab, CoreProcess, _model_manager, vehicles/vab)

  //OG1
  GET_CHILD_VAR2 (_bnx8, CoreProcess, _model_manager, vehicles/bnx8)
  GET_CHILD_VAR2 (_agilex2, CoreProcess, _model_manager, vehicles/agilex2)
  GET_CHILD_VAR2 (_agilex1, CoreProcess, _model_manager, vehicles/agilex1)
  GET_CHILD_VAR2 (_lynx, CoreProcess, _model_manager, vehicles/lynx)
  
  //OG2
  GET_CHILD_VAR2 (_agilex3, CoreProcess, _model_manager, vehicles/agilex3)
  GET_CHILD_VAR2 (_minnie, CoreProcess, _model_manager, vehicles/minnie)
  GET_CHILD_VAR2 (_m600, CoreProcess, _model_manager, vehicles/m600)

  //OG3
  GET_CHILD_VAR2 (_long_eye, CoreProcess, _model_manager, vehicles/long_eye)
  GET_CHILD_VAR2 (_pprz, CoreProcess, _model_manager, vehicles/pprz)
  GET_CHILD_VAR2 (_spot, CoreProcess, _model_manager, vehicles/spot)

  // Safety pilots
  GET_CHILD_VAR2 (_drone_safety_pilot, CoreProcess, _model_manager, safety_pilots/drone_safety_pilot)
  GET_CHILD_VAR2 (_ground_safety_pilot, CoreProcess, _model_manager, safety_pilots/ground_safety_pilot)

  // Context
  GET_CHILD_VAR2 (_ref_NULL, RefProperty, _context, REF_NULL)
  GET_CHILD_VAR2 (_ref_node_graph_edition, RefProperty, _context, ref_node_graph_edition)
  GET_CHILD_VAR2 (_ref_node_status_edition, RefProperty, _context, ref_node_status_edition)
  GET_CHILD_VAR2 (_ref_current_trap, RefProperty, _context, ref_current_trap)
  GET_CHILD_VAR2 (_selected_itinerary_id, TextProperty, _context, selected_itinerary_id)

  // TASKS
  GET_CHILD_VAR2 (_task_area_models, CoreProcess, _model_manager, task_areas)
  GET_CHILD_VAR2 (_task_edge_models, CoreProcess, _model_manager, task_edges)
  GET_CHILD_VAR2 (_task_trap_models, CoreProcess, _model_manager, task_traps)

  // TRAPS
  GET_CHILD_VAR2 (_trap_models, CoreProcess, _model_manager, traps)


  // ---------------------------
  // VIEW
  GET_CHILD_VAR2 (_fw_input, CoreProcess, _parent, parent/fw/input)
  GET_CHILD_VAR2 (_fw_console_input, CoreProcess, _parent, parent/fw_console/input)
  GET_CHILD_VAR2 (_console, CoreProcess, _parent, parent/right_panel/layer/console)

  GET_CHILD_VAR2 (_result_layer, CoreProcess, _parent, parent/map/layers/visibility_map)
  GET_CHILD_VAR2 (_image, DataImage, _parent, parent/map/layers/visibility_map/ui/image)


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

  sub_robot_state.reset();
  sub_robot_config.reset();
  sub_chat.reset();
  sub_itinerary.reset();
  //Traps
  sub_trap_detection.reset();
  sub_trap_update.reset();
 
#endif  
  // De-activate
  _current_plan_id_vab.deactivate();
  _start_plan_vab_id.deactivate();
  _end_plan_vab_id.deactivate();
  ExternalSource::please_stop ();
}


void
RosNode::run () {
#ifndef NO_ROS
  rclcpp::spin(_node);
  rclcpp::shutdown();
#endif
}


#ifndef NO_ROS

// **************************************************************************************************
//
//  Navigation Graph
//
// **************************************************************************************************

// Receive msg "Navigation Graph"
// Receive msg "Navigation Graph"
void 
RosNode::receive_msg_navgraph (const icare_interfaces::msg::StringStamped& _msg)
{
  auto msg = &_msg;
  get_exclusive_access(DBG_GET);

  cout << "Receive msg Navigation Graph" << endl;

  //GET_CHILD_VALUE (timestamp, Text, _context, w_clock/state_text)
  //SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Received new navgraph\n", true)
  
  // Reset "nodes" in case it contains a pointer on a node that will be removed
  _ref_node_graph_edition->set_value (_ref_NULL, true);
  _ref_node_status_edition->set_value (_ref_NULL, true);

  // FIXME TODO: schedule_deletion old tasks about edges, about traps and about zones
  // schedule delete old itineraries
  // schedule delete old edges
  // schedule delete old nodes

  nlohmann::json j = nlohmann::json::parse(msg->data);
  nlohmann::json j_graph;
  if (j.contains("graph")) {
    j_graph = j["graph"];
  }
  else if (j.contains("graphs"))
  {
    if (j.size() > 1)
      j_graph = j["graphs"][0];
    else if (j.size() == 0)
      return;
  }

  // NODES
  for (auto& j_node : j_graph["nodes"])
  {
    const string& node_id = j_node["id"];
    const string& label = j_node["label"];
    
    auto& m = j_node["metadata"];
    int phase = m["phase"].get<int>();
    double latitude = m["latitude"].get<double>();
    double longitude = m["longitude"].get<double>();
    double altitude = m["altitude"].get<double>();
    bool mandatory = m["compulsory"].get<bool>();


    Process* node = _node_models->find_child_impl(node_id);
    if (node == nullptr)
    {
      // We need a pointer on the TextProperty (else memory pb)
      TextProperty* tmp = new TextProperty (_node_ids, "", node_id);
      //cout << "String _ (\"_" << node_id << "\")" << endl;

      NodeModel (_node_models, node_id, node_id, phase, label, latitude, longitude, altitude, mandatory);
      //cout << "NodeModel _" << node_id << " (\"" + node_id << "\", " << to_string(phase) << ", \"" << label << "\", " << latitude << ", " << longitude << ", " << altitude << ", " << mandatory << ")" << endl;
    }
    //else
    //  cout << "Model of node " << node_id << " already exist. Need to update it ?" << endl;
  }

  // EDGES
  for (auto& j_edge : j_graph["edges"])
  {
    //const string& str_source = j_edge["source"].get<string>();
    //const string& str_target = j_edge["target"].get<string>();
    const string& str_source = j_edge["source"];
    const string& str_target = j_edge["target"];

    string edge_id = str_source + '_' + str_target;

    auto& m = j_edge["metadata"];
    double length = m["length"].get<double>();

    Process* edge = _edge_models->find_child_impl(edge_id);
    if (edge == nullptr)
    {
      // We need a pointer on the TextProperty (else memory pb)
      TextProperty* tmp = new TextProperty (_edge_ids, "", edge_id);
      //cout << "String _ (\"_" << str_source << "__" << str_target << "\")" << endl;

     Process* source = _node_models->find_child (str_source);
      Process* target = _node_models->find_child (str_target);

      EdgeModel (_edge_models, edge_id, source, target, length);
      //cout << "EdgeModel _" << str_source << "__" << str_target << " (find(this.nodes, \"_" << str_source << "\"), find(this.nodes, \"_" << str_target << "\"), " << length << ")" << endl;
    }  
    //else
    //  cout << "Model of edge " << edge_id << " already exist. Need to update it ?" << endl;
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

// **************************************************************************************************
//
//  SATELLITE
//
// **************************************************************************************************
void 
RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState& msg)
{  
  //TODO make it static maybe
  //list in an array ordered by vehicles ID
  djnn::Process * robots[] = {nullptr, _m600, _bnx8, _agilex1, _agilex2, _agilex3, _lynx, _minnie ,_spot, _long_eye, _pprz, _vab};

  if ((msg.robot_id < 1) || (msg.robot_id >= sizeof(robots))) {
    RCLCPP_INFO(_node->get_logger(), "incorrect robot_id: '%d'", msg.robot_id);
    return;
  }
 
  djnn::Process * robot = robots[msg.robot_id];

  get_exclusive_access(DBG_GET);
  GET_CHILD_VALUE (robot_name, Text, robot, title);
  GET_CHILD_VALUE (timestamp, Text, _context, w_clock/state_text);
  SET_CHILD_VALUE (Double, robot, lat, msg.position.latitude, true);
  SET_CHILD_VALUE (Double, robot, lon, msg.position.longitude, true);
  //SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Received robot_state for " + robot_name + "\n", true);
  SET_CHILD_VALUE (Double, robot, heading_rot, msg.compass_heading, true);

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void RosNode::receive_msg_robot_config(const icare_interfaces::msg::RobotConfig& msg)
{  
  djnn::Process * robots[] = {nullptr, _m600, _bnx8, _agilex1, _agilex2, _agilex3, _lynx, _minnie ,_spot, _long_eye, _pprz, _vab};

  if ((msg.robot_id < 1) || (msg.robot_id >= sizeof(robots))) {
    RCLCPP_INFO(_node->get_logger(), "incorrect robot_id: '%d'", msg.robot_id);
    return;
  }
 
  djnn::Process * robot = robots[msg.robot_id];
  get_exclusive_access(DBG_GET);
  SET_CHILD_VALUE (Bool, robot, available, msg.available, true);
  SET_CHILD_VALUE (Bool, robot, teleoperated, msg.teleoperated, true);
  SET_CHILD_VALUE (Bool, robot, contact, msg.contact, true);
  SET_CHILD_VALUE (Bool, robot, detection, msg.detection, true);
  SET_CHILD_VALUE (Bool, robot, laser, msg.laser, true);
  SET_CHILD_VALUE (Bool, robot, identification, msg.identification, true);
  
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void RosNode::receive_msg_chat(const icare_interfaces::msg::ChatMessage& msg)
{  
  std::cout << "CHAT from:"<< msg.sender << " type:" << msg.type << " Text: " << msg.text << std::endl; 
  GRAPH_EXEC;
}

void RosNode::send_msg_chat(string _text, int _type, double _lat, double _lng, double _alt) {
  //std::cout << "send msg 'chat'"  << std::endl;
  icare_interfaces::msg::ChatMessage message = icare_interfaces::msg::ChatMessage();
  message.sender = message.SENDER_OT;
  message.text = _text;
  message.type = _type;

  geographic_msgs::msg::GeoPoint point = geographic_msgs::msg::GeoPoint();
  point.latitude = _lat;
  point.longitude = _lng;
  point.altitude = _alt;
  message.localisation = point;

  message.stamp = _node->get_clock()->now();
  publisher_chat->publish(message);  
}

void RosNode::send_msg_itinerary_request() {
  //std::cout << "send itinerary request"  << std::endl;

  icare_interfaces::msg::ItineraryRequest message = icare_interfaces::msg::ItineraryRequest();
  message.id = _current_plan_id_vab.get_string_value();
  cout << "send_msg_planning_request " << _current_plan_id_vab.get_string_value() << endl;

  for (auto item : ((djnn::List*)_node_models)->children())
  {
    GET_CHILD_VALUE (str_status, Text, item, status)
    GET_CHILD_VALUE (str_id, Text, item, id)

    if (str_status == "start")
      message.start_node = str_id;
    else if ( str_status == "end")
      message.end_node = str_id;
  }

  //GET_CHILD_VALUE (timestamp, Text, _context, w_clock/state_text);
  //SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Asked planification between nodes "+ message.start_node + " and " + message.end_node + " \n", true);

  message.header.stamp = _node->get_clock()->now();
  publisher_itinerary_request->publish(message);  
}

void RosNode::receive_msg_itinerary(const icare_interfaces::msg::Itinerary& msg){  
  std::cout << "TODO received itinerary"<< std::endl; 
  GRAPH_EXEC;
}

///GROUP CONFIG
void RosNode::send_msg_group_config() {
  icare_interfaces::msg::GroupConfig message = icare_interfaces::msg::GroupConfig();
  
  vector<CoreProcess*> robots_1 = dynamic_cast<ProcessCollector*>(_og1->find_child("robots"))->get_list();
  vector<CoreProcess*> robots_2 = dynamic_cast<ProcessCollector*>(_og2->find_child("robots"))->get_list();
  vector<CoreProcess*> robots_3 = dynamic_cast<ProcessCollector*>(_og3->find_child("robots"))->get_list();

  vector<unsigned char> group_1 ; 
  vector<unsigned char> group_2 ; 
  vector<unsigned char> group_3 ; 


  for( int i = 0; i < robots_1.size(); i++){
      group_1.push_back(static_cast<IntProperty*>(robots_1[i]->find_child("uid"))->get_value());
  }

  for( int i = 0; i < robots_2.size(); i++){
      group_2.push_back(static_cast<IntProperty*>(robots_2[i]->find_child("uid"))->get_value());
  }

  for( int i = 0; i < robots_3.size(); i++){
      group_3.push_back(static_cast<IntProperty*>(robots_3[i]->find_child("uid"))->get_value());
  }

  std::cout << "doing it" << std::endl;
  message.group_1 = group_1;
  message.group_2 = group_2;
  message.group_3 = group_3;

  message.stamp = _node->get_clock()->now();
  publisher_group_config->publish(message);
}

//////////////////////////:
// TRAPS///////////////////
void RosNode::send_msg_trap_activation(int uid , bool is_active) {
  std::cout << "TODO send msg trap activation"  << std::endl;
  // TODO
}

void RosNode::send_msg_trap_delete(int uid) {
  //std::cout << "send trap deleted"  << uid <<std::endl;
  icare_interfaces::msg::Trap message = icare_interfaces::msg::Trap();
  message.id = uid;
  publisher_trap_delete->publish(message);  
}

void RosNode::receive_msg_trap_detection(const icare_interfaces::msg::TrapDetection& msg) {
  std::cout << "TODO send trap detection"  << std::endl;
  // TODO
}

void RosNode::receive_msg_trap_update(const icare_interfaces::msg::Trap& msg) {
  std::cout << "TODO send trap update"  << std::endl;
  // TODO
}

void RosNode::send_msg_trap_add(int uit, double _lat, double _lng) {
  std::cout << "TODO send trap add"  << std::endl;
  // TODO
}

void RosNode::send_msg_trap_set_identification_mode(int uid, int ident_mode) {
  std::cout << "TODO send trap identification mode"  << std::endl;
  // TODO
}

void RosNode::send_msg_trap_set_confirmation_mode(int uid, int confirmation_mode){
  std::cout << "TODO send trap confirmation mode"  << std::endl;
  // TODO
}

void RosNode::send_msg_trap_set_deactivation_action(int uid, int action) {
  std::cout << "TODO send trap deactivation action"  << std::endl;
  // TODO
}

void RosNode::send_msg_trap_set_clustering_distance(double clustering_distance) {
  std::cout << "TODO send trap clustering distance"  << std::endl;
  // TODO
}

#endif