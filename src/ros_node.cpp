#include "ros_node.h"

#include "exec_env/global_mutex.h"
#include "core/control/binding.h"
#include "core/ontology/coupling.h"
#include "base/connector.h"
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

#include "model/LimaModel.h"
#include "model/PointModel.h"
#include "model/ExclusionZoneModel.h"
#include "model/NodeModel.h"
#include "model/EdgeModel.h"
#include "model/trap/TrapModel.h"
#include "model/trap/TrapDetectionModel.h"

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
  
  
  //PUBLISH
  /*
  sub_navgraph = _node->create_subscription<icare_interfaces::msg::StringStamped>("/navgraph", qos_transient, std::bind(&RosNode::receive_msg_navgraph, this, _1));
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

  // Itineraries
  CoreProcess *shortest, *safest, *tradeoff;
  GET_CHILD_VAR2 (shortest, CoreProcess, _model_manager, shortest_itinerary)
  GET_CHILD_VAR2 (safest, CoreProcess, _model_manager, safest_itinerary)
  GET_CHILD_VAR2 (tradeoff, CoreProcess, _model_manager, tradeoff_itinerary)
  _itineraries.push_back(shortest);
  _itineraries.push_back(safest);
  _itineraries.push_back(tradeoff);
  
  // Vehicles
  GET_CHILD_VAR2 (_vab, CoreProcess, _model_manager, vehicles/vab)
  GET_CHILD_VAR2 (_agilex1, CoreProcess, _model_manager, vehicles/agilex1)
  GET_CHILD_VAR2 (_agilex2, CoreProcess, _model_manager, vehicles/agilex2)
  GET_CHILD_VAR2 (_lynx, CoreProcess, _model_manager, vehicles/lynx)
  GET_CHILD_VAR2 (_spot, CoreProcess, _model_manager, vehicles/spot)
  GET_CHILD_VAR2 (_drone, CoreProcess, _model_manager, vehicles/m600)

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

 //sub_robot_state.reset ();
  /*
  sub_navgraph.reset ();
 
  sub_graph_itinerary_loop.reset ();
  sub_graph_itinerary_final.reset ();
  sub_candidate_tasks.reset ();
  sub_allocation.reset ();
  */

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



// **************************************************************************************************
//
//  SATELLITE
//
// **************************************************************************************************
void 
RosNode::receive_msg_robot_state(const icare_interfaces::msg::RobotState& msg_)
{  
  auto msg = &msg_;
  RCLCPP_INFO(_node->get_logger(), "I heard: '%f'  '%f'", msg->position.latitude, msg->position.longitude);

  djnn::Process * robots[] = {nullptr, _drone, _agilex1, _agilex2, _lynx, _spot, _vab, _drone_safety_pilot, _ground_safety_pilot};
  static const string robots_name[] = {"", "drone", "agilex1", "agilex2", "lynx", "spot", "vab", "drone_safety_pilot", "ground_safety_pilot"};
  if ((msg->robot_id < 1) || (msg->robot_id >= sizeof(robots))) {
    RCLCPP_INFO(_node->get_logger(), "incorrect robot_id: '%d'", msg->robot_id);
    return;
  }
    
  djnn::Process * robot = robots[msg->robot_id];
  const string& robot_name = robots_name[msg->robot_id];

  get_exclusive_access(DBG_GET);

  GET_CHILD_VALUE (timestamp, Text, _context, w_clock/state_text);

  SET_CHILD_VALUE (Double, robot, lat, msg->position.latitude, true);
  SET_CHILD_VALUE (Double, robot, lon, msg->position.longitude, true);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - Received robot_state for " + robot_name + "\n", true);
  
  if ((robot != _drone_safety_pilot) && (robot != _ground_safety_pilot)) {
    //SET_CHILD_VALUE (Double, robot, altitude_msl, msg->position.altitude, true);
    SET_CHILD_VALUE (Double, robot, heading_rot, msg->compass_heading, true);
    //SET_CHILD_VALUE (Int, robot, battery_percentage, msg->battery_percentage, true);
    //SET_CHILD_VALUE (Int, robot, operation_mode, msg->operating_mode, true); // FIXME: operation_mode vs operating_mode
    //SET_CHILD_VALUE (Bool, robot, emergency_stop, msg->emergency_stop, true);
    //SET_CHILD_VALUE (Bool, robot, failsafe, msg->failsafe, true);
  }

  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void RosNode::receive_msg_robot_config(const icare_interfaces::msg::RobotConfig& msg)
{  
  std::cout << "robot config"<< std::endl; 
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void RosNode::receive_msg_chat(const icare_interfaces::msg::ChatMessage& msg)
{  
  std::cout << "CHAT from:"<< msg.sender << " type:" << msg.type << " Text: " << msg.text << std::endl; 
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void RosNode::send_msg_chat() {
  std::cout << "send msg 'chat'"  << std::endl;
  // TODO
}

void RosNode::send_itinerary_request() {
  std::cout << "send itinerary request"  << std::endl;
  // TODO
  /*
  icare_interfaces::msg::PlanningRequest message = icare_interfaces::msg::PlanningRequest();
  message.id = _current_plan_id_vab.get_string_value();
  //cout << "send_msg_planning_request " << _current_plan_id_vab.get_string_value() << endl;

  for (auto item : ((djnn::List*)_node_models)->children())
  {
    GET_CHILD_VALUE (str_status, Text, item, status)
    GET_CHILD_VALUE (str_id, Text, item, id)

    if (str_status == "start")
      message.start_node = str_id;
    else if ( str_status == "end")
      message.end_node = str_id;
    else if (str_status == "forced")
        message.node_contraints.push_back(str_id);
  }

  GET_CHILD_VALUE (timestamp, Text, _context, w_clock/state_text);
  SET_CHILD_VALUE (Text, _fw_input, , timestamp + " - " + "Asked planification between nodes "+ message.start_node + " and " + message.end_node + " \n", true);

  message.header.stamp = _node->get_clock()->now();

  publisher_planning_request->publish(message);  
  */
}

void RosNode::receive_msg_itinerary(const icare_interfaces::msg::Itinerary& msg){  
  std::cout << "itinerary"<< std::endl; 
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}


void RosNode::send_group_config() {
  std::cout << "send group config"  << std::endl;
  // TODO
}


#endif

