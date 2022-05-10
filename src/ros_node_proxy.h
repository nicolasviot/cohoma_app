
#pragma once

//Djnn-smala stuff
#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/property/text_property.h"
#include "core/property/double_property.h"
#include "core/property/int_property.h"
#include "core/property/bool_property.h"
#include "core/property/ref_property.h"
#include "core/tree/component.h"
#include "exec_env/external_source.h"
#include "core/control/native_action.h"


//C++ stuff



using namespace djnn;

class RosNode;

class RosNodeProxy : public FatProcess
  {
  public:
    RosNodeProxy (ParentProcess* parent, const string& n, CoreProcess* map, CoreProcess* manager);
    ~RosNodeProxy() {}

    void impl_activate () override;
    void impl_deactivate () override;

    //void run () override;
  	
#if 0
    void receive_msg_navgraph (const icare_interfaces::msg::StringStamped::SharedPtr msg);
    void receive_msg_robot_state (const icare_interfaces::msg::RobotState::SharedPtr msg);
    void receive_msg_graph_itinerary_loop (const icare_interfaces::msg::GraphItineraryList::SharedPtr msg);
    void receive_msg_graph_itinerary_final (const icare_interfaces::msg::GraphItinerary::SharedPtr msg);
    void receive_msg_trap (const icare_interfaces::msg::TrapList msg);
    void receive_msg_allocated_tasks(const icare_interfaces::msg::Tasks);
    void receive_msg_allocation(const icare_interfaces::msg::Allocation);
    void receive_msg_site(const icare_interfaces::msg::Site);
#else
    void receive_msg_navgraph (const string& msg);
#endif



    void send_msg_planning_request();
    void send_msg_navgraph_update();
    void send_validation_plan();
    void send_selected_tasks();
    void send_validation_tasks();
    void send_msg_lima(int);
  
    void test_multiple_itineraries();
    void write_to_log (string, string);
 
   private:
    const std::string _topic_name;

    RosNode * _ros_node;
    friend class RosNode; // TODO: remove when refactor is finished

    //Arguments
	  CoreProcess* _map, *_manager;
	

    //navgraph fields
    TextProperty navgraph_data;
    CoreProcess *_nodes, *_edges, *_shadow_edges, *_traps;
    CoreProcess *_task_edges, *_task_areas, *_task_traps;
    CoreProcess *_exclusion_areas, *_limas; 
    Component *_itineraries_list;
    RefProperty *_ref_curent_itenerary;
    NativeAction* _edge_released_na;

    //robot
    CoreProcess* _vab, *_agilex1, *_agilex2, *_lynx, *_spot, *_drone, *_frame;

    RefProperty *_current_wpt, *_entered_wpt;
    TextProperty *_id_curent_itenerary;
    
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
    
  };
