/*
 *	COHOMA (Collaboration Homme Machine) application
 *
 *	The copyright holders for the contents of this file are:
 *	Ecole Nationale de l'Aviation Civile, France (2021-2023)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *	Contributors:
 *      Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */

use core
use gui
use base

import SubLayerModel
import team.OperatorModel
import team.SafetyPilotModel
import team.VehicleModel
import PointModel
import ExclusionZoneModel
import LimaModel
import NodeModel
import EdgeModel
import TrapModel
import ItineraryModel

_native_code_
%{
    #include <iostream>
    using namespace std;

%}

// FIXME: Failed to do it directly in smala
/*_action_
action_clear_itineraries (Process src, Process self)
{
    print ("Clear itineraries...\n")

    for itinerary : self.itineraries
    //for (int i = 1; i <= 3; i++)
    {
        //itinerary = find (self.itineraries, i)
        //int size = 
        for (int j = 1; j <= itinerary.node_ids.size; j++)
        {
            print ("delete " + itinerary.node_ids.[j] + "\n")
            //delete itinerary.node_ids.[j]
            node_id = find (itinerary.node_ids, j)
            remove node_id from itinerary.node_ids
            delete node_id
        }
    }
}*/

_action_
action_clear_itineraries (Process c)
%{
    Process* self = (Process*) get_native_user_data(c);

    cout << "Clear itineraries..." << endl;

    for (int i = 1; i <= 3; i++)
    {
        Process* itinerary = self->find_child("itineraries/" + to_string(i));
        if (itinerary != nullptr)
        {
            string type = static_cast<TextProperty*>(itinerary->find_child("type"))->get_value();
            Container* node_ids = dynamic_cast<Container*>(itinerary->find_child("node_ids"));
            if (node_ids != nullptr)
            {
                int nodes_size = dynamic_cast<IntProperty*>(node_ids->find_child("size"))->get_value();
                if (nodes_size > 0)
                {
                    cout << i << ": itinerary '" << type << "'' with " << nodes_size << " nodes. Clean up content..." << endl;
                
                    // FIXME: node_ids.$removed is not activated
                    //node_ids->clean_up_content();

                    vector <Process*> tmp;
                    for (Process* node_id : node_ids->children()) {
                        tmp.push_back(node_id);
                    }
                    for (Process* node_id : tmp)
                    {
                        node_ids->remove_child(node_id);
                        node_id->schedule_delete();
                    }
                }
            }
        }
    }
%}


_action_
action_clear_tasks (Process c)
%{
    Process* self = (Process*) get_native_user_data(c);

    cout << "Clear models of tasks..." << endl;

    // AREAS (ZONES)
    Container* task_areas = dynamic_cast<Container*>(self->find_child("task_areas"));
    if (task_areas != nullptr)
    {
        int tasks_size = dynamic_cast<IntProperty*>(task_areas->find_child("size"))->get_value();
        if (tasks_size > 0)
        {
            cout << "Delete " << tasks_size << " tasks about an AREA. Clean up content..." << endl;
        
            // Create a local temporary vector to iterate on it while removing child from task_areas
            vector <Process*> tmp;
            for (Process* task : task_areas->children()) {
                tmp.push_back(task);
            }
            for (Process* task : tmp)
            {
                task_areas->remove_child(task);
                task->schedule_delete();
            }
        }
    }

    // EDGES
    Container* task_edges = dynamic_cast<Container*>(self->find_child("task_edges"));
    if (task_edges != nullptr)
    {
        int tasks_size = dynamic_cast<IntProperty*>(task_edges->find_child("size"))->get_value();
        if (tasks_size > 0)
        {
            cout << "Delete " << tasks_size << " tasks about an EDGE. Clean up content..." << endl;
        
            // Create a local temporary vector to iterate on it while removing child from task_edges
            vector <Process*> tmp;
            for (Process* task : task_edges->children()) {
                tmp.push_back(task);
            }
            for (Process* task : tmp)
            {
                task_edges->remove_child(task);
                task->schedule_delete();
            }
        }
    }

    // TRAPS
    Container* task_traps = dynamic_cast<Container*>(self->find_child("task_traps"));
    if (task_traps != nullptr)
    {
        int tasks_size = dynamic_cast<IntProperty*>(task_traps->find_child("size"))->get_value();
        if (tasks_size > 0)
        {
            cout << "Delete " << tasks_size << " tasks about a TRAP. Clean up content..." << endl;
        
            // Create a local temporary vector to iterate on it while removing child from task_traps
            vector <Process*> tmp;
            for (Process* task : task_traps->children()) {
                tmp.push_back(task);
            }
            for (Process* task : tmp)
            {
                task_traps->remove_child(task);
                task->schedule_delete();
            }
        }
    }
%}



_define_
ModelManager (Process _context, int _is_debug)
{
    context aka _context

    Bool IS_DEBUG (_is_debug)

    //print ("Model Manager\n")

	Spike clear_itineraries
    Spike clear_tasks

    Spike itineraries_updated


    // sub Layers
    List layers {
        SubLayerModel _ ("Map")
        SubLayerModel _ ("Visibility map") // ("Result")
        SubLayerModel _ ("Site")
        SubLayerModel _ ("Navig. Graph")
        SubLayerModel _ ("Itineraries")
        SubLayerModel _ ("Traps")
        SubLayerModel _ ("Tasks")
        SubLayerModel _ ("Allocations")
        SubLayerModel _ ("Safety pilots")
        SubLayerModel _ ("Vehicles") // ("Satellites")

        //SubLayerModel _ ("Geoportail")
        //SubLayerModel _ ("OSM")
    }


    // **************************************************************************************************
    //
    //  OPERATORS
    //
    // **************************************************************************************************
    Int OP_TYPE_TACTICAL (1)
    Int OP_TYPE_ROBOT (2)
    Int OP_TYPE_SAFETY (2)

    Component operators {
        OperatorModel ot (_context, 1, $OP_TYPE_TACTICAL, "OT", "Op. Tactique", "Charles", $_context.OT_COLOR)
        OperatorModel og1 (_context, 2, $OP_TYPE_ROBOT, "OG1", "Op. Système 1", "Christophe", $_context.OG1_COLOR)
        OperatorModel og2 (_context, 3, $OP_TYPE_ROBOT, "OG2", "Op. Système 2", "Titouan", $_context.OG2_COLOR)
        OperatorModel og3 (_context, 4, $OP_TYPE_ROBOT, "OG3", "Op. Système 3", "Baptiste", $_context.OG3_COLOR)
    }


    // **************************************************************************************************
    //
    //  VEHICLES
    //
    // **************************************************************************************************
    Component vehicles {
        VehicleModel vab (_context, "vab", "VAB", $_context.init_lat, $_context.init_lon, $_context.VAB_COL)
        VehicleModel agilex1 (_context, "agilex1", "AGILEX 1", $_context.init_lat + 0.0005, $_context.init_lon, $_context.AGI_1_COL)
        VehicleModel agilex2 (_context, "agilex2", "AGILEX 2", $_context.init_lat + 0.001, $_context.init_lon, $_context.AGI_2_COL)
        VehicleModel lynx (_context, "lynx", "LYNX", $_context.init_lat, $_context.init_lon + 0.001, $_context.LYNX_COL)
        VehicleModel spot (_context, "spot", "SPOT", $_context.init_lat + 0.001 , $_context.init_lon + 0.001, $_context.SPOT_COL)
        VehicleModel drone (_context, "drone", "DRONE", $_context.init_lat + 0.0015 , $_context.init_lon + 0.0015, $_context.DRONE_COL)
    }


    // **************************************************************************************************
    //
    //  SAFETY PILOTS
    //
    // **************************************************************************************************
    Component safety_pilots {
        // Unmanned Aerial Vehicle
        //SafetyPilotModel uav (_context, "uav", "UAV", $_context.init_lat, $_context.init_lon - 0.005, $_context.UAV_COL)
        SafetyPilotModel drone_safety_pilot (_context, "uav", "UAV", $_context.init_lat, $_context.init_lon - 0.005, $_context.UAV_COL)

        // Unmanned Ground Vehicle
        //SafetyPilotModel ugv (_context, "ugv", "UGV", $_context.init_lat, $_context.init_lon + 0.005, $_context.UGV_COL)
        SafetyPilotModel ground_safety_pilot (_context, "ugv", "UGV", $_context.init_lat, $_context.init_lon + 0.005, $_context.UGV_COL)
    }


    // **************************************************************************************************
    //
    //  SITE
    //
    // **************************************************************************************************
    // LIMITS (list of points)
    List limits

    // EXCLUSION ZONES
    List zones

    // LIMAS
    List limas


    // **************************************************************************************************
    //
    //  NODES
    //
    // **************************************************************************************************
    List node_ids

    // Don't use index but node id instead
    //List nodes

    // Get a node with its id
    Component nodes
    
    // Model of the temporary node during edition 
    NodeModel temp_node ("temp", -1, "", 0.0, 0.0, 0, 0)

    Spike create_node_from_temp

    create_node_from_temp -> na_create_node_from_temp:(this) {
        string s_node_id = toString (this.node_ids.size)
        
        NodeModel (this.nodes, s_node_id, s_node_id, -1, "by operator", $this.temp_node.lat, $this.temp_node.lon, 0, 0)

        addChildrenTo this.node_ids {
            String _ (s_node_id)
        }
    }


    // **************************************************************************************************
    //
    //  EDGES
    //
    // **************************************************************************************************
    List edge_ids

    // Don't use index but edge id instead
    //List edges

    // Get a edge with its id
    Component edges

    // Temporary edge during graph edition
    //EdgeModel temp_edge (this.nodes[..], this.nodes[..], 0.0)


    // **************************************************************************************************
    //
    //  ITINERARIES
    //
    // **************************************************************************************************
    List itineraries {
        ItineraryModel _ (_context, "shortest")
        ItineraryModel _ (_context, "safest")
        ItineraryModel _ (_context, "tradeoff")
    }
    // FIXME: use aka
    shortest_itinerary aka itineraries.[1]
    safest_itinerary aka itineraries.[2]
    tradeoff_itinerary aka itineraries.[3]

    NativeAction na_clear_itineraries (action_clear_itineraries, this, 1)
    clear_itineraries -> na_clear_itineraries


    // **************************************************************************************************
    //
    //  TRAPS
    //
    // **************************************************************************************************
    List traps


    // **************************************************************************************************
    //
    //  TASKS
    //
    // **************************************************************************************************
    //List task_edge_ids

    // Don't use list but edge id instead
    List task_edges

    // Get a task edge with its edge id
    //Component task_edges

    List task_areas
    List task_traps

    NativeAction na_clear_tasks (action_clear_tasks, this, 1)
    clear_tasks -> na_clear_tasks
    
}