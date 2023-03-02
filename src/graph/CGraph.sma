use core
use gui
use base

import Node
import model.EdgeModel

_define_
CGraph (Process root, Process f, Process model_manager, Process map, Process context)
{
    // ----------------------------------------------------
  // FSM to manage the addition of node in the graph
  FSM fsm_add_node {
    State idle 

    State preview {

      Translation pos (0, 0)
      context.map_translation_x =:> pos.tx
      context.map_translation_y =:> pos.ty

      // Temporary view uses temporary model
      Node temporary (map, context, model_manager.temp_node)

      // Update model
      map.pointer_lat =:> model_manager.temp_node.lat
      map.pointer_lon =:> model_manager.temp_node.lon

      f.release -> model_manager.create_node_from_temp
    }
    idle -> preview (context.ctrl, root.show_reticule)
    preview -> idle (context.ctrl_r, root.hide_reticule)
  }


  // Spikes
  Spike start_create_edges
  Spike stop_create_edges
  Spike clear_all

  context.del -> context.set_node_graph_edition_to_null
  context.del -> context.set_node_status_edition_to_null
  context.del -> clear_all
  

  // FSM to manage the addition of edges in the graph
  FSM fsm_add_edge {
    State idle

    State shift_on

    State preview_on {
      List temp_id_list

      root.context.id_node_graph_edition.value -> (root) { // FIXME: lambda capture should contain multiple symbols
        addChildrenTo root.graph.fsm_add_edge.preview_on.temp_id_list { // FIXME there should be a better way to ref temp_id_list
          String _ (toString(root.context.id_node_graph_edition.value))
        }

        int size = $root.graph.fsm_add_edge.preview_on.temp_id_list.size 
        string source_id = toString (root.graph.fsm_add_edge.preview_on.temp_id_list.[size - 1])
        string target_id = toString (root.graph.fsm_add_edge.preview_on.temp_id_list.[size])
        string edge_id = source_id + "_" + target_id

        addChildrenTo root.model.edge_ids {
          String _ (edge_id)
        }
        source = find (root.model.nodes, source_id)
        target = find (root.model.nodes, target_id)
        EdgeModel (root.model.edges, edge_id, source, target, 0.0)
      }
      
      Translation pos (0, 0)
      context.map_translation_x =:> pos.tx
      context.map_translation_y =:> pos.ty

      // UI of temporary edge
      NoFill _
      OutlineOpacity _ (0.5)
      OutlineWidth _ (5)
      //OutlineColor _ ($context.EDGE_COLOR)
      OutlineColor _ (#FFFF00)
    
      Line temp_shadow_edge (0, 0, 0, 0)


      context.dx_node_graph_edition.value =:> temp_shadow_edge.x1
      context.dy_node_graph_edition.value =:> temp_shadow_edge.y1

      f.move.x - pos.tx =:> temp_shadow_edge.x2
      f.move.y - pos.ty =:> temp_shadow_edge.y2
    }

    idle -> shift_on (context.shift, root.show_reticule)
    shift_on -> idle (context.shift_r, root.hide_reticule)
    shift_on -> preview_on (root.context.id_node_graph_edition.value, start_create_edges)
    preview_on -> idle (context.shift_r, stop_create_edges) // + hide_reticule
  }
  stop_create_edges -> root.hide_reticule
  stop_create_edges -> context.set_node_graph_edition_to_null


  clear_all -> (root) {
    print ("Clear all\n")
    
    delete_content root.graph.fsm_add_edge.preview_on.temp_id_list
    
    //delete_content root.l.map.layers.navgraph.edges
    delete_content root.model.edges

    //delete_content root.l.map.layers.navgraph.nodes
    delete_content root.model.nodes
  }

  start_create_edges -> (root) {
    //print ("Start create edges\n")

    // Add the id of the first selected node
    addChildrenTo root.graph.fsm_add_edge.preview_on.temp_id_list{
      String _ (toString(root.context.id_node_graph_edition.value))
    }
  }

  stop_create_edges -> na_stop_create_edges:(root) {
    delete_content root.graph.fsm_add_edge.preview_on.temp_id_list
  }
}