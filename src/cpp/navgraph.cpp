#ifndef NO_LEMON

#include <iostream>
#include <limits>

#include "navgraph/navgraph.hpp"
#include "nlohmann/json.hpp"

namespace navgraph
{

NavGraph::NavGraph()
    : node_label_(graph_)
    , node_id_(graph_)
    , node_geopoint_(graph_, {})
    , node_compulsory_(graph_, false)
    , edge_length_(graph_, std::numeric_limits<double>::infinity())
{
}

NavGraph::NavGraph(const std::string& j_string)
    : NavGraph()
{
    nlohmann::json j = nlohmann::json::parse(j_string);
    from_json(j, *this);
}

/* Finds a simple path between souce and target nodes */
std::vector<std::string> NavGraph::finding_simple_path(std::vector<std::string> &nodes )
{
    // checking message
    if((int)nodes.size()==0 || (int)nodes.size()>2){
        std::cerr << "ERROR : Nodes is empty or too large" << std::endl;
        std::vector<std::string> error_msg;
        return error_msg;
    }
    // checking nodes existence
    auto its = this->nodes_by_index_.find(nodes[0]);
    auto itt = this->nodes_by_index_.find(nodes[1]);
    if (its == this->nodes_by_index_.end() || itt == this->nodes_by_index_.end()){
        std::cerr << "ERROR : at least one of the nodes is not in the graph " << nodes[0] << " " << nodes[1] << std::endl;
        std::vector<std::string> error_msg;
        return error_msg;
        }
    
        
    lemon::ListGraph::Node start_node = this->nodes_by_index_[nodes[0]];
    lemon::ListGraph::Node target_node = this->nodes_by_index_[nodes[1]];
    //std::cout << "From node " << nodes[0] << " to " << nodes[1] << std::endl;
    //std::cout << "From node " << this->node_id_[start_node] << " to " << this->node_id_[target_node] << std::endl; 
    //std::cout << this->graph_.id(start_node) << " " << this->graph_.id(target_node) << std::endl;
    lemon::Dijkstra<lemon::ListGraph, lemon::ListGraph::EdgeMap<double> > dijkstra_alg(this->graph_, this->edge_length_);
    dijkstra_alg.run(start_node, target_node);
    // Getting result
    lemon::Path<lemon::ListGraph> edges_path =  dijkstra_alg.path(target_node);
    std::vector<std::string> path_nodes;

    // Getting Nodes in the path    
    if(!edges_path.empty()){
        //std::cout << " Path: " << std::endl;
        for (lemon::Path<lemon::ListGraph>::ArcIt arc(edges_path); arc != lemon::INVALID; ++arc)
        {
            //getting all edge source node 
            path_nodes.push_back(this->node_id_[this->graph_.source(arc)]);
            //std::cout << nodes_path.back() << std::endl;
        }
        path_nodes.push_back(this->node_id_[target_node]);
        //std::cout << nodes_path.back() << std::endl;
    }
    // returning path
    return path_nodes;


}

/*Gets the ppo_list*/
std::vector<std::string> NavGraph::getting_ppos_list(){
    std::vector<std::string> ppos_list;
    for (lemon::ListGraph::NodeIt n(this->graph_); n != lemon::INVALID; ++n) {
        //std::cout << "Searching PPO " << this->node_id_[n] << std::endl;
        if (this->node_compulsory_[n]) {
            //std::cout << "      Yes" << std::endl;
            ppos_list.push_back(this->node_id_[n]);
        }
    }
    return ppos_list;
}

/* Gets an ordered PPO list wrt to a given node */
std::vector<std::string> NavGraph::getting_ordered_ppos_list(std::string current_node){
    std::vector<std::string> ppos_ordered;
    std::vector<std::string> ppos_list = this->getting_ppos_list();
    if(!ppos_list.empty()){
        if((int)ppos_list.size()>1){
            lemon::Dijkstra<lemon::ListGraph,lemon::ListGraph::EdgeMap<double>> dijkstra_alg(this->graph_, this->edge_length_);
            lemon::ListGraph::NodeMap<double> nodes_distances(this->graph_);
            dijkstra_alg.distMap(nodes_distances);
            dijkstra_alg.run(this->nodes_by_index_[current_node]);
            std::map<double,std::string> distances_to_ppo;
            std::vector<double> ppo_distances;
            for(auto ppo : ppos_list){
                //std::cout<<"dist from node " << current_node <<" to PPO " << ppo << " " << nodes_distances[this->nodes_by_index_[ppo]] << std::endl;
                distances_to_ppo.insert(std::make_pair(nodes_distances[this->nodes_by_index_[ppo]],ppo) );
                ppo_distances.push_back(nodes_distances[this->nodes_by_index_[ppo]]);
            }
            std::sort(ppo_distances.begin(),ppo_distances.end());
            for(auto dist : ppo_distances)
                ppos_ordered.push_back(distances_to_ppo[dist]);
            
        }
        else{
            ppos_ordered = ppos_list;
        }
    }
    return ppos_ordered;
}

/* Finds a path that passes throw all PPOs*/
std::vector<std::string> NavGraph::finding_path_wrt_ppo_list(std::vector<std::string> &nodes, 
                                                            std::vector<std::string> &ppo_ordered)
                                                            {
    std::vector<std::string> path_nodes;
    std::vector<lemon::Path<lemon::ListGraph>> path_chuncks;
    lemon::ListGraph::Node start_node = this->nodes_by_index_[nodes[0]];
    lemon::ListGraph::Node target_node = this->nodes_by_index_[nodes[1]];
    ppo_ordered.push_back(this->node_id_[target_node]);

    if((int)nodes.size()==0 || (int)nodes.size()>2){
        std::cerr << "ERROR : Nodes is empty or too large" << std::endl;
        std::vector<std::string> error_msg;
        return error_msg;
    }
    // now need to find the path covering all PPOs given the orderedcost
    lemon::Dijkstra<lemon::ListGraph,lemon::ListGraph::EdgeMap<double>> djk_alg(this->graph_, this->edge_length_);
    for(auto ppo: ppo_ordered){
        //std::cout << "starting from " << this->g.id(start_node) << std::endl;
        djk_alg.run(start_node, this->nodes_by_index_[ppo]);
        path_chuncks.push_back(djk_alg.path(this->nodes_by_index_[ppo]));
        start_node = this->nodes_by_index_[ppo];
        djk_alg.init();
    }
    // merging paths
    lemon::Path<lemon::ListGraph> res_path;
    for (int p =0; p<(int)path_chuncks.size();p++){
        for (lemon::Path<lemon::ListGraph>::ArcIt arc(path_chuncks[p]); arc != lemon::INVALID; ++arc){
            res_path.addBack(arc);
        }
    }
    // verifing the merging
    //std::cout << "Path passing by all PPOs" << std::endl;
    for (lemon::Path<lemon::ListGraph>::ArcIt arc(res_path); arc != lemon::INVALID; ++arc){
            //std::cout << "arc from node: " << this->node_id_[this->graph_.source(arc)] << 
            //" to node: " << this->node_id_[this->graph_.target(arc)] << std::endl;
            path_nodes.push_back(this->node_id_[this->graph_.source(arc)]);
    }
    path_nodes.push_back(this->node_id_[target_node]);

    return path_nodes;
}

/* Finds a alternative paths between source and target nodes*/
std::vector<std::string> NavGraph::finding_alternative_paths(std::vector<std::string> &nodes){
    std::vector<std::string> path_nodes;
    return path_nodes;
}




NavGraph::~NavGraph()
{
}

}  // namespace navgraph

#endif
