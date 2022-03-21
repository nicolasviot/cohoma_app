#ifndef NO_LEMON

#include <iostream>
#include "navgraph/navgraph.hpp"
#include "nlohmann/json.hpp"

namespace navgraph
{

void to_json(nlohmann::json& j, const GeoPoint& p) {
    j.emplace("latitude", p.latitude);
    j.emplace("longitude", p.longitude);
    j.emplace("altitude", p.altitude);   
}

void from_json(const nlohmann::json& j, GeoPoint& p) {
    j.at("latitude").get_to(p.latitude);
    j.at("longitude").get_to(p.longitude);
    if (j.contains("altitude"))
        j.at("altitude").get_to(p.altitude);
    else
        p.altitude = 0.0;
}

void from_json(const nlohmann::json& j, NavGraph& g) 
{
    std::cerr << "in from_json" << std::endl;
    g.graph_.clear();
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
        auto n = g.graph_.addNode();        
        g.nodes_by_index_[node["id"]] = n;
        g.node_id_[n] = node["id"];
        if (node.contains("label"))
            g.node_label_[n] = node["label"];
        if (node.contains("metadata")) {
            auto& m = node["metadata"];
            if (m.contains("latitude") && m.contains("longitude")) {
                g.node_geopoint_[n] = m.get<GeoPoint>();
            }
            if (m.contains("compulsory")){
                g.node_compulsory_[n] = m["compulsory"].get<bool>();
            }
        }
    }
    std::cerr << "about to parse edges" << std::endl;
    // edges
    for (auto& edge: j_graph["edges"]) {
        std::string source = edge["source"].get<std::string>();
        std::string target = edge["target"].get<std::string>();
        auto e = g.graph_.addEdge(g.nodes_by_index_[source], g.nodes_by_index_[target]);
        if (edge.contains("metadata")) {
            auto& m = edge["metadata"];
            if (m.contains("length"))
                g.edge_length_[e] = m["length"].get<double>();
        }
    }
}

void to_json(nlohmann::json& j, const NavGraph& g) {
    j["graph"]["directed"] = false;
    for (lemon::ListGraph::NodeIt n(g.graph_); n != lemon::INVALID; ++n) {
        auto& p = g.node_geopoint_[n];
        nlohmann::json jn = {
            {"id", g.node_id_[n]},
            {"label", g.node_label_[n]},
            {"metadata", {
                {"compulsory", g.node_compulsory_[n]}
            }}
        };                
        if (p.has_value()) to_json(jn["metadata"], p.value());
        j["graph"]["nodes"].push_back(jn);
    }
    for (lemon::ListGraph::EdgeIt e(g.graph_); e != lemon::INVALID; ++e) {
        std::stringstream source; source << g.node_id_[g.graph_.u(e)];
        std::stringstream target; target << g.node_id_[g.graph_.v(e)];
        
        j["graph"]["edges"].push_back({
            {"source", source.str()},
            {"target", target.str()},
            {"metadata", {
                {"length", g.edge_length_[e]}
            }}
        });
    }
}

}  // namespace navgraph

#endif

