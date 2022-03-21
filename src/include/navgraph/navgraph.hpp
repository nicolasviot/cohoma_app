#ifndef NO_LEMON

#ifndef NAVGRAPH__NAVGRAPH_HPP_
#define NAVGRAPH__NAVGRAPH_HPP_

#include <optional>

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/adaptors.h>
#include <lemon/suurballe.h>
#include <lemon/path.h>
#include <lemon/bits/traits.h>

#include <nlohmann/json.hpp>
#include "navgraph/visibility_control.h"

namespace navgraph
{

struct GeoPoint {
  double latitude;
  double longitude;
  double altitude;
};

void to_json(nlohmann::json& j, const GeoPoint& p);
void from_json(const nlohmann::json& j, GeoPoint& p);

class NavGraph
{
public:
  typedef lemon::ListGraph::Node Node;
  typedef lemon::ListGraph::NodeIt NodeIterator;
  typedef lemon::ListGraph::Edge Edge;
  typedef lemon::ListGraph::EdgeIt EdgeIterator;

  NavGraph();
  NavGraph(const std::string& j_string);

  virtual ~NavGraph();

  inline NodeIterator nodes() { return lemon::ListGraph::NodeIt(graph_); };
  inline EdgeIterator edges() { return lemon::ListGraph::EdgeIt(graph_); };

  inline std::string get_label(const Node& node) const { return node_label_[node]; };
  inline std::string get_id(const Node& node) const {return node_id_[node];};
  inline GeoPoint get_geopoint(const Node& node) const { return node_geopoint_[node].value(); };
  inline double get_length(const Edge& edge) const { return edge_length_[edge]; };
  inline Node source(const Edge& edge) const { return graph_.u(edge); };
  inline Node target(const Edge& edge) const { return graph_.v(edge); };

  friend void from_json(const nlohmann::json& j, NavGraph& g);
  friend void to_json(nlohmann::json& j, const NavGraph& g);

  friend bool sort_using_greater_than(std::pair<std::string,double> ppo_a, std::pair<std::string,double> ppo_b);

  inline bool empty(){ return !lemon::countNodes(this->graph_); }

  std::vector<std::string> getting_ppos_list();
  std::vector<std::string> getting_ordered_ppos_list(std::string current_node);

  std::vector<std::string> finding_simple_path(std::vector<std::string> &nodes);
  std::vector<std::string> finding_path_wrt_ppo_list(std::vector<std::string> &nodes, std::vector<std::string> &ppo_ordered);  
  std::vector<std::string> finding_alternative_paths(std::vector<std::string> &nodes);

private:
  lemon::ListGraph graph_;
  std::map<std::string, lemon::ListGraph::Node> nodes_by_index_;
  lemon::ListGraph::NodeMap<std::string> node_label_;
  lemon::ListGraph::NodeMap<std::string> node_id_;
  lemon::ListGraph::NodeMap<std::optional<GeoPoint> > node_geopoint_;
  lemon::ListGraph::NodeMap<bool> node_compulsory_;
  lemon::ListGraph::EdgeMap<double> edge_length_;

};

void from_json(const nlohmann::json& j, NavGraph& g);
void to_json(nlohmann::json& j, const NavGraph& g);

inline bool sort_using_greater_than(std::pair<std::string,double> ppo_a, std::pair<std::string,double> ppo_b){return (ppo_a.second) > (ppo_b.second);}

}  // namespace navgraph

#endif  // NAVGRAPH__NAVGRAPH_HPP_
#endif
