#ifndef NAVGRAPH__NAVGRAPH_HPP_
#define NAVGRAPH__NAVGRAPH_HPP_

#include <optional>
#include <lemon/list_graph.h>
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
  NavGraph();
  NavGraph(const std::string& j_string);

  virtual ~NavGraph();

  friend void from_json(const nlohmann::json& j, NavGraph& g);
  friend void to_json(nlohmann::json& j, const NavGraph& g);

//private:
  lemon::ListGraph graph_;
  std::map<std::string, lemon::ListGraph::Node> nodes_by_index_;
  lemon::ListGraph::NodeMap<std::string> node_label_;
  lemon::ListGraph::NodeMap< std::optional<GeoPoint> > node_geopoint_;
  lemon::ListGraph::NodeMap<bool> node_compulsory_;
  lemon::ListGraph::EdgeMap<double> edge_length_;

};

void from_json(const nlohmann::json& j, NavGraph& g);
void to_json(nlohmann::json& j, const NavGraph& g);

}  // namespace navgraph

#endif  // NAVGRAPH__NAVGRAPH_HPP_
