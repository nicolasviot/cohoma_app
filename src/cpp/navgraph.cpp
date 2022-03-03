#include <iostream>
#include <limits>

#include "navgraph/navgraph.hpp"
#include "nlohmann/json.hpp"

namespace navgraph
{

NavGraph::NavGraph()
    : node_label_(graph_)
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

NavGraph::~NavGraph()
{
}

}  // namespace navgraph
