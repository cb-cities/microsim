#include "RoadGraph/roadGraph.h"
#include "config.h"
#include "traffic/sp/graph.h"

namespace LC {

//! Network Class
//! \brief Base class for the network used in the traffic simulation
class Network {
public:
  // Constructor with network path
  //! \param[in] networkPath network file path
  Network(const std::string &networkPath);

  //! Destructor
  ~Network() = default;

  //! return the abm street graph
  const std::shared_ptr<abm::Graph> street_graph() const {
    return street_graph_;
  }

  // Get edge_id from node ids
  //! \param[in] v1 node id for one end of the edge
  //! \param[in] v2 node id for the other end of the edge
  //! \retval edge id
  abm::graph::vertex_t edge_id(abm::graph::vertex_t v1,
                               abm::graph::vertex_t v2) {
    if (!street_graph_->directed_) // Handle undirect cases
      if (v1 > v2)
        std::swap(v1, v2);

    return street_graph_->edge_ids_[v1][v2];
  }

  std::vector<std::vector<double>> edge_weights() { return edge_weights_; };

  abm::graph::vertex_t num_edges() { return street_graph_->edges_.size(); }

  int num_vertices() { return street_graph_->vertices_data_.size(); }

  std::vector<std::vector<long>> edge_vertices();

  std::vector<unsigned int> heads();

  std::vector<unsigned int> tails();

private:
  std::string edgeFileName_;
  std::string nodeFileName_;
  //! abm street graph (base graph for the network, defined in the sp folder)
  std::shared_ptr<abm::Graph> street_graph_;
  //! edge weights for route finding
  std::vector<std::vector<double>> edge_weights_;

  //! initialize abm graph (the base graph)
  void loadABMGraph_();
  //! initialize edge weights (free flow time)
  void init_edge_weights_();
};

} // namespace LC
