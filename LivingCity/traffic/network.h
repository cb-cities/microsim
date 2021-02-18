#pragma once

#include "RoadGraph/roadGraph.h"
#include "config.h"
#include "traffic/sp/graph.h"

namespace LC {

/**
 * RoadGraph.
 **/
class Network {

public:
  Network(const std::string &networkPath, const std::string &odFileName);

  //! Destructor
  ~Network() = default;

  abm::graph::vertex_t edge_id(abm::graph::vertex_t v1,
                               abm::graph::vertex_t v2) {
    return street_graph_->edge_ids_[v1][v2];
  }
  std::vector<std::vector<long >> edge_vertices();

  std::vector<std::vector<double>> edge_weights();

  abm::graph::vertex_t num_edges() { return street_graph_->edges_.size(); }
  int num_vertices() {
    return street_graph_->vertices_data_.size();
  }

  const std::vector<std::vector<long>> &od_pairs() { return od_pairs_; }
  const std::vector<float> &dep_time() const { return dep_time_; }
  const unsigned long totalNumPeople() const { return od_pairs_.size(); }
  void
  map_person2init_edge(const std::vector<abm::graph::edge_id_t> &all_paths);

private:
  std::string edgeFileName_;
  std::string nodeFileName_;
  std::string odFileName_;
  std::shared_ptr<abm::Graph> street_graph_;
  std::vector<std::vector<long>> od_pairs_;
  std::vector<float> dep_time_;

  void loadABMGraph_();

  void read_od_pairs_();

  void read_dep_times_();
};

} // namespace LC
