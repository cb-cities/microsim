#pragma once

#include "RoadGraph/roadGraph.h"
#include "traffic/b18TrafficSP.h"
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

  std::shared_ptr<abm::Graph> street_graph() const { return street_graph_; }

  std::vector<std::vector<abm::graph::vertex_t>> edge_vertices();

  std::vector<std::vector<double>> edge_weights();

  void
  map_person2init_edge(const std::vector<abm::graph::edge_id_t> &all_paths);

  const unsigned long totalNumPeople() const { return od_pairs_.size(); }

  const std::vector<std::array<abm::graph::vertex_t, 2>> &od_pairs() {
    return od_pairs_;
  }

private:
  std::string edgeFileName_;
  std::string nodeFileName_;
  std::string odFileName_;
  std::shared_ptr<abm::Graph> street_graph_;
  std::vector<std::array<abm::graph::vertex_t, 2>> od_pairs_;

  void loadABMGraph_();

  void read_od_pairs_();

  std::vector<float> read_dep_times_();
};

} // namespace LC
