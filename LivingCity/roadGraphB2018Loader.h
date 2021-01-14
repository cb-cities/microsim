#pragma once

#include "RoadGraph/roadGraph.h"
#include "traffic/b18TrafficSP.h"
#include "traffic/sp/graph.h"
#include <QString>

namespace LC {

class DemandB2018 {
public:
  int num_people;
  int src_vertex;
  int tgt_vertex;
  DemandB2018(int num_people, int src_vertex, int tgt_vertex)
      : num_people(num_people), src_vertex(src_vertex), tgt_vertex(tgt_vertex) {
  }
};

/**
 * RoadGraph.
 **/
class RoadGraphB2018 {

public:
  RoadGraphB2018(const std::string &networkPath,
                 const std::string &odDemandPath) {
    edgeFileName_ = networkPath + "edges.csv";
    nodeFileName_ = networkPath + "nodes.csv";
    odFileName_ = networkPath + odDemandPath;
    street_graph_ = std::make_shared<abm::Graph>(true, networkPath);
  };

  //! Destructor
  ~RoadGraphB2018() = default;

  void loadABMGraph();

  std::vector<std::array<abm::graph::vertex_t, 2>>
  read_od_pairs(int nagents = std::numeric_limits<int>::max());

  std::vector<float> read_dep_times();

  const std::shared_ptr<abm::Graph> street_graph() const {
    return street_graph_;
  }

private:
  std::string edgeFileName_;
  std::string nodeFileName_;
  std::string odFileName_;
  std::shared_ptr<abm::Graph> street_graph_;
  std::vector<DemandB2018> demandB2018_;
  unsigned long totalNumPeople_;
};

} // namespace LC
