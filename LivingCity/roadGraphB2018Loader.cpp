#include <iostream>
#include <stdexcept>
#include <string>
using namespace std;

#include "Geometry/client_geometry.h"
#include "global.h"
#include "roadGraphB2018Loader.h"
#include "traffic/bTrafficIntersection.h"

namespace LC {

using namespace std::chrono;

void RoadGraphB2018::loadABMGraph() {
  std::cout << edgeFileName_ << " as edges file\n";
  std::cout << nodeFileName_ << " as nodes file\n";
  std::cout << odFileName_ << " as OD file\n";

  // EDGES
  street_graph_->read_graph_osm(edgeFileName_);

  // NODES
  street_graph_->read_vertices(nodeFileName_);
}

std::vector<std::array<abm::graph::vertex_t, 2>>
RoadGraphB2018::read_od_pairs(int nagents) {
  std::vector<std::array<abm::graph::vertex_t, 2>> od_pairs;
  try {
    csvio::CSVReader<2> in(odFileName_);
    in.read_header(csvio::ignore_extra_column, "origin", "destination");
    abm::graph::vertex_t v1, v2;
    abm::graph::weight_t weight;
    while (in.read_row(v1, v2)) {
      // std::array<abm::graph::vertex_t, 2> od = {v1, v2};
      std::array<abm::graph::vertex_t, 2> od = {v1, v2};
      od_pairs.emplace_back(od);
      demandB2018_.push_back(
          DemandB2018(1, v1, v2)); // there is only one person for each OD pair
    }
    totalNumPeople_ = demandB2018_.size();
    if (nagents != std::numeric_limits<int>::max())
      od_pairs.resize(nagents);
  } catch (std::exception &exception) {
    std::cout << "Read OD file: " << exception.what() << "\n";
  }
  return od_pairs;
}

std::vector<float> RoadGraphB2018::read_dep_times() {
  std::vector<float> dep_time_vec;
  try {
    csvio::CSVReader<1> in(odFileName_);
    in.read_header(csvio::ignore_extra_column, "dep_time");
    float dep_time;
    while (in.read_row(dep_time)) {
      // printf("dep time %f\n", dep_time);
      dep_time_vec.emplace_back(dep_time);
    }
  } catch (std::exception &exception) {
    std::cout << "Read OD file: " << exception.what() << "\n";
  }
  return dep_time_vec;
}

} // namespace LC
