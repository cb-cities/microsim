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

std::vector<std::vector<long>> RoadGraphB2018::edge_vals() {
  std::vector<std::vector<long>> edge_vals;
  for (auto const &x : street_graph_->edges_) {
    std::vector<long> edge_nodes = {std::get<0>(x.first), std::get<1>(x.first)};
    edge_vals.emplace_back(edge_nodes);
  }

  return edge_vals;
}

std::vector<std::vector<double>> RoadGraphB2018::edge_weights() {
  std::vector<std::vector<double>> edge_weights;
  std::vector<double> edge_weights_inside_vec;
  edge_weights_inside_vec.reserve(street_graph_->edges_.size());

  for (auto const &x : street_graph_->edges_) {
    double metersLength = std::get<1>(x)->second[0];
    edge_weights_inside_vec.emplace_back(metersLength);
  }
  edge_weights.emplace_back(edge_weights_inside_vec);
  return edge_weights;
}

void RoadGraphB2018::map_person2init_edge(
    const std::vector<abm::graph::edge_id_t> &all_paths) {
  int count = 0;
  for (int i = 0; i < all_paths.size(); i++) {
    if (i == 0) { // first one that doesn't contain a -1 for logic
      street_graph_->person_to_init_edge_[count] = i;
      count++;
    } else if ((all_paths[i] == -1) &&
               (all_paths[i + 1] == -1)) { // if current is -1 and next is -1,
                                           // increment (will result in nan)
      street_graph_->person_to_init_edge_[count] = i;
      count++;
    } else if ((all_paths[i] != -1) &&
               (all_paths[i - 1] ==
                -1)) { // if previous is -1, use this as first edge for p
      street_graph_->person_to_init_edge_[count] = i;
      count++;
    } else if ((all_paths[i] == -1) &&
               (i == (all_paths.size() - 1))) { // reach the end
      break;
    }
  }
}

} // namespace LC
