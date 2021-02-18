#include "network.h"
#include "Geometry/client_geometry.h"
#include "global.h"
#include "traffic/bTrafficIntersection.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace LC {

using namespace std::chrono;

Network::Network(const std::string &networkPath,
                 const std::string &odFileName) {
  edgeFileName_ = networkPath + "edges.csv";
  nodeFileName_ = networkPath + "nodes.csv";
  odFileName_ = networkPath + odFileName;
  street_graph_ = std::make_shared<abm::Graph>(true, networkPath);

  loadABMGraph_();
  read_od_pairs_();
  read_dep_times_();
}

void Network::loadABMGraph_() {
  std::cout << edgeFileName_ << " as edges file\n";
  std::cout << nodeFileName_ << " as nodes file\n";
  std::cout << odFileName_ << " as OD file\n";

  // EDGES
  street_graph_->read_graph_osm(edgeFileName_);
  // NODES
  street_graph_->read_vertices(nodeFileName_);
}

void Network::read_od_pairs_() {
  try {
    csvio::CSVReader<2> in(odFileName_);
    in.read_header(csvio::ignore_extra_column, "origin", "destination");
    abm::graph::vertex_t v1, v2;
    abm::graph::weight_t weight;
    while (in.read_row(v1, v2)) {
      // std::array<abm::graph::vertex_t, 2> od = {v1, v2};
      std::vector<long> od = {v1, v2};
      od_pairs_.emplace_back(od);
    }
  } catch (std::exception &exception) {
    std::cout << "Read OD file: " << exception.what() << "\n";
  }
}

void Network::read_dep_times_() {
  try {
    csvio::CSVReader<1> in(odFileName_);
    in.read_header(csvio::ignore_extra_column, "dep_time");
    float dep_time;
    while (in.read_row(dep_time)) {
      // printf("dep time %f\n", dep_time);
      dep_time_.emplace_back(dep_time);
    }
  } catch (std::exception &exception) {
    std::cout << "Read OD file: " << exception.what() << "\n";
  }
}

std::vector<std::vector<long >> Network::edge_vertices() {
  std::vector<std::vector<long>> edge_vertices;
  for (auto const &x : street_graph_->edge_ids_to_vertices) {
    auto edge_id = x.first;
    auto vertices = x.second;
    std::vector<long> v = {std::get<0>(vertices),
                                           std::get<1>(vertices)};
    edge_vertices.emplace_back(v);
  }
  return edge_vertices;
}

std::vector<std::vector<double>> Network::edge_weights() {
  std::vector<std::vector<double>> edge_weights;
  std::vector<double> weights;
  weights.reserve(street_graph_->edges_.size());

  for (auto const &x : street_graph_->edge_costs_) {

    weights.emplace_back(x.second);
  }
  edge_weights.emplace_back(weights);
  return edge_weights;
}

void Network::map_person2init_edge(
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
