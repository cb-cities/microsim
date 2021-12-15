#include "network.h"
#include "Geometry/client_geometry.h"
#include "global.h"
#include "traffic/bTrafficIntersection.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace LC {

using namespace std::chrono;

Network::Network(const std::string &networkPath) {
  edgeFileName_ = networkPath + "edges.csv";
  nodeFileName_ = networkPath + "nodes.csv";
  street_graph_ = std::make_shared<abm::Graph>(false, networkPath);

  loadABMGraph_();
  init_edge_weights_();
}

void Network::loadABMGraph_() {
  //  std::cout << edgeFileName_ << " as edges file\n";
  //  std::cout << nodeFileName_ << " as nodes file\n";

  // EDGES
  street_graph_->read_graph_osm(edgeFileName_);
  // NODES
  street_graph_->read_vertices(nodeFileName_);
}

std::vector<std::vector<long>> Network::edge_vertices() {
  std::vector<std::vector<long>> edge_vertices;
  for (auto const &x : street_graph_->edge_ids_to_vertices) {
    auto vertices = x.second;
    std::vector<long> v = {std::get<0>(vertices), std::get<1>(vertices)};
    edge_vertices.emplace_back(v);
  }
  return edge_vertices;
}

std::vector<unsigned int> Network::heads() {
  std::vector<unsigned int> heads;
  for (auto const &x : street_graph_->edge_ids_to_vertices) {
    auto vertices = x.second;
    heads.emplace_back(std::get<0>(vertices));
  }
  return heads;
}

std::vector<unsigned int> Network::tails() {
  std::vector<unsigned int> tails;
  for (auto const &x : street_graph_->edge_ids_to_vertices) {
    auto vertices = x.second;
    tails.emplace_back(std::get<1>(vertices));
  }
  return tails;
}

void Network::init_edge_weights_() {
  std::vector<double> weights;
  weights.reserve(street_graph_->edges_.size());

  for (auto const &x : street_graph_->edge_costs_) {
    weights.emplace_back(x.second);
  }
  edge_weights_.emplace_back(weights);
}

// void Network::map_person2init_edge(
//    const std::vector<abm::graph::edge_id_t> &all_paths) {
//  int count = 0;
//  for (int i = 0; i < all_paths.size(); i++) {
//    if (i == 0) { // first one that doesn't contain a -1 for logic
//      street_graph_->person_to_init_edge_[count] = i;
//      count++;
//    } else if ((all_paths[i] == -1) &&
//               (all_paths[i + 1] == -1)) { // if current is -1 and next is -1,
//                                           // increment (will result in nan)
//      street_graph_->person_to_init_edge_[count] = i;
//      count++;
//    } else if ((all_paths[i] != -1) &&
//               (all_paths[i - 1] ==
//                -1)) { // if previous is -1, use this as first edge for p
//      street_graph_->person_to_init_edge_[count] = i;
//      count++;
//    } else if ((all_paths[i] == -1) &&
//               (i == (all_paths.size() - 1))) { // reach the end
//      break;
//    }
//  }
//}

} // namespace LC
