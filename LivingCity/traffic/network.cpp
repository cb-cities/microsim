#include "network.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace LC {

using namespace std::chrono;

Network::Network(const std::string &networkPath) {
  edgeFileName_ = networkPath + "edges.csv";
  nodeFileName_ = networkPath + "nodes.csv";
  street_graph_ = std::make_shared<abm::Graph>(true, networkPath);

  loadABMGraph_();
  init_edge_weights_();
}

void Network::loadABMGraph_() {
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

} // namespace LC
