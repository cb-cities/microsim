#include "lanemap.h"
#include "config.h"
#include "sp/graph.h"
#include <cassert>
#include <cmath>
#include <ios>

namespace LC {

void Lanemap::create_edgesData_(const std::shared_ptr<abm::Graph> &graph) {
  edgesData_.resize(graph->nedges() * 6); // 6 times for enough space

  int lanemap_idx = 0;
  for (auto const &x : graph->edges_) {
    auto edge_vertices = std::get<1>(x)->first;
    auto edge_id =
        graph->edge_ids_[std::get<0>(edge_vertices)][std::get<1>(edge_vertices)];
    auto edge_val = std::get<1>(x)->second;
    const int numLanes = edge_val[1];

    if (numLanes == 0) {
      printf("Error! One edge has 0 lane.\n");
      abort();
    }

    edgesData_[lanemap_idx].length = edge_val[0];
    edgesData_[lanemap_idx].maxSpeedMperSec = edge_val[2];

    edgesData_[lanemap_idx].num_lanes = numLanes;
    edgesData_[lanemap_idx].vertex[0] = std::get<0>(std::get<0>(x));
    edgesData_[lanemap_idx].vertex[1] = std::get<1>(std::get<0>(x));

    mid2eid_[lanemap_idx] = edge_id;
    eid2mid_[edge_id] = lanemap_idx;
    const int numWidthNeeded =
        ceil(edge_val[0] / kMaxMapWidthM_); // number of cells for each lane
    lanemap_idx += numLanes * numWidthNeeded;
  }

  edgesData_.resize(lanemap_idx);
}

void Lanemap::create_LaneMap_() {
  laneMap_.resize(kMaxMapWidthM_ * edgesData_.size() *
                  2); // 2: to have two maps.
  memset(laneMap_.data(), -1, laneMap_.size() * sizeof(unsigned char)); //
}

void Lanemap::create_intersections_(const std::shared_ptr<abm::Graph> &graph) {
  intersections_.resize(graph->vertex_edges_.size()); // as many as vertices
  for (const auto &vertex_edges : graph->vertex_in_edges_) {
    auto vertex = std::get<0>(vertex_edges);
    auto in_edges = std::get<1>(vertex_edges);

    auto &intersection = intersections_[vertex];
    intersection.num_edge = 2 * in_edges.size();
    for (const auto &in_edge : in_edges) {
      for (const auto &out_edge : graph->vertex_out_edges_[vertex]) {
        auto edge_id1 =
            graph->edge_ids_[in_edge->first.first][in_edge->first.second];
        auto lanemap_id1 = eid2mid_[edge_id1];

        auto edge_id2 =
            graph->edge_ids_[out_edge->first.first][out_edge->first.second];
        auto lanemap_id2 = eid2mid_[edge_id2];

        if ((in_edge->first.first == out_edge->first.second) &&
            (in_edge->first.second == out_edge->first.first)) {
          continue; // skip turn around
        }

        intersection.start_edge[intersection.num_queue] = lanemap_id1;
        intersection.end_edge[intersection.num_queue] = lanemap_id2;
        intersection.num_queue++;
      }
    }
  }
}

} // namespace LC
