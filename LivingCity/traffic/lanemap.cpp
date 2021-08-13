#include "lanemap.h"
#include "config.h"
#include "sp/graph.h"
#include <cassert>
#include <cmath>
#include <ios>

namespace LC {

namespace {
bool compareSecondPartTupleC(
    const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float> &i,
    const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float> &j) {
  return (i.second < j.second);
}
bool compareSecondPartTupleCSP(
    const std::pair<std::shared_ptr<abm::Graph::Edge>, float> &i,
    const std::pair<std::shared_ptr<abm::Graph::Edge>, float> &j) {
  return (i.second < j.second);
}

} // namespace

void Lanemap::create_edgesData_(const std::shared_ptr<abm::Graph> &graph) {

  abm::graph::edge_id_t max_edge_id = 0;
  for (int v1_index = 0; v1_index < graph->edge_ids_.size(); v1_index++) {
    for (const std::pair<abm::graph::vertex_t, abm::graph::edge_id_t> v2_it :
         graph->edge_ids_[v1_index]) {
      max_edge_id = std::max(max_edge_id, v2_it.second);
    }
  }
  edgeIdToLaneMapNum_.resize(max_edge_id + 1);
  edgesData_.resize(graph->nedges() * 4); // 4 to make sure it fits
  /////////////////////////////////
  // Create EdgeData
  // Instead of having maxWidth (for b18 would have been 26km), we define a max
  // width for the map and we wrap down.
  int tNumMapWidth = 0;
  for (auto const &x : graph->edges_) {
    auto edge_val = std::get<1>(x)->second;
    const int numLanes = edge_val[1];
    if (numLanes == 0) {
      printf("Error! One edge has 0 lane.\n");
      continue;
    }

    edgesData_[tNumMapWidth].length = edge_val[0];
    edgesData_[tNumMapWidth].maxSpeedMperSec = edge_val[2];

    const int numWidthNeeded = ceil(edge_val[0] / kMaxMapWidthM_);
    edgesData_[tNumMapWidth].numLines = numLanes;
    edgesData_[tNumMapWidth].nextInters = std::get<1>(std::get<0>(x));

    edgeDescToLaneMapNum_.insert(std::make_pair(x.second, tNumMapWidth));
    laneMapNumToEdgeDesc_.insert(std::make_pair(tNumMapWidth, x.second));

    auto edge_vertices = std::get<1>(x)->first;
    auto edge_id =
        graph->edge_ids_[get<0>(edge_vertices)][get<1>(edge_vertices)];

    edgeIdToLaneMapNum_[edge_id] = tNumMapWidth;

    tNumMapWidth += numLanes * numWidthNeeded;
  }

  edgesData_.resize(tNumMapWidth);
  tNumMapWidth_ = tNumMapWidth;
}

void Lanemap::create_LaneMap_() {
  laneMap_.resize(kMaxMapWidthM_ * tNumMapWidth_ * 2); // 2: to have two maps.
  memset(laneMap_.data(), -1, laneMap_.size() * sizeof(unsigned char)); //
}

void Lanemap::read_path(std::vector<abm::graph::edge_id_t> paths) {
  for (abm::graph::edge_id_t &edge_in_path : paths) {
    if (edge_in_path != -1) {
      indexPathVec_.emplace_back(edgeIdToLaneMapNum_[edge_in_path]);
    } else {
      indexPathVec_.emplace_back(-1);
    }
  }
}

void Lanemap::create_intersections_(const std::shared_ptr<abm::Graph> &graph) {
  intersections_.resize(graph->vertex_edges_.size()); // as many as vertices
  traffic_lights_.assign(tNumMapWidth_, 0);
  for (const auto &vertex_edges : graph->vertex_edges_) {
    auto vertex = std::get<0>(vertex_edges);
    auto & intersection = intersections_[vertex];
    // match the edges
    auto edges = graph->vertex_in_edges_[vertex];
//    auto out_edges = graph->vertex_out_edges_[vertex];
//    edges.insert( edges.end(), out_edges.begin(), out_edges.end() );
    for (unsigned i = 0; i < edges.size(); i++) {
      auto edge_in = edges[i];
      auto inedge_id =
          graph->edge_ids_[edge_in->first.first][edge_in->first.second];
      auto lanemap_id_in = edgeIdToLaneMapNum_[inedge_id];
      for (unsigned j = i + 1; j < edges.size(); j++) {
        auto edge_out = edges[j];
        auto outedge_id =
            graph->edge_ids_[edge_out->first.first][edge_out->first.second];
        auto lanemap_id_out = edgeIdToLaneMapNum_[outedge_id];

        IntersectionQ q_in, q_out;
        std::vector<IntersectionQ> q_pair = {q_in, q_out};
        intersection.paired_queues.emplace_back(q_pair);
        std::vector<unsigned int> dir1 = {lanemap_id_in,lanemap_id_out};
        std::vector<unsigned int> dir2 = {lanemap_id_out,lanemap_id_in};
        intersection.dir2q[dir1] = &intersection.paired_queues.back()[0];
        intersection.dir2q[dir2] = &intersection.paired_queues.back()[1];
      }
    }
  }
}

//    // sort by angle
//    QVector3D p0, p1;
//    // edgeAngleOut;
//    std::vector<std::pair<std::shared_ptr<abm::Graph::Edge>, float>>
//        edgeAngleOut;
//    int numOutEdges = 0;
//    for (const auto &edge : graph->vertex_out_edges_[vertex]) {
//
//      if (edge->second[1] == 0) {
//        continue;
//      }
//
//      p0 = graph->vertices_data_[edge->first.first];
//      p1 = graph->vertices_data_[edge->first.second];
//
//      QVector3D edgeDir = (p1 - p0).normalized();
//      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
//
//      edgeAngleOut.push_back(std::make_pair(edge, angle));
//
//      if (edgeDescToLaneMapNum_.find(edge) == edgeDescToLaneMapNum_.end())
//      {
//        std::cout << "p0 = " << edge->first.first << "\n";
//        std::cout << "p1 = " << edge->first.second << "\n";
//        printf("->ERROR OUT\n"); // edge desc not found in map
//      }
//
//      numOutEdges++;
//      // edgeAngleOut.push_back(std::make_pair(edge_pair.first,angle));
//    }
//
//    std::vector<std::pair<std::shared_ptr<abm::Graph::Edge>, float>>
//        edgeAngleIn;
//    int numInEdges = 0;
//
//    for (const auto &edge : graph->vertex_in_edges_[vertex]) {
//      if (edge->second[1] == 0) {
//        continue;
//      }
//
//      p0 = graph->vertices_data_[edge->first.first];
//      p1 = graph->vertices_data_[edge->first.second];
//
//      QVector3D edgeDir = (p0 - p1).normalized();
//      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
//
//      edgeAngleIn.push_back(std::make_pair(edge, angle));
//
//      if (edgeDescToLaneMapNum_.find(edge) == edgeDescToLaneMapNum_.end())
//      {
//        printf("->ERROR IN\n"); // edge desc not found in map
//        continue;
//      }
//
//      numInEdges++;
//    }
//
//    intersections_[vertex].totalInOutEdges = numOutEdges + numInEdges;
//    // save in sorterd way as lane number
//    if (edgeAngleOut.size() > 0) {
//      std::sort(edgeAngleOut.begin(), edgeAngleOut.end(),
//                compareSecondPartTupleCSP);
//    }
//
//    if (edgeAngleIn.size() > 0) {
//      std::sort(edgeAngleIn.begin(), edgeAngleIn.end(),
//                compareSecondPartTupleCSP);
//    }
//    //!!!!
//    int outCount = 0;
//    int inCount = 0;
//    int totalCount = 0;
//
//    // Use intersections[std::get<0>(vertex)] = blah blah to set those
//    values
//    // for each vertex Intersection data:
//    //  Store the edges that go in or out of this intersection
//    //  Said edges will be sorted by angle
//    //
//    //      0xFF00 0000 Num lines
//    //      0x0080 0000 in out (one bit)
//    //      0x007F FFFF Edge number
//    for (int iter = 0; iter < edgeAngleOut.size() + edgeAngleIn.size();
//         iter++) {
//      if ((outCount < edgeAngleOut.size() && inCount < edgeAngleIn.size()
//      &&
//           edgeAngleOut[outCount] <= edgeAngleIn[inCount]) ||
//          (outCount < edgeAngleOut.size() && inCount >=
//          edgeAngleIn.size())) {
//        assert(edgeDescToLaneMapNum_[edgeAngleOut[outCount].first] <
//                   0x007fffff &&
//               "Edge number is too high");
//        intersections_[vertex].edge[totalCount] =
//            edgeDescToLaneMapNum_[edgeAngleOut[outCount].first];
//        intersections_[vertex].edge[totalCount] |=
//            (edgesData_[intersections_[vertex].edge[totalCount]].numLines
//             << 24); // put the number of lines in each edge
//        intersections_[vertex].edge[totalCount] |=
//            kMaskOutEdge; // 0x000000 mask to define out edge
//        // std::cout << "edgeDesc " <<
//        // edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first] << "\n";
//        outCount++;
//      } else {
//        assert(edgeDescToLaneMapNum_[edgeAngleIn[inCount].first] <
//        0x007fffff &&
//               "Edge number is too high");
//        intersections_[vertex].edge[totalCount] =
//            edgeDescToLaneMapNum_[edgeAngleIn[inCount].first];
//        intersections_[vertex].edge[totalCount] |=
//            (edgesData_[intersections_[vertex].edge[totalCount]].numLines
//             << 24); // put the number of lines in each edge
//        intersections_[vertex].edge[totalCount] |=
//            kMaskInEdge; // 0x800000 mask to define in edge
//        inCount++;
//      }
//
//      totalCount++;
//    }
//    // std::cout << "outCount = " << outCount << " inCount = " << inCount
//    <<
//    // "\n";
//
//    if (totalCount != intersections_[vertex].totalInOutEdges) {
//      printf(
//          "Error totalCount!=intersections[vertex].totalInOutEdges %d %
//          d\n ", totalCount, intersections_[vertex].totalInOutEdges);
//    }
//  }
//  }

// void B18TrafficLaneMap::createLaneMap(
//    const std::shared_ptr<abm::Graph> &graph, std::vector<uchar> &laneMap,
//    std::vector<B18IntersectionData> &intersections,
//    std::vector<uchar> &trafficLights,
//    std::map<uint, std::shared_ptr<abm::Graph::Edge>>
//    &laneMapNumToEdgeDescSP, std::map<std::shared_ptr<abm::Graph::Edge>,
//    uint> &edgeDescToLaneMapNumSP, std::vector<uint> &edgeIdToLaneMapNum) {
//
//  // 2. RESIZE LANE MAP
//  printf("Total Memory %d\n", kMaxMapWidthM * tNumMapWidth * 2);
//  laneMap.resize(kMaxMapWidthM * tNumMapWidth * 2); // 2: to have two maps.
//  memset(laneMap.data(), -1, laneMap.size() * sizeof(unsigned char)); //
//
//  //////////////////////////////////////////////////////////
//  // GENERATE INTERSECTION INFO
//  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
//  RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
//  RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
//  //
//  intersections.resize(boost::num_vertices(inRoadGraph.myRoadGraph_BI)); //
//  as
//  // many as vertices
//  intersections.resize(graph->vertex_edges_.size()); // as many as vertices
//  // std::cout << "intersections size = " << intersections.size() << "\n";
//  trafficLights.assign(tNumMapWidth, 0);
//  // trafficLights.resize(tNumMapWidth); // we could use tNumLanes but then
//  the
//      // edge number would not match and we would need to add logic.
//      // memset(trafficLights.data(), 0,
//      trafficLights.size()*sizeof(uchar));
//
//      int index = 0;
//  for (const auto &vertex : graph->vertex_edges_) {
//    // std::cout << "GET<0> VERTEX = " << std::get<0>(vertex) << "\n";
//    // intersections[std::get<0>(vertex)].state = 0;
//    // intersections[std::get<0>(vertex)].nextEvent = 0.0f;
//    // intersections[std::get<0>(vertex)].totalInOutEdges =
//    // vertex.second.size();
//    intersections[std::get<0>(vertex)].nextEvent = 0.0f;
//    intersections[std::get<0>(vertex)].totalInOutEdges =
//    vertex.second.size(); if
//    (intersections[std::get<0>(vertex)].totalInOutEdges <= 0) {
//      printf("Vertex without in/out edges\n");
//      continue;
//    }
//
//    if (intersections[std::get<0>(vertex)].totalInOutEdges >= 20) {
//      printf("Vertex with more than 20 in/out edges\n");
//      continue;
//    }
//    index++;
//
//    // sort by angle
//    QVector3D referenceVector(0, 1, 0);
//    QVector3D p0, p1;
//    // std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float>>
//    // edgeAngleOut;
//    std::vector<std::pair<std::shared_ptr<abm::Graph::Edge>, float>>
//        edgeAngleOut;
//    int numOutEdges = 0;
//
//    float angleRef = atan2(referenceVector.y(), referenceVector.x());
//
//    for (const auto &edge : graph_->vertex_out_edges_[std::get<0>(vertex)])
//    {
//      // if (inRoadGraph.myRoadGraph_BI[*Oei].numberOfLanes == 0) {
//      continue;
//    }
//    if (edge->second[1] == 0) {
//      continue;
//    }
//
//    p0 = graph_->vertices_data_[edge->first.first];
//    p1 = graph_->vertices_data_[edge->first.second];
//
//    QVector3D edgeDir = (p1 - p0).normalized();
//    float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
//
//    edgeAngleOut.push_back(std::make_pair(edge, angle));
//
//    if (edgeDescToLaneMapNumSP.find(edge) == edgeDescToLaneMapNumSP.end()) {
//      std::cout << "p0 = " << edge->first.first << "\n";
//      std::cout << "p1 = " << edge->first.second << "\n";
//      printf("->ERROR OUT\n"); // edge desc not found in map
//    }
//
//    numOutEdges++;
//    // edgeAngleOut.push_back(std::make_pair(edge_pair.first,angle));
//  }
//
//  std::vector<std::pair<std::shared_ptr<abm::Graph::Edge>, float>>
//  edgeAngleIn; int numInEdges = 0;
//
//  for (const auto &edge : graph_->vertex_in_edges_[std::get<0>(vertex)]) {
//    if (edge->second[1] == 0) {
//      continue;
//    }
//
//    p0 = graph_->vertices_data_[edge->first.first];
//    p1 = graph_->vertices_data_[edge->first.second];
//
//    QVector3D edgeDir = (p0 - p1).normalized();
//    float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
//
//    edgeAngleIn.push_back(std::make_pair(edge, angle));
//
//    if (edgeDescToLaneMapNumSP.find(edge) == edgeDescToLaneMapNumSP.end()) {
//      printf("->ERROR IN\n"); // edge desc not found in map
//      continue;
//    }
//
//    numInEdges++;
//  }
//
//  intersections[std::get<0>(vertex)].totalInOutEdges = numOutEdges +
//  numInEdges;
//  // save in sorterd way as lane number
//  if (edgeAngleOut.size() > 0) {
//    std::sort(edgeAngleOut.begin(), edgeAngleOut.end(),
//              compareSecondPartTupleCSP);
//  }
//
//  if (edgeAngleIn.size() > 0) {
//    std::sort(edgeAngleIn.begin(), edgeAngleIn.end(),
//              compareSecondPartTupleCSP);
//  }
//  //!!!!
//  int outCount = 0;
//  int inCount = 0;
//  int totalCount = 0;
//
//  // Use intersections[std::get<0>(vertex)] = blah blah to set those values
//  // for each vertex Intersection data:
//  //  Store the edges that go in or out of this intersection
//  //  Said edges will be sorted by angle
//  //
//  //      0xFF00 0000 Num lines
//  //      0x0080 0000 in out (one bit)
//  //      0x007F FFFF Edge number
//  for (int iter = 0; iter < edgeAngleOut.size() + edgeAngleIn.size();
//  iter++)
//  {
//    if ((outCount < edgeAngleOut.size() && inCount < edgeAngleIn.size() &&
//         edgeAngleOut[outCount] <= edgeAngleIn[inCount]) ||
//        (outCount < edgeAngleOut.size() && inCount >= edgeAngleIn.size())) {
//      assert(edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first] <
//                 0x007fffff &&
//             "Edge number is too high");
//      intersections[std::get<0>(vertex)].edge[totalCount] =
//          edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first];
//      intersections[std::get<0>(vertex)].edge[totalCount] |=
//          (edgesData_[intersections[std::get<0>(vertex)].edge[totalCount]]
//               .numLines
//           << 24); // put the number of lines in each edge
//      intersections[std::get<0>(vertex)].edge[totalCount] |=
//          kMaskOutEdge; // 0x000000 mask to define out edge
//      // std::cout << "edgeDesc " <<
//      // edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first] << "\n";
//      outCount++;
//    } else {
//      assert(edgeDescToLaneMapNumSP[edgeAngleIn[inCount].first] < 0x007fffff
//      &&
//             "Edge number is too high");
//      intersections[std::get<0>(vertex)].edge[totalCount] =
//          edgeDescToLaneMapNumSP[edgeAngleIn[inCount].first];
//      intersections[std::get<0>(vertex)].edge[totalCount] |=
//          (edgesData_[intersections[std::get<0>(vertex)].edge[totalCount]]
//               .numLines
//           << 24); // put the number of lines in each edge
//      intersections[std::get<0>(vertex)].edge[totalCount] |=
//          kMaskInEdge; // 0x800000 mask to define in edge
//      inCount++;
//    }
//
//    totalCount++;
//  }
//  // std::cout << "outCount = " << outCount << " inCount = " << inCount <<
//  // "\n";
//
//  if (totalCount != intersections[std::get<0>(vertex)].totalInOutEdges) {
//    printf("Error totalCount!=intersections[vertex].totalInOutEdges %d
//               % d\n ",
//               totalCount,
//           intersections[std::get<0>(vertex)].totalInOutEdges);
//  }
//}
//} // namespace LC

//  void Lanemap::resetIntersections(std::vector<B18IntersectionData> &
//                                       intersections,
//                                   std::vector<uchar> & trafficLights) {
//    for (int i = 0; i < intersections.size(); i++) {
//      intersections[i].nextEvent =
//          0.0f; // otherwise they not change until reach time again
//      intersections[i].state = 0; // to make the system to repeat same
//      execution
//    }
//
//    if (trafficLights.size() > 0) {
//      memset(trafficLights.data(), 0,
//             trafficLights.size() *
//                 sizeof(uchar)); // to make the system to repeat same
//                 execution
//    }
//  }

//

} // namespace LC
