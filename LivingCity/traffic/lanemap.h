#ifndef LC_B18_TRAFFIC_LANEMAP_H
#define LC_B18_TRAFFIC_LANEMAP_H

#include "../misctools/misctools.h"
#include "RoadGraph/roadGraph.h"
#include "b18EdgeData.h"
#include "config.h"
#include "traffic/sp/graph.h"
#include <QVector3D>

namespace LC {

class Lanemap {
public:
  Lanemap() = default;

  ~Lanemap() = default;

  explicit Lanemap(const std::shared_ptr<abm::Graph> &graph) {
    create_edgesData_(graph);
    create_LaneMap_();
    create_intersections_ (graph);
  };

  void resetIntersections(std::vector<B18IntersectionData> &intersections,
                          std::vector<uchar> &trafficLights);

  const std::vector<B18EdgeData> &edgesData() const { return edgesData_; }

  const std::map<uint, std::shared_ptr<abm::Graph::Edge>> &
  laneMapNumToEdgeDesc() const {
    return laneMapNumToEdgeDesc_;
  }

  const std::map<std::shared_ptr<abm::Graph::Edge>, uint> &
  edgeDescToLaneMapNum() const {
    return edgeDescToLaneMapNum_;
  }

  const std::vector<uint> &edgeIdToLaneMapNum() const {
    return edgeIdToLaneMapNum_;
  }

  const std::vector<uchar> &lanemap_array() const { return laneMap_; }

  const std::vector<B18IntersectionData> &intersections() const {
    return intersections_;
  }

private:
  std::vector<uchar> laneMap_;
  std::vector<B18EdgeData> edgesData_;
  std::vector<B18IntersectionData> intersections_;
    std::vector<uchar> traffic_lights_;

  std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDesc_;
  std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNum_;
  std::vector<uint> edgeIdToLaneMapNum_;
  int tNumMapWidth_{0};
  int kMaxMapWidthM_{1024}; // size of the bin

  void create_edgesData_(const std::shared_ptr<abm::Graph> &graph);
  void create_LaneMap_();
  void create_intersections_(const std::shared_ptr<abm::Graph> &graph);
};
} // namespace LC

#endif // LC_B18_TRAFFIC_LANEMAP_H
