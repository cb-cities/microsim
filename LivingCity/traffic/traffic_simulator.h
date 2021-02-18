
#ifndef LC_B18_TRAFFIC_SIMULATOR_H
#define LC_B18_TRAFFIC_SIMULATOR_H

#include <qt5/QtCore/QSettings>
#include <qt5/QtCore/qcoreapplication.h>
#include <thread>
#include <unistd.h>

#include "network.h"
#include "agent.h"
#include "src/benchmarker.h"


namespace LC {

class LCUrbanMain;

class TrafficSimulator {

public:
  TrafficSimulator(RoadGraph *geoRoadGraph, const IDMParameters &simParameters);
  ~TrafficSimulator();

  void reset_agent();

  void load_agents(const std::shared_ptr<Network> &network);


  const std::vector<Agent> & agents() const {return agents_;}
//  float deltaTime;
//  int threadNumber;
//  float avgTravelTime;
//
//  // PM
//  B18TrafficLaneMap b18TrafficLaneMap;
//
//  void simulateInGPU(int numOfPasses, float startTimeH, float endTimeH,
//                     bool useJohnsonRouting, bool useSP,
//                     const std::shared_ptr<abm::Graph> &graph_,
//                     std::vector<abm::graph::edge_id_t> paths_SP,
//                     const parameters &simParameters);
//
//  // Lanes
//  std::vector<uint> edgeIdToLaneMapNum;
//  std::vector<uchar> laneMap;
//  std::vector<B18EdgeData> edgesData;
//  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> edgeDescToLaneMapNum;
//  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> laneMapNumToEdgeDesc;
//  std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDescSP;
//  std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNumSP;
//  void createLaneMap();
//  void createLaneMapSP(const std::shared_ptr<abm::Graph> &graph_);
//
//  // car path
//  void generateCarPaths(bool useJohnsonRouting);
//
//  // People
//
//  std::vector<uint> indexPathVec;
//
//  void resetPeopleJobANDintersections();
//  void saveODToFile(){}; // TODO
//  void loadODFromFile(){};
//
//  // Traffic lights
//  std::vector<uchar> trafficLights;
//  std::vector<B18IntersectionData> intersections;
//
//  // measurements
//  std::vector<float> accSpeedPerLinePerTimeInterval;
//  std::vector<float> numVehPerLinePerTimeInterval;
//
//  void save_edges(const std::vector<std::vector<unsigned>> &edge_upstream_count,
//                  std::vector<std::vector<unsigned>> &edge_downstream_count);
//
//  void savePeopleAndRoutesSP(int numOfPass,
//                             const std::shared_ptr<abm::Graph> &graph_,
//                             const std::vector<abm::graph::edge_id_t> &paths_SP,
//                             int start_time, int end_time);
//
//  // pollution
//  B18GridPollution gridPollution;

private:
  RoadGraph *simRoadGraph_;
  IDMParameters simParameters_;
  std::vector<Agent> agents_;
};
} // namespace LC

#endif // LC_B18_TRAFFIC_SIMULATOR_H
