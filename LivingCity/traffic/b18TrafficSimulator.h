/************************************************************************************************
 *		@desc Class that contains the traffic simulator b2018.
 *		@author igaciad
 ************************************************************************************************/

#ifndef LC_B18_TRAFFIC_SIMULATOR_H
#define LC_B18_TRAFFIC_SIMULATOR_H

#include <qt5/QtCore/QSettings>
#include <qt5/QtCore/qcoreapplication.h>

#include "b18GridPollution.h"
#include "b18TrafficLaneMap.h"
#include "b18TrafficOD.h"
#include "misctools/misctools.h"
#include "roadGraphB2018Loader.h"

namespace LC {

class LCUrbanMain;

class B18TrafficSimulator {

public:
  B18TrafficSimulator(float deltaTime, RoadGraph *geoRoadGraph,
                      const parameters &simParameters);
  ~B18TrafficSimulator();

  // init data
  RoadGraph *simRoadGraph;
  parameters simParameters;

  float deltaTime;
  int threadNumber;
  float avgTravelTime;

  // PM
  B18TrafficOD b18TrafficOD;
  B18TrafficLaneMap b18TrafficLaneMap;

  void simulateInGPU(int numOfPasses, float startTimeH, float endTimeH,
                     bool useJohnsonRouting, bool useSP,
                     const std::shared_ptr<abm::Graph> &graph_,
                     std::vector<abm::graph::edge_id_t> paths_SP,
                     const parameters &simParameters);

  // Lanes
  std::vector<uint> edgeIdToLaneMapNum;
  std::vector<uchar> laneMap;
  std::vector<B18EdgeData> edgesData;
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> edgeDescToLaneMapNum;
  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> laneMapNumToEdgeDesc;
  std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDescSP;
  std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNumSP;
  void createLaneMap();
  void createLaneMapSP(const std::shared_ptr<abm::Graph> &graph_);

  // car path
  void generateCarPaths(bool useJohnsonRouting);

  // People
  std::vector<B18TrafficPerson> trafficPersonVec;
  std::vector<uint> indexPathVec;

  void createB2018People(float startTime, float endTime, int limitNumPeople,
                         bool addRandomPeople, bool useSP);

  void createB2018PeopleSP(float startTime, float endTime,
                           const std::shared_ptr<RoadGraphB2018> graph_loader,
                           std::vector<float> dep_times);

  void resetPeopleJobANDintersections();
  void saveODToFile(){}; // TODO
  void loadODFromFile(){};

  // Traffic lights
  std::vector<uchar> trafficLights;
  std::vector<B18IntersectionData> intersections;

  // measurements
  std::vector<float> accSpeedPerLinePerTimeInterval;
  std::vector<float> numVehPerLinePerTimeInterval;


  void save_edges(const std::vector<std::vector<unsigned>> &edge_upstream_count,
                  std::vector<std::vector<unsigned>> &edge_downstream_count);

  void savePeopleAndRoutesSP(int numOfPass,
                             const std::shared_ptr<abm::Graph> &graph_,
                             const std::vector<abm::graph::edge_id_t> &paths_SP,
                             int start_time, int end_time);

  // pollution
  B18GridPollution gridPollution;
};
} // namespace LC

#endif // LC_B18_TRAFFIC_SIMULATOR_H
