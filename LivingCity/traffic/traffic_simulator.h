
#ifndef LC_B18_TRAFFIC_SIMULATOR_H
#define LC_B18_TRAFFIC_SIMULATOR_H

#include <boost/filesystem.hpp>
#include <qt5/QtCore/QSettings>
#include <qt5/QtCore/qcoreapplication.h>
#include <thread>
#include <unistd.h>

#include <QSettings>
#include <qt5/QtCore/qcoreapplication.h>
#include <string>

#include "Geometry/client_geometry.h"
#include "traffic/traffic_simulator.h"

#include "network.h"
#include "pandana_ch/accessibility.h"
#include "agent.h"
#include "lanemap.h"
#include "od.h"
#include "cuda_simulator.h"
#include "src/benchmarker.h"

namespace LC {

//! Traffic Simulator Class
//! \brief Class that assembles network, od to perform agent based car following
//! simulation
class TrafficSimulator {

public:
  // Constructor for the simulator class
  //! \param[in] network ptr to the network
  //! \param[in] od ptr to the ods
  //! \param[in] lanemap ptr to the lanemap
  //! \param[in] save_path path to save simulation results
  TrafficSimulator(std::shared_ptr<Network> network, std::shared_ptr<OD> od,
                   std::shared_ptr<Lanemap> lanemap,
                   const std::string &save_path = "./results/");

  ~TrafficSimulator() = default;

  void simulateInGPU(float start_time, float end_time);

  //
  void
  save_edges(const std::vector<std::vector<unsigned>> &edge_upstream_count,
             const std::vector<std::vector<unsigned>> &edge_downstream_count);

  void save_intersection(
      const std::vector<std::vector<unsigned>> &intersection_count);

  void savePeopleAndRoutesSP(int numOfPass,
                             const std::shared_ptr<abm::Graph> &graph_,
                             const std::vector<abm::graph::edge_id_t> &paths_SP,
                             int start_time, int end_time);

  //  // pollution
  //  B18GridPollution gridPollution;

private:
  void writePeopleFile(int numOfPass, const std::shared_ptr<abm::Graph> &graph_,
                       int start_time, int end_time,
                       const std::vector<Agent> &agents_, float deltaTime);
  void writeRouteFile(int numOfPass,
                      const std::vector<abm::graph::edge_id_t> &paths_SP,
                      int start_time, int end_time);
  void writeIndexPathVecFile(int numOfPass, int start_time, int end_time,
                             const std::vector<uint> &indexPathVec);

  //! Find shortest path for each agent
  void route_finding_();

  std::shared_ptr<Network> network_;
  std::shared_ptr<OD> od_;
  std::shared_ptr<Lanemap> lanemap_;
    //! simulation time resolution
  double deltaTime_ = 0.5;
  std::string save_path_ = "./";
};
} // namespace LC

#endif // LC_B18_TRAFFIC_SIMULATOR_H
