
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

#include "agent.h"
#include "cuda_simulator.h"
#include "lanemap.h"
#include "network.h"
#include "od.h"
#include "pandana_ch/accessibility.h"
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

  void simulateInGPU(float start_time, float end_time,int save_interval);

  //! save edge data
  void save_edges(int current_time);
  //! save agent data
  void save_agents(int current_time);

  //  // pollution
  //  B18GridPollution gridPollution;

private:
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
