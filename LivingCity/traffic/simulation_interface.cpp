#include "simulation_interface.h"
#include <chrono>
namespace LC {

// using namespace std::chrono;

void SimulationInterface::run_simulation() {
  /************************************************************************************************
    Parse Simulation parameters
  ************************************************************************************************/
  QSettings settings("./command_line_options.ini", QSettings::IniFormat);
  std::string networkPath =
      settings.value("NETWORK_PATH", "../berkeley_2018/new_full_network/")
          .toString()
          .toStdString();
  std::string save_path =
      settings.value("SAVE_PATH", "./results/").toString().toStdString();

  const float start = settings.value("START", 5 * 3600).toFloat();
  const float end = settings.value("END", 12 * 3600).toFloat();
  const bool showBenchmarks = settings.value("SHOW_BENCHMARKS", false).toBool();
  const int save_interval = settings.value("SAVE_INTERVAL", 100).toInt();
  std::string od_path =
      settings
          .value("OD_PATH",
                 "../berkeley_2018/new_full_network/od_demand_5to12.csv")
          .toString()
          .toStdString();

  // new benchmarks
  if (showBenchmarks) {
    Benchmarker::enableShowBenchmarks();
  }
  Benchmarker loadNetwork("Load_network", true);
  Benchmarker loadODDemandData("Load_OD_demand_data", true);
  Benchmarker routingCH("Routing_CH", true);
  Benchmarker CHoutputNodesToEdgesConversion(
      "CH_output_nodes_to_edges_conversion", true);
  Benchmarker initBench("Initialize traffic simulator task");
  Benchmarker peopleBench("People creation task");
  Benchmarker simulationBench("Simulation task");

  /************************************************************************************************
    Network Building
  ************************************************************************************************/
  std::shared_ptr<Network> network = std::make_shared<LC::Network>(networkPath);
  std::shared_ptr<OD> od = std::make_shared<LC::OD>(od_path);
  std::shared_ptr<Lanemap> lanemap =
      std::make_shared<LC::Lanemap>(network->street_graph());

  /************************************************************************************************
    Start Simulation
  ************************************************************************************************/
  TrafficSimulator simulator(network, od, lanemap, "./test_results/");
  simulator.simulateInGPU(start, end, save_interval);
}
} // namespace LC
