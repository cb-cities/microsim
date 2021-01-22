#include "b18CommandLineVersion.h"

namespace LC {

// using namespace std::chrono;

void B18CommandLineVersion::runB18Simulation() {
  QSettings settings("../command_line_options.ini", QSettings::IniFormat);
  bool useJohnsonRouting = false;
  bool useSP = settings.value("USE_SP_ROUTING", true).toBool();
  bool usePrevPaths = settings.value("USE_PREV_PATHS", false).toBool();

  auto networkPath =
      settings.value("NETWORK_PATH", "../berkeley_2018/new_full_network/")
          .toString();
  std::string networkPathSP = networkPath.toStdString();

  int numOfPasses = settings.value("NUM_PASSES", 1).toInt();
  const float deltaTime = settings.value("TIME_STEP", .5).toFloat();
  const float startDemandH = settings.value("START_HR", 5).toFloat();
  const float endDemandH = settings.value("END_HR", 12).toFloat();
  const bool showBenchmarks = settings.value("SHOW_BENCHMARKS", false).toBool();
  const parameters simParameters{
      settings.value("a", 0.557040909258405).toDouble(),
      settings.value("b", 2.9020578588167).toDouble(),
      settings.value("T", 0.5433027817144876).toDouble(),
      settings.value("s_0", 1.3807498735425845).toDouble()};

  std::string odDemandPath =
      settings.value("OD_DEMAND_FILENAME", "od_demand_5to12.csv")
          .toString()
          .toStdString();

  std::cout << "b18CommandLineVersion received the parameters "
            << "[a: " << simParameters.a << ", b: " << simParameters.b
            << ", T: " << simParameters.T << ", s_0: " << simParameters.s_0
            << "]" << std::endl;

  float startSimulationH = startDemandH;
  float endSimulationH = endDemandH;

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
    Network loading
  ************************************************************************************************/
  // make the graph from edges file and load the OD demand from od file
  loadNetwork.startMeasuring();
  auto graph_loader =
      std::make_shared<RoadGraphB2018>(networkPathSP, odDemandPath);
  graph_loader->loadABMGraph();
  loadNetwork.stopAndEndBenchmark();

  loadODDemandData.startMeasuring();
  const auto all_od_pairs_ = graph_loader->read_od_pairs();
  const auto dep_times = graph_loader->read_dep_times();
  auto street_graph = graph_loader->street_graph();
  printf("# of OD pairs = %d\n", all_od_pairs_.size());
  loadODDemandData.stopAndEndBenchmark();

  /************************************************************************************************
    Route Finding
  ************************************************************************************************/
  std::vector<abm::graph::edge_id_t> all_paths;
  std::vector<std::vector<int>> all_paths_ch;
  if (usePrevPaths) {
    // open file
    const std::string &pathsFileName = networkPathSP + "all_paths_ch.txt";
    std::cout << "Loading " << pathsFileName << " as paths file\n";
    std::ifstream inputFile(pathsFileName);
    // test file open
    if (inputFile) {
      abm::graph::vertex_t value;
      // read the elements in the file into a vector
      while (inputFile >> value) {
        all_paths.push_back(value);
      }
    }
  } else {
    std::vector<std::vector<long>> edge_vals = graph_loader->edge_vals();
    std::vector<std::vector<double>> edge_weights =
        graph_loader->edge_weights();

    // compute routes use ch
    routingCH.startMeasuring();
    auto graph_ch = std::make_shared<MTC::accessibility::Accessibility>(
        street_graph->vertices_data_.size(), edge_vals, edge_weights, false);

    std::vector<long> sources, targets;
    for (int x = 0; x < all_od_pairs_.size(); x++) {
      sources.emplace_back(all_od_pairs_[x][0]);
      targets.emplace_back(all_od_pairs_[x][1]);
    }
    all_paths_ch = graph_ch->Routes(sources, targets, 0);
    std::cout << "# of paths = " << all_paths_ch.size() << " \n";
    routingCH.stopAndEndBenchmark();

    CHoutputNodesToEdgesConversion.startMeasuring();
    std::cout << all_paths_ch.size() << "; " << all_paths_ch[0].size()
              << std::endl;
    std::cout << all_paths_ch.size() << "; " << all_paths_ch[10].size()
              << std::endl;
    // convert from nodes to edges
    for (int i = 0; i < all_paths_ch.size(); i++) {
      for (int j = 0; j < all_paths_ch[i].size() - 1; j++) {
        auto vertex_from = all_paths_ch[i][j];
        auto vertex_to = all_paths_ch[i][j + 1];
        auto one_edge = street_graph->edge_ids_[vertex_from][vertex_to];
        all_paths.emplace_back(one_edge);
      }
      all_paths.emplace_back(-1);
    }
    CHoutputNodesToEdgesConversion.stopAndEndBenchmark();

    // write paths to file so that we can just load them instead
    const std::string &pathsFileName = networkPathSP + "all_paths_ch.txt";
    std::cout << "Save " << pathsFileName << " as paths file\n";
    std::ofstream output_file(pathsFileName);
    std::ostream_iterator<abm::graph::vertex_t> output_iterator(output_file,
                                                                "\n");
    std::copy(all_paths.begin(), all_paths.end(), output_iterator);
  }
  // map person to their initial edge
  graph_loader->map_person2init_edge (all_paths);

  /************************************************************************************************
    Start Simulation
  ************************************************************************************************/

  ClientGeometry cg;
  B18TrafficSimulator b18TrafficSimulator(deltaTime, &cg.roadGraph,
                                          simParameters);

  // create a set of people for simulation (trafficPersonVec)
  b18TrafficSimulator.createB2018PeopleSP(startDemandH, endDemandH,
                                          graph_loader, dep_times);
  //
  // if useSP, convert all_paths to indexPathVec format and run simulation
  b18TrafficSimulator.simulateInGPU(numOfPasses, startSimulationH,
                                    endSimulationH, useJohnsonRouting, useSP,
                                    street_graph, all_paths, simParameters);
}

} // namespace LC
