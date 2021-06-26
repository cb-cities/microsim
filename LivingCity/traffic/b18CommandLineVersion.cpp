#include "b18CommandLineVersion.h"
#include <chrono>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/customizable_contraction_hierarchy.h>

namespace LC {

// using namespace std::chrono;

void B18CommandLineVersion::runB18Simulation() {
  QSettings settings("../command_line_options.ini", QSettings::IniFormat);
  bool usePrevPaths = settings.value("USE_PREV_PATHS", false).toBool();
  auto networkPath =
      settings.value("NETWORK_PATH", "../berkeley_2018/new_full_network/")
          .toString();
  std::string networkPathSP = networkPath.toStdString();
  const float start = settings.value("START", 5 * 3600).toFloat();
  const float end = settings.value("END", 12 * 3600).toFloat();
  const bool showBenchmarks = settings.value("SHOW_BENCHMARKS", false).toBool();
  const IDMParameters simParameters;

  std::string odDemandPath =
      settings.value("OD_DEMAND_FILENAME", "od_demand_5to12.csv")
          .toString()
          .toStdString();

  std::cout << "b18CommandLineVersion received the parameters "
            << "[a: " << simParameters.a << ", b: " << simParameters.b
            << ", T: " << simParameters.T << ", s_0: " << simParameters.s_0
            << "]" << std::endl;

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
  // make the graph from edges file and load the OD demand from od file
  loadNetwork.startMeasuring();
  auto network = std::make_shared<Network>(networkPathSP, odDemandPath);
  loadNetwork.stopAndEndBenchmark();

  /************************************************************************************************
    Route Finding (OLD)
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
    // compute routes use ch
    routingCH.startMeasuring();
    auto graph_ch = std::make_shared<MTC::accessibility::Accessibility>(
        network->num_vertices(), network->edge_vertices(),
        network->edge_weights(), false);

    std::vector<long> sources, targets;
    for (const auto &od : network->od_pairs()) {
      sources.emplace_back(od[0]);
      targets.emplace_back(od[1]);
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
        auto one_edge = network->edge_id(vertex_from, vertex_to);
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
  network->map_person2init_edge(all_paths);

  //  /************************************************************************************************
  //    Route Finding (NEW)
  //  ************************************************************************************************/
  //  std::vector<abm::graph::edge_id_t> all_paths;
  //
  //  // compute routes use ch
  //  routingCH.startMeasuring();
  //
  //  auto t1 = chrono::high_resolution_clock::now();
  //  RoutingKit::ContractionHierarchy ch =
  //  RoutingKit::ContractionHierarchy::build(
  //      network->num_vertices(), network->heads(), network->tails(),
  //      network->edge_weights_kit());
  //  auto t2 = chrono::high_resolution_clock::now();
  //  auto diff = t2 - t1;
  //  std::cout << chrono::duration<double, milli>(diff).count() << " ms" <<
  //  endl; routingCH.stopAndEndBenchmark();
  //
  //  routingCH.startMeasuring();
  //  auto t3 = chrono::high_resolution_clock::now();
  //  RoutingKit::CustomizableContractionHierarchy cch(ch.order,
  //  network->heads(),
  //                                                   network->tails());
  //  RoutingKit::CustomizableContractionHierarchyMetric metric(
  //      cch, network->edge_weights_kit());
  //  //    metric.customize();
  //  RoutingKit::CustomizableContractionHierarchyParallelization
  //      parallel_customization(cch);
  //  parallel_customization.customize(metric);
  //  auto t4 = chrono::high_resolution_clock::now();
  //  auto diff2 = t4 - t3;
  //  std::cout << chrono::duration<double, milli>(diff2).count() << " ms" <<
  //  endl; routingCH.stopAndEndBenchmark();
  //
  //  auto t5 = chrono::high_resolution_clock::now();
  //  RoutingKit::ContractionHierarchy ch2 =
  //      metric.build_contraction_hierarchy_using_perfect_witness_search();
  //  auto t6 = chrono::high_resolution_clock::now();
  //  auto diff3 = t6 - t5;
  //  std::cout << chrono::duration<double, milli>(diff3).count() << " ms" <<
  //  endl;

  //    routingCH.startMeasuring();
  //    RoutingKit::ContractionHierarchyQuery query(ch);
  //    std::vector<std::vector<unsigned int>> all_paths_ch;
  //
  //    for (const auto &od : network->od_pairs()) {
  //      query.reset().add_source(od[0]).add_target(od[1]).run();
  //      auto path = query.get_node_path();
  //      all_paths_ch.emplace_back(path);
  //    }
  //    std::cout << "# of paths = " << all_paths_ch.size() << " \n";
  //    routingCH.stopAndEndBenchmark();
  //
  //    CHoutputNodesToEdgesConversion.startMeasuring();
  //    std::cout << all_paths_ch.size() << "; " << all_paths_ch[0].size()
  //              << std::endl;
  //
  //    // convert from nodes to edges
  //    for (int i = 0; i < all_paths_ch.size(); i++) {
  //      for (int j = 0; j < all_paths_ch[i].size() - 1; j++) {
  //        auto vertex_from = all_paths_ch[i][j];
  //        auto vertex_to = all_paths_ch[i][j + 1];
  //        auto one_edge = network->edge_id(vertex_from, vertex_to);
  //        all_paths.emplace_back(one_edge);
  //      }
  //      all_paths.emplace_back(-1);
  //    }
  //    CHoutputNodesToEdgesConversion.stopAndEndBenchmark();
  //
  //  //  // write paths to file so that we can just load them instead
  //  //  const std::string &pathsFileName = networkPathSP + "all_paths_ch.txt";
  //  //  std::cout << "Save " << pathsFileName << " as paths file\n";
  //  //  std::ofstream output_file(pathsFileName);
  //  //  std::ostream_iterator<abm::graph::vertex_t>
  //  output_iterator(output_file,
  //  //                                                              "\n");
  //  //  std::copy(all_paths.begin(), all_paths.end(), output_iterator);
  //    // map person to their initial edge
  //    network->map_person2init_edge(all_paths);

  /************************************************************************************************
    Start Simulation
  ************************************************************************************************/

  ClientGeometry cg;
  TrafficSimulator simulator(&cg.roadGraph, simParameters, network,
                             "./case1_results/");

  simulator.load_agents();
  simulator.simulateInGPU(all_paths, start, end);
}
} // namespace LC
