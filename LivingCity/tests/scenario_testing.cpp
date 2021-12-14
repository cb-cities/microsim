#include "catch.hpp"
#include "traffic_simulator.h"
#include "simulation_interface.h"

using namespace LC;
TEST_CASE("Scenario Testing", "[SIMULATOR]") {
    SECTION("Case1") {
        const IDMParameters simParameters;
        ClientGeometry cg;
        std::string networkPath = "../tests/scenarios/case1/";
        std::string odFileName = "od.csv";
        auto network = std::make_shared<LC::Network>(networkPath, odFileName);


        std::vector<abm::graph::edge_id_t> all_paths;
        std::vector<std::vector<int>> all_paths_ch;

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

//        std::string route_path = "../tests/test_data/all_paths_ch.txt";
//        std::vector<abm::graph::edge_id_t> all_paths;
//
//        std::ifstream inputFile(route_path);
//        // test file open
//        if (inputFile) {
//            abm::graph::vertex_t value;
//            // read the elements in the file into a vector
//            while (inputFile >> value) {
//                all_paths.push_back(value);
//            }
//        }
        network->map_person2init_edge (all_paths);
        TrafficSimulator simulator(&cg.roadGraph, simParameters,network,"./case1_results/");

        simulator.load_agents();
        simulator.simulateInGPU (all_paths,0,240);
    }

}
