#include "catch.hpp"
#include "traffic_simulator.h"
#include "b18CommandLineVersion.h"

using namespace LC;
TEST_CASE("CHECK SIMULATOR", "[SIMULATOR]") {
    const IDMParameters simParameters;
    ClientGeometry cg;
    std::string networkPath = "../tests/test_data/";
    std::string odFileName = "od.csv";
    auto network = std::make_shared<LC::Network>(networkPath, odFileName);
    std::string route_path = "../tests/test_data/all_paths_ch.txt";
    std::vector<abm::graph::edge_id_t> all_paths;

    std::ifstream inputFile(route_path);
    // test file open
    if (inputFile) {
        abm::graph::vertex_t value;
        // read the elements in the file into a vector
        while (inputFile >> value) {
            all_paths.push_back(value);
        }
    }
    network->map_person2init_edge (all_paths);

    TrafficSimulator simulator(&cg.roadGraph, simParameters,network);

    SECTION("Check loaded agents") {

        simulator.load_agents();

        auto agents = simulator.agents();
        REQUIRE(agents.size() == 4);

        auto a0 = agents.at(0);
        REQUIRE(a0.init_intersection == 0);
        REQUIRE(a0.end_intersection == 4);
        REQUIRE(a0.time_departure == 0);

        auto a1 = agents.at(1);
        REQUIRE(a1.init_intersection == 1);
        REQUIRE(a1.end_intersection == 4);
        REQUIRE(a1.time_departure == 300);

        auto a2 = agents.at(2);
        REQUIRE(a2.init_intersection == 1);
        REQUIRE(a2.end_intersection == 4);
        REQUIRE(a2.time_departure == 350);

        auto a3 = agents.at(3);
        REQUIRE(a3.init_intersection == 0);
        REQUIRE(a3.end_intersection == 4);
        REQUIRE(a3.time_departure == 400);

    }
    SECTION("Check simulation") {

        simulator.load_agents();
        simulator.simulateInGPU (all_paths);

    }

}
