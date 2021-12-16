#include "catch.hpp"
#include "traffic_simulator.h"

using namespace LC;
TEST_CASE("CHECK SIMULATOR", "[SIMULATOR]") {
  std::string networkPath = "../tests/test_data/";
  std::string odFileName = "../tests/test_data/od.csv";

  std::shared_ptr<Network> network = std::make_shared<LC::Network>(networkPath);
  std::shared_ptr<OD> od = std::make_shared<LC::OD>(odFileName);
  std::shared_ptr<Lanemap> lanemap =
      std::make_shared<LC::Lanemap>(network->street_graph());
  TrafficSimulator simulator(network, od, lanemap, "./test_results/");
  //
  SECTION("Check loaded agents") {
    auto &agents = od->agents();
    auto &mid2eid = lanemap->mid2eid();
    std::cout << "Print Shortest Path" << std::endl;
    std::cout << "========================================" << std::endl;

    for (const auto &agent : agents) {
      std::cout << "Start; End " << agent.init_intersection << ";"
                << agent.end_intersection << std::endl;
      std::cout << "Current ptr " << agent.route_ptr << std::endl;
      std::cout << "number of passing edges: " << agent.route_size << std::endl;
      for (int j = 0; j < agent.route_size; ++j) {
        std::cout << mid2eid.at(agent.route[j]) << ";";
      }
      std::cout << std::endl;
      std::cout << "========================================" << std::endl;
    }

    REQUIRE(agents[0].route_size == 3);
    REQUIRE(mid2eid.at(agents[0].route[0]) == 4);
    REQUIRE(mid2eid.at(agents[0].route[1]) == 7);
    REQUIRE(mid2eid.at(agents[0].route[2]) == 8);

    REQUIRE(agents[1].route_size == 2);
    REQUIRE(mid2eid.at(agents[1].route[0]) == 2);
    REQUIRE(mid2eid.at(agents[1].route[1]) == 8);
  }
  //
  //        simulator.load_agents();
  //
  //        auto agents = simulator.agents();
  //        REQUIRE(agents.size() == 4);
  //
  //        auto a0 = agents.at(0);
  //        REQUIRE(a0.init_intersection == 0);
  //        REQUIRE(a0.end_intersection == 4);
  //        REQUIRE(a0.time_departure == 0);
  //
  //        auto a1 = agents.at(1);
  //        REQUIRE(a1.init_intersection == 1);
  //        REQUIRE(a1.end_intersection == 4);
  //        REQUIRE(a1.time_departure == 300);
  //
  //        auto a2 = agents.at(2);
  //        REQUIRE(a2.init_intersection == 1);
  //        REQUIRE(a2.end_intersection == 4);
  //        REQUIRE(a2.time_departure == 350);
  //
  //        auto a3 = agents.at(3);
  //        REQUIRE(a3.init_intersection == 0);
  //        REQUIRE(a3.end_intersection == 4);
  //        REQUIRE(a3.time_departure == 400);
  //
  //    }
  //    SECTION("Check simulation") {
  //        simulator.load_agents();
  //        simulator.simulateInGPU (all_paths,0,600);
  //    }
}
