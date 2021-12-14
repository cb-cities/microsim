#include "catch.hpp"
#include "od.h"
#include <memory>

TEST_CASE("CHECK THE OD", "[OD]") {
  double tolerance = 1e-6;
  std::string odFileName = "../tests/test_data/od.csv";
  auto od = std::make_shared<LC::OD>(odFileName);

  SECTION("Check agents") {
    REQUIRE(od->num_agents() == 4);

    auto agents = od->agents();

    auto agent0 = agents.at(0);
    REQUIRE(agent0.agent_type == LC::CAR);
    REQUIRE(agent0.init_intersection == 0);
    REQUIRE(agent0.end_intersection == 4);
    REQUIRE(agent0.time_departure == 0);

    auto agent1 = agents.at(1);
    REQUIRE(agent1.agent_type == LC::CAR);
    REQUIRE(agent1.init_intersection == 1);
    REQUIRE(agent1.end_intersection == 4);
    REQUIRE(agent1.time_departure == 300);

    auto agent2 = agents.at(2);
    REQUIRE(agent2.agent_type == LC::CAR);
    REQUIRE(agent2.init_intersection == 1);
    REQUIRE(agent2.end_intersection == 4);
    REQUIRE(agent2.time_departure == 350);

    auto agent3 = agents.at(3);
    REQUIRE(agent3.agent_type == LC::CAR);
    REQUIRE(agent3.init_intersection == 0);
    REQUIRE(agent3.end_intersection == 4);
    REQUIRE(agent3.time_departure == 400);
  }
}