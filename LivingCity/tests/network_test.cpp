#include "catch.hpp"
#include "network.h"

TEST_CASE("CHECK NETWORK", "[IO]") {
  double tolerance = 1e-6;
  std::string networkPath = "../tests/test_data/";
  std::string odFileName = "od.csv";

  auto network = std::make_shared<LC::Network>(networkPath, odFileName);

  SECTION("Check agents") {
    REQUIRE(network->totalNumPeople() == 4);

    auto od_pairs = network->od_pairs();

    auto od0 = od_pairs.at(0);
    REQUIRE(od0.at(0) == 0);
    REQUIRE(od0.at(1) == 4);

    auto od1 = od_pairs.at(1);
    REQUIRE(od1.at(0) == 1);
    REQUIRE(od1.at(1) == 4);

    auto od3 = od_pairs.at(3);
    REQUIRE(od3.at(0) == 0);
    REQUIRE(od3.at(1) == 4);
  }

  SECTION("Check graph vertices") {
    auto edge_vals = network->edge_vertices();

    auto edges_val0 = edge_vals.at(0);
    REQUIRE(edges_val0.at(0) == 0);
    REQUIRE(edges_val0.at(1) == 2);

    auto edges_val1 = edge_vals.at(1);
    REQUIRE(edges_val1.at(0) == 1);
    REQUIRE(edges_val1.at(1) == 2);

    auto edges_val2 = edge_vals.at(2);
    REQUIRE(edges_val2.at(0) == 0);
    REQUIRE(edges_val2.at(1) == 3);

    auto edges_val3 = edge_vals.at(3);
    REQUIRE(edges_val3.at(0) == 2);
    REQUIRE(edges_val3.at(1) == 3);

    auto edges_val4 = edge_vals.at(4);
    REQUIRE(edges_val4.at(0) == 2);
    REQUIRE(edges_val4.at(1) == 4);

    auto edges_val5 = edge_vals.at(5);
    REQUIRE(edges_val5.at(0) == 3);
    REQUIRE(edges_val5.at(1) == 4);
  }

  SECTION("Check graph weights") {
    auto edge_weights = network->edge_weights().at(0);

    REQUIRE(edge_weights.at(0) == 50);
    REQUIRE(edge_weights.at(1) == 100);
    REQUIRE(edge_weights.at(2) == 200);
    REQUIRE(edge_weights.at(3) == 300);
    REQUIRE(edge_weights.at(4) == 400);
    REQUIRE(edge_weights.at(5) == 500);
  }
}
