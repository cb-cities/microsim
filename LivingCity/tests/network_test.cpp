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
    REQUIRE(network->num_vertices() == 5);
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

  SECTION("Check graph edges") {
    REQUIRE(network->num_edges() == 6);
    auto edge_weights = network->edge_weights().at(0);

    REQUIRE(edge_weights.at(0) == 50);
    REQUIRE(edge_weights.at(1) == 100);
    REQUIRE(edge_weights.at(2) == 200);
    REQUIRE(edge_weights.at(3) == 300);
    REQUIRE(edge_weights.at(4) == 400);
    REQUIRE(edge_weights.at(5) == 500);

    REQUIRE(network->edge_id(0, 2) == 0);
    REQUIRE(network->edge_id(1, 2) == 1);
    REQUIRE(network->edge_id(0, 3) == 2);
    REQUIRE(network->edge_id(2, 3) == 3);
    REQUIRE(network->edge_id(2, 4) == 4);
    REQUIRE(network->edge_id(3, 4) == 5);
  }

  SECTION("Check loaded ods") {
    auto od_pairs = network->od_pairs();
    auto dep_times = network->dep_time();

    REQUIRE(network->totalNumPeople() == 4);

    REQUIRE(od_pairs.at(0)[0] == 0);
    REQUIRE(od_pairs.at(0)[1] == 4);

    REQUIRE(od_pairs.at(1)[0] == 1);
    REQUIRE(od_pairs.at(1)[1] == 4);

    REQUIRE(od_pairs.at(2)[0] == 1);
    REQUIRE(od_pairs.at(2)[1] == 4);

    REQUIRE(od_pairs.at(3)[0] == 0);
    REQUIRE(od_pairs.at(3)[1] == 4);

    REQUIRE(dep_times.at(0) == 0);
    REQUIRE(dep_times.at(1) == 300);
    REQUIRE(dep_times.at(2) == 350);
    REQUIRE(dep_times.at(3) == 400);
  }
}
