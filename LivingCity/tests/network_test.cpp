#include "catch.hpp"
#include "network.h"

TEST_CASE("CHECK THE NETWORK", "[NETWORK]") {
  double tolerance = 1e-6;
  std::string networkPath = "../tests/test_data/";
  auto network = std::make_shared<LC::Network>(networkPath);

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

    REQUIRE(edge_weights.at(0) ==  Approx(74.56472839797681).epsilon(tolerance) );
    REQUIRE(edge_weights.at(1) == Approx(74.56472839797681).epsilon(tolerance) );
    REQUIRE(edge_weights.at(2) == Approx(37.282364198988404).epsilon(tolerance) );
    REQUIRE(edge_weights.at(3) == Approx(18.641182099494202).epsilon(tolerance) );
    REQUIRE(edge_weights.at(4) == Approx(74.56472839797681).epsilon(tolerance) );
    REQUIRE(edge_weights.at(5) == Approx(149.12945679595362).epsilon(tolerance) );

    REQUIRE(network->edge_id(0, 2) == 0);
    REQUIRE(network->edge_id(1, 2) == 1);
    REQUIRE(network->edge_id(0, 3) == 2);
    REQUIRE(network->edge_id(2, 3) == 3);
    REQUIRE(network->edge_id(2, 4) == 4);
    REQUIRE(network->edge_id(3, 4) == 5);
  }
}
