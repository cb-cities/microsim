#include "catch.hpp"
#include "lanemap.h"
#include "network.h"

using namespace LC;

TEST_CASE("CHECK LANEMAP", "[lanemap]") {
  std::string networkPath = "../tests/test_data/";
  std::string odFileName = "od.csv";
  auto network = std::make_shared<Network>(networkPath, odFileName);
  auto graph = network->street_graph();
  auto lanemap = Lanemap(graph);

  SECTION("Check edge data") {
    auto &edge_data = lanemap.edgesData();
    auto &edgeIdToLaneMapNum = lanemap.edgeIdToLaneMapNum();

    REQUIRE(edge_data.size() == 9);
    REQUIRE(edgeIdToLaneMapNum.size() == 6 + 1);

    auto i0 = edgeIdToLaneMapNum.at(0);
    auto e0 = edge_data.at(i0);
    REQUIRE(e0.length == 50);
    REQUIRE(e0.nextInters == 2);

    auto i6 = edgeIdToLaneMapNum.at(6);
    auto e6 = edge_data.at(i6);
    REQUIRE(e6.length == 400);
    REQUIRE(e6.nextInters == 4);
  }
  SECTION("Check Lanemap") {
    auto &lanemap_array = lanemap.lanemap_array();
    REQUIRE(lanemap_array.size() == 9 * 1024 * 2);
    REQUIRE(lanemap_array.at(399) == 0xFF);
  }
  SECTION("Check intersections") {
    auto &interections = lanemap.intersections();
    REQUIRE(interections.size() == 5);
  }
}