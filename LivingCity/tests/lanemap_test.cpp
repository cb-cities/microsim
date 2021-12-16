#include "catch.hpp"
#include "lanemap.h"
#include "network.h"
using namespace LC;

TEST_CASE("CHECK LANEMAP", "[lanemap]") {
  std::string networkPath = "../tests/test_data/";
  std::shared_ptr<Network> network = std::make_shared<Network>(networkPath);
  std::shared_ptr<Lanemap> lanemap =
      std::make_shared<LC::Lanemap>(network->street_graph());

  SECTION("Check edge data") {
    auto &edge_data = lanemap->edgesData();
    auto &eid2mid = lanemap->eid2mid();

    REQUIRE(edge_data.size() == 24);
    REQUIRE(eid2mid.size() == 12);

    auto i0 = eid2mid.at(0);
    auto e0 = edge_data.at(i0);
    REQUIRE(e0.length == 2000);
    //    //
    auto i6 = eid2mid.at(6);
    auto e6 = edge_data.at(i6);
    REQUIRE(e6.length == 1000);
  }
  SECTION("Check Lanemap") {
    auto &lanemap_array = lanemap->lanemap_array();
    REQUIRE(lanemap_array.size() == 24 * 1024 * 2);
    REQUIRE(lanemap_array.at(399) == 0xFF);
  }

  SECTION("Check Intersections") {
    auto &intersections = lanemap->intersections();
    auto &mid2eid = lanemap->mid2eid();
    REQUIRE(intersections.size() == 5);

    auto &intersection2 = intersections[2];
    REQUIRE(intersection2.num_edge == 2 * 4);
    REQUIRE(intersection2.num_queue == 3 * 4);
    std::cout << "start; end for intersection 2: " << std::endl;

    for (int j = 0; j < intersection2.num_queue; ++j) {
      auto start_mid = intersection2.start_edge[j];
      auto end_mid = intersection2.end_edge[j];
      std::cout << mid2eid.at(start_mid) << ";" << mid2eid.at(end_mid)
                << std::endl;
    }
  }

}