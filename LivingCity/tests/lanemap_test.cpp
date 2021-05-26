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
  SECTION("Check paths") {
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
    lanemap.read_path(all_paths);
    auto indexPath = lanemap.indexPathVec();
    auto eid2index = lanemap.edgeIdToLaneMapNum();
    REQUIRE(indexPath.at(0) == eid2index.at(2));
    REQUIRE(indexPath.at(1) == eid2index.at(6));
    REQUIRE(indexPath.at(2) == -1);
    REQUIRE(indexPath.at(3) == eid2index.at(0));
    REQUIRE(indexPath.at(4) == eid2index.at(4));
    REQUIRE(indexPath.at(5) == -1);
  }
}