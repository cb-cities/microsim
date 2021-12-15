#include "catch.hpp"
#include "lanemap.h"
#include "network.h"
using namespace LC;

TEST_CASE("CHECK LANEMAP", "[lanemap]") {
  std::string networkPath = "../tests/test_data/";
  auto network = std::make_shared<Network>(networkPath);
  auto lanemap = Lanemap(network->street_graph());

  SECTION("Check edge data") {
    auto &edge_data = lanemap.edgesData();
    auto &eid2mid = lanemap.eid2mid();

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
    auto &lanemap_array = lanemap.lanemap_array();
    REQUIRE(lanemap_array.size() == 24 * 1024 * 2);
    REQUIRE(lanemap_array.at(399) == 0xFF);
  }

  SECTION("Check Intersections") {
    auto &intersections = lanemap.intersections();
    auto &eid2mid = lanemap.eid2mid();
    REQUIRE(intersections.size() == 5);

    auto &intersection2 = intersections[2];
    REQUIRE(intersection2.num_edge == 8);
    REQUIRE(intersection2.num_queue == 12);

    //    std::vector<unsigned int> dir_01 = {eid2mid.at(0),
    //                                        eid2mid.at(1)};
    //    std::cout<<intersection2.start_edge
    //    q_01->queue[q_01->q_ptr] = 1000;
    //    q_01->q_ptr++;
    //    auto &paird_q = intersection2.paired_queues.at(0);
    //    auto &q_01_vect = paird_q.at(0);
    //    REQUIRE(q_01_vect.queue[0] == 1000);
    //    REQUIRE(q_01_vect.q_ptr == 1);
  }

  //  SECTION("Check Intersections CUDA") {
  //    auto &intersections = lanemap.intersections();
  //    auto &edgeIdToLaneMapNum = lanemap.edgeIdToLaneMapNum();
  //    REQUIRE(intersections.size() == 5);
  //
  //    auto &intersection2 = intersections[2];
  //    for (unsigned i = 0; i < intersection2.num_queue; i++) {
  //      std::cout << intersection2.start_edge[i] << ";"
  //                << intersection2.end_edge[i] << ";" << std::endl;
  //    }
  //
  //    //    int lanemap_id0 = edgeIdToLaneMapNum[0];
  //    //    int lanemap_id1 = edgeIdToLaneMapNum[4];
  //    //    std::cout << lanemap_id0 << ";" << lanemap_id1 << ";" <<
  //    std::endl;
  //    //    int num_edge = intersection2.num_edge;
  //    //    int pos0, pos1;
  //    //    for (unsigned i = 0; i < num_edge; i++) {
  //    //      std::cout << intersection2.lanemap_id[i] << std::endl;
  //    //      if (lanemap_id0 == intersection2.lanemap_id[i]) {
  //    //        pos0 = i;
  //    //      }
  //    //      if (lanemap_id1 == intersection2.lanemap_id[i]) {
  //    //        pos1 = i;
  //    //      }
  //    //    }
  //    //    int base_idx = 0;
  //    //    if (pos0 > pos1) {
  //    //      base_idx += intersection2.num_queue;
  //    //      int temp = pos1;
  //    //      pos1 = pos0;
  //    //      pos0 = temp;
  //    //    }
  //    //    int idx = (pos0 * num_edge - (pos0 * (pos0 + 1)) / 2 + pos1 - pos0
  //    -
  //    //    1); std::cout << pos0 << ";" << pos1 << ";" << idx;
  //  }
}