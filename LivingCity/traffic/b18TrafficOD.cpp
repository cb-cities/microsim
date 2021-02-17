#include "b18TrafficOD.h"

#include "../misctools/misctools.h"
#include "network.h"

#include <boost/math/distributions/non_central_t.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <qt5/QtCore/QTime>

#define ROUTE_DEBUG 0
#define PERSON_DEBUG 0

namespace LC {

B18TrafficOD::B18TrafficOD(const parameters &inputSimParameters)
    : simParameters(inputSimParameters) {} //
B18TrafficOD::~B18TrafficOD() {}           //

void B18TrafficOD::randomPerson(int p, B18TrafficPerson &person, uint srcvertex,
                                uint tgtvertex, float startTimeH) {

  // Data
  person.init_intersection = srcvertex;
  person.end_intersection = tgtvertex;
  person.time_departure = startTimeH; // * 3600.0f; //seconds

  person.a = simParameters.a;
  person.b = simParameters.b;
  person.T = simParameters.T;
  person.v = 0;
  person.num_steps = 0;
  person.co = 0;
  person.active = 0;
  person.numOfLaneInEdge = 0;
  person.LC_stateofLaneChanging = 0;
  person.indexPathInit = 0; // 0 should point to -1.
}

void B18TrafficOD::resetTrafficPersonJob(
    std::vector<B18TrafficPerson> &trafficPersonVec) {
  for (int p = 0; p < trafficPersonVec.size(); p++) {
    trafficPersonVec[p].active = 0;
  }
}

void B18TrafficOD::loadB18TrafficPeopleSP(
    std::vector<B18TrafficPerson> &trafficPersonVec, // out
    const std::shared_ptr<RoadGraphB2018> &graph_loader,
    std::vector<float> dep_times) {

  trafficPersonVec.clear();

  // printf("demandB2018 size = %d\n", RoadGraphB2018::demandB2018.size());
  auto totalNumPeople = graph_loader->totalNumPeople();
  if (graph_loader->totalNumPeople() == 0) {
    printf("ERROR: Imposible to generate b2018 without loading b2018 demmand "
           "first\n");
    return;
  }
  trafficPersonVec.resize(totalNumPeople);

  int numPeople = 0;
  auto demand = graph_loader->demand();
  for (int d = 0; (d < totalNumPeople) && (numPeople < totalNumPeople); d++) {
    int odNumPeople =
        std::min<int>(totalNumPeople - numPeople, demand[d].num_people);
    // printf("odNumPeople = %d\n", odNumPeople);
    uint src_vertex = demand[d].src_vertex;
    uint tgt_vertex = demand[d].tgt_vertex;
    float goToWorkH = dep_times[d];
    // printf("dep time %f\n", goToWorkH);

    for (int p = 0; p < odNumPeople; p++) {
      randomPerson(numPeople, trafficPersonVec[numPeople], src_vertex,
                   tgt_vertex, goToWorkH);
      numPeople++;
    }
  }

  if (totalNumPeople != numPeople) {
    printf("ERROR: generateB2018TrafficPeople totalNumPeople != numPeople, "
           "this should not happen.");
    exit(-1);
  }

  printf("loadB18TrafficPeople: People %d\n", numPeople);
}
} // namespace LC
