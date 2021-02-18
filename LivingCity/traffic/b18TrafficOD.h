
/************************************************************************************************
*
*		Procedural Machine Traffic Person B2018
*
*		@desc Class that generates procedually a traffic person
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_PM_TRAFFIC_PERSON_H
#define LC_B18_PM_TRAFFIC_PERSON_H

#include "../misctools/misctools.h"

#include <QtGlobal>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "agent.h"
#include "RoadGraph/roadGraph.h"
#include "sp/graph.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <random>
#include <qt5/QtGui/QVector3D>
#include <traffic/network.h>

namespace LC {
class B18TrafficOD {

 public:
  B18TrafficOD(const parameters & inputSimParameters);
  ~B18TrafficOD();

  const parameters simParameters;

  void randomPerson(int p, B18TrafficPerson &person, uint srcvertex,
                    uint tgtvertex, float startTimeH);

  void loadB18TrafficPeopleSP(std::vector<B18TrafficPerson> &trafficPersonVec, // out
    const std::shared_ptr<RoadGraphB2018>& graph_loader, std::vector<float> dep_times);

  void resetTrafficPersonJob(std::vector<B18TrafficPerson> &trafficPersonVec);
};
}

#endif  // LC_B18_PM_TRAFFIC_PERSON_H
