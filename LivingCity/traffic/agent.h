/************************************************************************************************
 *
 *		LC Project - B18 Traffic Person
 *
 *
 *		@desc Class that contains the info of a person
 *		@author igaciad
 *
 ************************************************************************************************/

#ifndef LC_B18_TRAFFIC_PERSON_H
#define LC_B18_TRAFFIC_PERSON_H

#include "config.h"

namespace LC {

struct Agent {
  Agent() = default;
  Agent(uint srcvertex, uint tgtvertex, float dep_time, IDMParameters param) {
    init_intersection = srcvertex;
    end_intersection = tgtvertex;
    time_departure = dep_time;
    a = param.a;
    b = param.b;
    T = param.T;
    s_0 = param.s_0;
  }

  unsigned int init_intersection;
  unsigned int end_intersection;
  float time_departure;
  float dist_traveled = 0;

  unsigned short active = 0;          // 0 inactive 1 active 2 finished
  unsigned short numOfLaneInEdge = 0; // number of lane in that edge

  float posInLaneM;

  //////////////////////////
  // current edge (from edgeData)
  unsigned short edgeNumLanes; // total number of lanes in that edge
  unsigned int edgeNextInters;
  float length;
  float maxSpeedMperSec;
  /////////////////////////
  // to check next edge
  unsigned short nextEdgeNumLanes;
  unsigned short nextEdgeNextInters;
  float nextEdgeLength;
  float nextEdgemaxSpeedMperSec;
  ///////////////////////////
  unsigned int indexPathInit = 0;
  unsigned int indexPathCurr;

  // for edge speed calculations
  unsigned int currentEdge;
  unsigned int nextEdge;
  unsigned int prevEdge;
  float start_time_on_prev_edge;
  float end_time_on_prev_edge;
  float manual_v;

  // data
  unsigned short num_steps = 0;
  float co = 0;
  float gas = 0;
  float cum_length = 0;
  unsigned short waited_steps = 0;
  unsigned short slow_down_steps = 0;

  // IDM
  float v = 0;     // current velocity
  float delta_v = 0; // velocity difference to the front car
  float s = 0; //space to the front car
  float a;         // acceleration
  float b;         // break
  float T;         // Time heading
  float s_0;       // min car following dist
  float cum_v = 0; // Cumulative velocity of each person across all iterations

  // lane changing
  unsigned short LC_initOKLanes;
  unsigned short LC_endOKLanes;
  unsigned short LC_stateofLaneChanging = 0;
  int num_lane_change = 0;
  int isInIntersection;
};

} // namespace LC

#endif // LC_B18_TRAFFIC_PERSON_H
