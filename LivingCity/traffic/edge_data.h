/************************************************************************************************
 *
 *		LC Project - B18 Edge data
 *
 *		@author igaciad
 *
 ************************************************************************************************/

#ifndef LC_B18_EDGE_DATA_H
#define LC_B18_EDGE_DATA_H

#include "stdint.h"
#include <map>
#include <set>
#include <vector>

#ifndef ushort
#define ushort uint16_t
#endif
#ifndef uint
#define uint uint32_t
#endif
#ifndef uchar
#define uchar uint8_t
#endif

namespace LC {
//! EdgeData Class
//! \brief Data structure that hold essential information for a road/edge in ONE
//! direction
struct EdgeData {
  //! number of lanes
  ushort num_lanes;
  //! Two end nodes of the edge
  uint vertex[2];
  //! Length of the edge (road)
  float length;
  //! max speed for the road (m/s)
  float maxSpeedMperSec;

  //! upstream vehicle count for the edge (number of vehicles enters the edge)
  unsigned int upstream_veh_count{0};
  //! downstream vehicle count for the edge (number of vehicles leaves the edge)
  unsigned int downstream_veh_count{0};
  //! cumulative travel steps for the edge in the simulation period
  unsigned int period_cum_travel_steps{0};
};

//! IntersectionData Class
//! \brief Data structure that hold essential information for a intersection
struct IntersectionData {
  //! Potential queues (for each direction) in the intersection
  int queue[100][10];
  //! Entering eid for each queue
  unsigned start_edge[100];
  //! Leaving eid for each queue
  unsigned end_edge[100];
  //! Ptr position for each queue (number of cars in queue)
  unsigned pos[100] = {0};

  unsigned short num_edge{0};
  unsigned short num_queue{0};
  unsigned short max_queue{0};
  //! Ptr for the next queue to check
  unsigned short queue_ptr{0};

  //! virtual queue for initialization
  int init_queue[1000] = {-1}; // +5 for safety
  unsigned init_queue_rear{0};
};

// struct IntersectionData {
//  ushort state;
//  ushort stateLine;
//  ushort totalInOutEdges;
//  uint edge[24]; // up to six arms intersection
//  float nextEvent;
//};
} // namespace LC

#endif // LC_B18_EDGE_DATA_H
