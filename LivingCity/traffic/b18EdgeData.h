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
#include <vector>
#include <map>
#include <set>

#ifndef ushort
#define ushort uint16_t
#endif
#ifndef uint
#define uint uint32_t
#endif
#ifndef uchar
#define uchar uint8_t
#endif


const uint kMaskOutEdge = 0x000000;
const uint kMaskInEdge = 0x800000;
const uint kMaskLaneMap = 0x007FFFFF;

namespace LC {

struct B18EdgeData {
  ushort numLines;
  uint nextInters;
  uint vertex[2];
  float length;
  float maxSpeedMperSec;
  uint nextIntersMapped;

  unsigned int upstream_veh_count = 0;
  unsigned int downstream_veh_count = 0;
};

//struct IntersectionQ{
//        int queue[3] = {};
//        int q_ptr = 0;
//    };
//
//struct  B18IntersectionData{
//    std::vector<std::vector<IntersectionQ>> paired_queues;
//    std::map<std::vector<unsigned int>,IntersectionQ*> dir2q;
//    //  ushort state;
//  ushort state;
//  ushort stateLine;
//  ushort totalInOutEdges;
//  uint edge[24]; // up to six arms intersection
//  float nextEvent;
//};

struct B18IntersectionData{
    int lanemap_id[6];
    int queue[30][30];
    int start_edge[30];
    int end_edge[30];
    int pos[30] = {0};
    unsigned num_edge;
    unsigned num_queue;
    unsigned queue_ptr;
    unsigned max_queue;

    ushort state;
    ushort stateLine;
    ushort totalInOutEdges;
    uint edge[24]; // up to six arms intersection
    float nextEvent;

};


//struct B18IntersectionData {
//  ushort state;
//  ushort stateLine;
//  ushort totalInOutEdges;
//  uint edge[24]; // up to six arms intersection
//  float nextEvent;
//};
} // namespace LC

#endif // LC_B18_EDGE_DATA_H
