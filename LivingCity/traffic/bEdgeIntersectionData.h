
#pragma once

#include <vector>


#include "stdint.h"

#define ushort uint16_t
#define uint uint32_t
#define uchar uint8_t

namespace LC {

struct BEdgesData {
  std::vector<uchar> numLinesB;

  std::vector<ushort> nextInters;
  std::vector<uchar> nextIntersType;

  std::vector<ushort> lengthC;//edge_length in cells
  std::vector<float> maxSpeedCpSec;//speed in cells per delta time

  void resize(int size) {
    numLinesB.resize(size);

    nextInters.resize(size);
    nextIntersType.resize(size);

    lengthC.resize(size);
    maxSpeedCpSec.resize(size);
  }

  void clear() {
    numLinesB.clear();
    nextInters.clear();
    nextIntersType.clear();
    lengthC.clear();
    maxSpeedCpSec.clear();
  }
};

/////////////////////////////////
// STOP

struct  BStopsData {
  std::vector<uchar>	state;
  std::vector<ushort> intersNumber;//number of intersect
  void resize(int size) {
    state.resize(size);
    intersNumber.resize(size);
  }
};

/////////////////////////////////
// TRAFFIC LIGHT
struct BTrafficLightData {
  std::vector<ushort> intersNumber;//number of intersect
  std::vector<uchar>	offSet;//time offset
  std::vector<ushort> phaseInd;
  std::vector<uchar>	numPhases;
  std::vector<uchar>	curPhase;

  std::vector<unsigned long> phases;
  std::vector<uchar>	 phaseTime;//note that it is *2
};

/////////////////////////////////
// INTERSECTION
struct IntersectionQ{
    int queue[3] = {};
    int q_ptr = 0;
};

struct  BIntersectionsData{
    std::map<int,IntersectionQ> roadId2q;
    std::vector<std::vector<IntersectionQ*>> paired_queues;
};

//struct BIntersectionsData {
//
//  std::vector<unsigned long> req;
//  std::vector<unsigned long> trafficLight;
//  // common
//  std::vector<uchar> numIn;
//  std::vector<uchar> type;
//  std::vector<float> nextEvent;
//
//
//  void resize(int size) {
//    req.resize(size);
//    trafficLight.resize(size);
//
//    numIn.resize(size);
//    type.resize(size);
//    nextEvent.resize(size);
//  }//
//
//  void resetReq() {
//    memset((uchar *)(&req[0]), 0, req.size()*sizeof(unsigned long));
//  }//
//
//  void resetTraff() {
//    memset((uchar *)(&trafficLight[0]), 0x00,
//           trafficLight.size()*sizeof(unsigned long));
//  }//
//};
}
