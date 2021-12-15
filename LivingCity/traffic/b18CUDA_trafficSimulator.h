/************************************************************************************************
 *
 *		CUDA hearder
 *
 *		@author igarciad
 *
 ************************************************************************************************/

#ifndef B18_TRAFFIC_SIMULATION_H
#define B18_TRAFFIC_SIMULATION_H

#include "agent.h"
#include "edge_data.h"
#include <vector>

extern void b18InitCUDA(
    bool fistInitialization, // crate buffers
    std::vector<LC::Agent> &trafficPersonVec, std::vector<uint> &indexPathVec,
    std::vector<LC::EdgeData> &edgesData, std::vector<uchar> &laneMap,
    std::vector<uchar> &trafficLights,
    std::vector<LC::IntersectionData> &intersections, float startTimeH,
    float endTimeH, std::vector<float> &accSpeedPerLinePerTimeInterval,
    std::vector<float> &numVehPerLinePerTimeInterval, float deltaTime);
extern void b18GetDataCUDA(std::vector<LC::Agent> &trafficPersonVec,
                           std::vector<LC::EdgeData> &edgesData,
                           std::vector<LC::IntersectionData> &intersections);
extern void
b18GetSampleTrafficCUDA(std::vector<float> &accSpeedPerLinePerTimeInterval,
                        std::vector<float> &numVehPerLinePerTimeInterval);
extern void b18FinishCUDA(void);                     // free memory
extern void b18ResetPeopleLanesCUDA(uint numPeople); // reset people to inactive
extern void b18SimulateTrafficCUDA(float currentTime, uint numPeople,
                                   uint numIntersections, float deltaTime,
                                   const LC::IDMParameters simParameters,
                                   int numBlocks, int threadsPerBlock);

#endif // B18_TRAFFIC_SIMULATION_H
