#ifndef B18_TRAFFIC_SIMULATION_H
#define B18_TRAFFIC_SIMULATION_H
#include <vector>
#include <iostream>

#include "agent.h"
#include "edge_data.h"
#include "config.h"
#include "src/benchmarker.h"




extern void init_cuda (
        bool fistInitialization, // crate buffers
        std::vector<LC::Agent> &agents,
        std::vector<LC::EdgeData> &edgesData, std::vector<uchar> &laneMap,
        std::vector<LC::IntersectionData> &intersections, float deltaTime);



extern void cuda_get_data (std::vector<LC::Agent> &trafficPersonVec,
                           std::vector<LC::EdgeData> &edgesData,
                           std::vector<LC::IntersectionData> &intersections);

extern void finish_cuda (void);                     // free memory
extern void cuda_simulate(float currentTime, uint numPeople, uint numIntersections,
                          float deltaTime, int numBlocks, int threadsPerBlock);

#endif // B18_TRAFFIC_SIMULATION_H
