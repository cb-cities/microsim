// CUDA CODE
#include "assert.h"
#include "cuda.h"
#include "cuda_runtime.h"
#include "curand_kernel.h"
#include "device_launch_parameters.h"
#include <stdio.h>

#include "agent.h"
#include "b18EdgeData.h"
#include <iostream>
#include <vector>

#include "config.h"
#include "src/benchmarker.h"

#ifndef ushort
#define ushort uint16_t
#endif
#ifndef uint
#define uint uint32_t
#endif
#ifndef uchar
#define uchar uint8_t
#endif

///////////////////////////////
// CONSTANTS

__constant__ float intersectionClearance = 7.8f; // TODO(pavan): WHAT IS THIS?

#define gpuErrchk(ans)                                                         \
  { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort)
      exit(code);
  }
}
inline void printMemoryUsage() {
  // show memory usage of GPU
  size_t free_byte;
  size_t total_byte;
  cudaError_t cuda_status = cudaMemGetInfo(&free_byte, &total_byte);
  if (cudaSuccess != cuda_status) {
    printf("Error: cudaMemGetInfo fails, %s \n",
           cudaGetErrorString(cuda_status));
    exit(1);
  }
  double free_db = (double)free_byte;
  double total_db = (double)total_byte;
  double used_db = total_db - free_db;
  printf("GPU memory usage: used = %.0f, free = %.0f MB, total = %.0f MB\n",
         used_db / 1024.0 / 1024.0, free_db / 1024.0 / 1024.0,
         total_db / 1024.0 / 1024.0);
}
////////////////////////////////
// VARIABLES
LC::Agent *trafficPersonVec_d;
uint *indexPathVec_d;
LC::B18EdgeData *edgesData_d;

__constant__ bool calculatePollution = true;
__constant__ float cellSize = 1.0f;

//__constant__ float deltaTime = 0.5f;
// const float deltaTimeH = 0.5f;

// const uint numStepsPerSample = 30.0f / deltaTimeH; //each min
// const uint numStepsTogether = 12; //change also in density (10 per hour)

uchar *laneMap_d;
bool readFirstMapC = true;
uint mapToReadShift;
uint mapToWriteShift;
uint halfLaneMap;
float startTime;

LC::B18IntersectionData *intersections_d;
uchar *trafficLights_d;

float *accSpeedPerLinePerTimeInterval_d;
float *numVehPerLinePerTimeInterval_d;

void b18InitCUDA(
    bool fistInitialization, std::vector<LC::Agent> &trafficPersonVec,
    std::vector<uint> &indexPathVec, std::vector<LC::B18EdgeData> &edgesData,
    std::vector<uchar> &laneMap, std::vector<uchar> &trafficLights,
    std::vector<LC::B18IntersectionData> &intersections, float startTimeH,
    float endTimeH, std::vector<float> &accSpeedPerLinePerTimeInterval,
    std::vector<float> &numVehPerLinePerTimeInterval, float deltaTime) {
  // printf(">>b18InitCUDA firstInitialization %s\n",
  // (fistInitialization?"INIT":"ALREADY INIT")); printMemoryUsage();

  const uint numStepsPerSample = 30.0f / deltaTime; // each min
  const uint numStepsTogether = 12; // change also in density (10 per hour)
  {                                 // people
    size_t size = trafficPersonVec.size() * sizeof(LC::Agent);
    if (fistInitialization)
      gpuErrchk(cudaMalloc((void **)&trafficPersonVec_d,
                           size)); // Allocate array on device
    gpuErrchk(cudaMemcpy(trafficPersonVec_d, trafficPersonVec.data(), size,
                         cudaMemcpyHostToDevice));
  }

  { // indexPathVec
    size_t sizeIn = indexPathVec.size() * sizeof(uint);
    if (fistInitialization)
      gpuErrchk(cudaMalloc((void **)&indexPathVec_d,
                           sizeIn)); // Allocate array on device
    gpuErrchk(cudaMemcpy(indexPathVec_d, indexPathVec.data(), sizeIn,
                         cudaMemcpyHostToDevice));
  }
  { // edgeData
    size_t sizeD = edgesData.size() * sizeof(LC::B18EdgeData);
    if (fistInitialization)
      gpuErrchk(
          cudaMalloc((void **)&edgesData_d, sizeD)); // Allocate array on device
    gpuErrchk(cudaMemcpy(edgesData_d, edgesData.data(), sizeD,
                         cudaMemcpyHostToDevice));
  }
  { // laneMap
    size_t sizeL = laneMap.size() * sizeof(uchar);
    if (fistInitialization)
      gpuErrchk(
          cudaMalloc((void **)&laneMap_d, sizeL)); // Allocate array on device
    gpuErrchk(
        cudaMemcpy(laneMap_d, laneMap.data(), sizeL, cudaMemcpyHostToDevice));
    halfLaneMap = laneMap.size() / 2;
  }
  { // intersections
    size_t sizeI = intersections.size() * sizeof(LC::B18IntersectionData);
    if (fistInitialization)
      gpuErrchk(cudaMalloc((void **)&intersections_d,
                           sizeI)); // Allocate array on device
    gpuErrchk(cudaMemcpy(intersections_d, intersections.data(), sizeI,
                         cudaMemcpyHostToDevice));
    size_t sizeT = trafficLights.size() * sizeof(uchar); // total number of
                                                         // lanes
    if (fistInitialization)
      gpuErrchk(cudaMalloc((void **)&trafficLights_d,
                           sizeT)); // Allocate array on device
    gpuErrchk(cudaMemcpy(trafficLights_d, trafficLights.data(), sizeT,
                         cudaMemcpyHostToDevice));
  }
  {
    startTime = startTimeH * 3600.0f;
    uint numSamples =
        ceil(((endTimeH * 3600.0f - startTimeH * 3600.0f) /
              (deltaTime * numStepsPerSample * numStepsTogether))) +
        1; //!!!
    accSpeedPerLinePerTimeInterval.clear();
    numVehPerLinePerTimeInterval.clear();
    accSpeedPerLinePerTimeInterval.resize(numSamples * trafficLights.size());
    numVehPerLinePerTimeInterval.resize(numSamples * trafficLights.size());
    size_t sizeAcc = accSpeedPerLinePerTimeInterval.size() * sizeof(float);
    if (fistInitialization)
      gpuErrchk(cudaMalloc((void **)&accSpeedPerLinePerTimeInterval_d,
                           sizeAcc)); // Allocate array on device
    if (fistInitialization)
      gpuErrchk(cudaMalloc((void **)&numVehPerLinePerTimeInterval_d,
                           sizeAcc)); // Allocate array on device
    gpuErrchk(cudaMemset(&accSpeedPerLinePerTimeInterval_d[0], 0, sizeAcc));
    gpuErrchk(cudaMemset(&numVehPerLinePerTimeInterval_d[0], 0, sizeAcc));
  }
  printMemoryUsage();
} //

void b18FinishCUDA(void) {
  //////////////////////////////
  // FINISH
  cudaFree(trafficPersonVec_d);
  cudaFree(indexPathVec_d);
  cudaFree(edgesData_d);
  cudaFree(laneMap_d);
  cudaFree(intersections_d);
  cudaFree(trafficLights_d);

  cudaFree(accSpeedPerLinePerTimeInterval_d);
  cudaFree(numVehPerLinePerTimeInterval_d);
} //

void b18GetDataCUDA(std::vector<LC::Agent> &trafficPersonVec,
                    std::vector<LC::B18EdgeData> &edgesData,
                    std::vector<LC::B18IntersectionData> &intersections) {
  // copy back people
  size_t size = trafficPersonVec.size() * sizeof(LC::Agent);
  size_t size_edges = edgesData.size() * sizeof(LC::B18EdgeData);
  size_t size_intersections =
      intersections.size() * sizeof(LC::B18IntersectionData);

  cudaMemcpy(trafficPersonVec.data(), trafficPersonVec_d, size,
             cudaMemcpyDeviceToHost); // cudaMemcpyHostToDevice
  cudaMemcpy(edgesData.data(), edgesData_d, size_edges,
             cudaMemcpyDeviceToHost); // cudaMemcpyHostToDevice
  cudaMemcpy(intersections.data(), intersections_d, size_intersections,
             cudaMemcpyDeviceToHost); // cudaMemcpyHostToDevice
}

__device__ uint lanemap_pos(const uint currentEdge, const uint edge_length,
                            const uint laneNum, const uint pos_in_lane) {
  uint kMaxMapWidthM = 1024;
  uint num_cell = pos_in_lane / kMaxMapWidthM;
  int tot_num_cell = edge_length / kMaxMapWidthM;
  if (edge_length % kMaxMapWidthM) {
    tot_num_cell += 1;
  }
  return kMaxMapWidthM * currentEdge + kMaxMapWidthM * laneNum * tot_num_cell +
         kMaxMapWidthM * num_cell + pos_in_lane % kMaxMapWidthM;
}

__device__ void calculateGaps(uint mapToReadShift, uchar *laneMap,
                              LC::Agent &agent, uint laneToCheck, float &gap_a,
                              float &gap_b, uchar &v_a, uchar &v_b) {

  int kMaxMapWidthM = 1024;
  // CHECK FORWARD
  for (ushort b = agent.posInLaneM - 1; b < agent.length;
       b++) { // NOTE -1 to make sure there is none in at the same level
    auto posToSample =
        lanemap_pos(agent.currentEdge, agent.length, laneToCheck, b);
    if (laneMap[mapToReadShift + posToSample] != 0xFF) {
      gap_a = b - agent.posInLaneM; // m
      v_a = laneMap[mapToReadShift + posToSample] / 3;
      break;
    }
  }
  // CHECK BACKWARD
  for (ushort b = agent.posInLaneM + 1; b > 0;
       b--) { // NOTE -1 to make sure there is none in at the same level
    auto posToSample =
        lanemap_pos(agent.currentEdge, agent.length, laneToCheck, b);
    if (laneMap[mapToReadShift + posToSample] != 0xFF) {
      gap_b = agent.posInLaneM - b; // m
      v_b = laneMap[mapToReadShift + posToSample] / 3;
      break;
    }
  }
}
__device__ void calculateGapsLC(uint mapToReadShift, uchar *laneMap,
                                uchar trafficLightState, uint laneToCheck,
                                uint currentEdge, float posInMToCheck,
                                float length, uchar &v_a, uchar &v_b,
                                float &gap_a, float &gap_b) {

  ushort numOfCells = ceil(length);
  ushort initShift = ceil(posInMToCheck);
  uchar laneChar;
  bool found = false;
  int kMaxMapWidthM = 1024;

  // CHECK FORWARD
  // printf("initShift %u numOfCells %u\n",initShift,numOfCells);
  for (ushort b = initShift - 1; (b < numOfCells);
       b++) { // NOTE -1 to make sure there is none in at the same level
    // laneChar = laneMap[mapToReadShift + maxWidth * (laneToCheck) + b];
    const uint posToSample =
        mapToReadShift + kMaxMapWidthM * (currentEdge + laneToCheck) + b;
    laneChar = laneMap[posToSample];

    if (laneChar != 0xFF) {
      gap_a = ((float)b - initShift); // m
      v_a = laneChar; // laneChar is in 3*ms (to save space in array)
      found = true;
      break;
    }
  }

  if (!found) {
    if (trafficLightState == 0x00) { // red
      // found=true;
      gap_a = gap_b = 1000.0f; // force to change to the line without vehicle
      v_a = v_b = 0xFF;
      return;
    }
  }

  if (!found) {
    gap_a = 1000.0f;
  }

  // CHECK BACKWARDS
  found = false;
  // printf("2initShift %u numOfCells %u\n",initShift,numOfCells);
  for (int b = initShift + 1; (b >= 0);
       b--) { // NOTE +1 to make sure there is none in at the same level
    // laneChar = laneMap[mapToReadShift + maxWidth * (laneToCheck) + b];
    const uint posToSample =
        mapToReadShift + kMaxMapWidthM * (currentEdge + laneToCheck) + b;
    laneChar = laneMap[posToSample];
    if (laneChar != 0xFF) {
      gap_b = ((float)initShift - b); // m
      v_b = laneChar; // laneChar is in 3*ms (to save space in array)
      found = true;
      break;
    }
  }

  // printf("3initShift %u numOfCells %u\n",initShift,numOfCells);
  if (!found) {
    gap_b = 1000.0f;
  }

} //

__device__ void calculateLaneCarShouldBe(uint curEdgeLane, uint nextEdge,
                                         LC::B18IntersectionData *intersections,
                                         uint edgeNextInters,
                                         ushort edgeNumLanes,
                                         ushort &initOKLanes,
                                         ushort &endOKLanes) {

  initOKLanes = 0;
  endOKLanes = edgeNumLanes;
  bool currentEdgeFound = false;
  bool exitFound = false;
  ushort numExitToTake = 0;
  ushort numExists = 0;

  for (int eN = intersections[edgeNextInters].totalInOutEdges - 1; eN >= 0;
       eN--) { // clockwise
    uint procEdge = intersections[edgeNextInters].edge[eN];

    if ((procEdge & kMaskLaneMap) == curEdgeLane) { // current edge 0xFFFFF
      currentEdgeFound = true;
      if (exitFound == false) {
        numExitToTake = 0;
      }
      continue;
    }

    if ((procEdge & kMaskInEdge) == 0x0) { // out edge 0x800000
      numExists++;
      if (currentEdgeFound == true) {
        numExitToTake++;
      }
      if (currentEdgeFound == false && exitFound == false) {
        numExitToTake++;
      }
    }
    if ((procEdge & kMaskInEdge) == nextEdge) {
      exitFound = true;
      currentEdgeFound = false;
    }
  }

  if (edgeNumLanes == 0) {
    printf("ERRRROR\n");
  }

  switch (edgeNumLanes) {
  /// ONE LANE
  case 1:
    initOKLanes = 0;
    endOKLanes = 1;
    break;

    /// TWO LANE
  case 2:
    switch (numExists) {
    case 1:
    case 2: // all okay
      initOKLanes = 0;
      endOKLanes = 2;
      break;

    case 3:
      if (numExitToTake > 2) { // left
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 2;
      break;

    default:
      if (numExitToTake >= numExists - 1) {
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 2;
      break;
    }

    break;

    /// THREE LANE
  case 3:
    switch (numExists) {
    case 1:
    case 2: // all okay
      initOKLanes = 0;
      endOKLanes = 3;
      break;

    case 3:
      if (numExitToTake > 2) { // left
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 3;
      break;

    default:
      if (numExitToTake >= numExists - 1) {
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 2;
      break;
    }

    break;

  case 4:
    switch (numExists) {
    case 1:
    case 2: // all okay
      initOKLanes = 0;
      endOKLanes = 4;
      break;

    case 3:
      if (numExitToTake == 1) { // right
        initOKLanes = 3;
        endOKLanes = 4;
      }

      if (numExitToTake > 3) { // left
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 4;
      break;

    default:
      if (numExitToTake == 1) { // right
        initOKLanes = edgeNumLanes - 1;
        endOKLanes = edgeNumLanes;
      }

      if (numExitToTake >= numExists - 2) {
        initOKLanes = 0;
        endOKLanes = 2;
        break;
      }

      initOKLanes = 1; // also lane 2
      endOKLanes = edgeNumLanes;
    }

    break;

  default:
    switch (numExists) {
    case 1:
    case 2: // all okay
      initOKLanes = 0;
      endOKLanes = edgeNumLanes;
      break;

    case 3:
      if (numExitToTake == 1) { // right
        initOKLanes = edgeNumLanes - 1;
        endOKLanes = edgeNumLanes;
      }

      if (numExitToTake > edgeNumLanes - 2) { // left
        initOKLanes = 0;
        endOKLanes = 2;
        break;
      }

      initOKLanes = 1;
      endOKLanes = edgeNumLanes;
      break;

    default:
      if (numExitToTake < 2) { // right
        initOKLanes = edgeNumLanes - 2;
        endOKLanes = edgeNumLanes;
      }

      if (numExitToTake >= numExists - 2) {
        initOKLanes = 0;
        endOKLanes = 2;
        break;
      }

      initOKLanes = 1; // also lane 2
      endOKLanes = edgeNumLanes - 1;
    }

    break;
  }
} //

__device__ void initialize_agent(LC::Agent &agent, LC::B18EdgeData *edgesData,
                                 uint *indexPathVec, uchar *laneMap,
                                 uint mapToReadShift, uint mapToWriteShift) {

  // 1.2 find first edge
  agent.indexPathCurr = agent.indexPathInit; // reset index.
  agent.currentEdge = indexPathVec[agent.indexPathCurr];
  agent.nextEdge = indexPathVec[agent.indexPathCurr + 1];
  if (agent.currentEdge == -1) {
    agent.active = 2;
    // printf("0xFFFF\n");
    return;
  }

  // 1.3 update person edgeData
  // COPY DATA FROM EDGE TO PERSON
  agent.edgeNumLanes = edgesData[agent.currentEdge].numLines;
  agent.edgeNextInters = edgesData[agent.currentEdge].nextIntersMapped;
  agent.length = edgesData[agent.currentEdge].length;

  // printf("edgesData length %f\n",edgesData[firstEdge].length);
  agent.maxSpeedMperSec = edgesData[agent.currentEdge].maxSpeedMperSec;
  // printf("edgesData %.10f\n",edgesData[firstEdge].maxSpeedMperSec);

  // 1.4 try to place the car
  ushort lN = agent.edgeNumLanes - 1;
  bool enough_space = true;
  for (auto b = 0; b < agent.s_0; b++) {
    // just right LANE !!!!!!!
    auto pos = lanemap_pos(agent.currentEdge, agent.length, lN, b);
    auto laneChar =
        laneMap[mapToReadShift + pos]; // get byte of edge (proper line)
    if (laneChar != 0xFF) {
      enough_space = false;
      break;
    }
  }
  if (enough_space) {
    agent.v = 0;
    agent.LC_stateofLaneChanging = 0;
    agent.numOfLaneInEdge = lN;
    agent.posInLaneM = 0; // m
    uchar vInMpS =
        (uchar)(agent.v * 3); // speed in m/s *3 (to keep more precisio
    auto pos = lanemap_pos(agent.currentEdge, agent.length,
                           agent.numOfLaneInEdge, agent.posInLaneM);
    laneMap[mapToWriteShift + pos] = vInMpS;
    atomicAdd(
        &(edgesData[indexPathVec[agent.indexPathCurr]].upstream_veh_count), 1);
  } else {
    agent.num_steps++;
    agent.waited_steps++;
    return;
  }

  // 1.5 active car

  agent.active = 1;
  agent.isInIntersection = 0;
  // trafficPersonVec[p].nextPathEdge++;//incremet so it continues in next
  // edge

  // 1.6 update next edge
  if (agent.nextEdge != -1) {
    agent.nextEdgemaxSpeedMperSec = edgesData[agent.nextEdge].maxSpeedMperSec;
    agent.nextEdgeNumLanes = edgesData[agent.nextEdge].numLines;
    agent.nextEdgeNextInters = edgesData[agent.nextEdge].nextIntersMapped;
    agent.nextEdgeLength = edgesData[agent.nextEdge].length;
    // trafficPersonVec[p].nextPathEdge++;
    agent.LC_initOKLanes = 0xFF;
    agent.LC_endOKLanes = 0xFF;
  }
}

__device__ void check_front_car(LC::Agent &agent, uchar *laneMap,
                                float deltaTime, uint mapToReadShift) {

  int numCellsCheck = fmax(15.0f, agent.v * deltaTime); // 15 or speed*time
  ushort byteInLine = (ushort)floor(agent.posInLaneM);
  ushort numOfCells = ceil((agent.length) - 2);

  // a) SAME LINE (BEFORE SIGNALING)
  bool found = false;
  float s = 30;
  float delta_v = agent.v - agent.maxSpeedMperSec;
  for (ushort b = byteInLine + 2; (b < numOfCells) && (numCellsCheck > 0);
       b++, numCellsCheck--) {

    uint posToSample =
        lanemap_pos(agent.currentEdge, agent.length, agent.numOfLaneInEdge, b);
    auto laneChar = laneMap[mapToReadShift + posToSample];
    if (laneChar != 0xFF) {
      s = ((float)(b - byteInLine)); // m
      delta_v =
          agent.v -
          (laneChar / 3.0f); // laneChar is in 3*ms (to save space in array)
      found = true;
      agent.thirdTerm = b;
      break;
    }
  }
  // NEXT LINE
  // e) MOVING ALONG IN THE NEXT EDGE
  //  if (!found && numCellsCheck > 0) { // check if in next line
  //    if ((agent.nextEdge != -1)) {    // we haven't arrived to
  //      // destination next line)
  //      ushort nextEdgeLaneToBe = agent.numOfLaneInEdge; // same lane
  //
  //      // printf("trafficPersonVec[p].numOfLaneInEdge
  //      // %u\n",trafficPersonVec[p].numOfLaneInEdge);
  //      if (nextEdgeLaneToBe >= agent.nextEdgeNumLanes) {
  //        nextEdgeLaneToBe =
  //            agent.nextEdgeNumLanes - 1; // change line if there are less
  //            roads
  //      }
  //
  //      ushort numOfCells = ceil(agent.nextEdgeLength);
  //
  //      for (ushort b = 0; (b < numOfCells) && (numCellsCheck > 0);
  //           b++, numCellsCheck--) {
  //        // laneChar = laneMap[mapToReadShift + maxWidth * (nextEdge +
  //        // nextEdgeLaneToBe) + b];
  //        uint posToSample = lanemap_pos(agent.nextEdge, nextEdgeLaneToBe, b);
  //        auto laneChar = laneMap[mapToReadShift + posToSample];
  //
  //        if (laneChar != 0xFF) {
  //          s = ((float)(b)); // m
  //          delta_v = agent.v -
  //                    (laneChar / 3.0f); // laneChar is in 3*ms (to save space
  //                    in
  //          break;
  //        }
  //      }
  //    }
  //  }

  agent.s = s;
  agent.delta_v = delta_v;
}

__device__ void update_agent_info(LC::Agent &agent, float deltaTime) {

  // update speed
  float thirdTerm = 0;
  if (agent.delta_v > 1) { // car in front and slower than us
    // 2.1.2 calculate dv_dt
    float s_star =
        agent.s_0 +
        fmax(0.0f, (agent.v * agent.T + (agent.v * agent.delta_v) /
                                            (2 * sqrtf(agent.a * agent.b))));

    thirdTerm = powf(((s_star) / (agent.s)), 2);
    agent.slow_down_steps++;
    // printf("s_star[%d] = %f\n", p, s_star);
    // printf(">FOUND s_star %f thirdTerm %f!!!!\n",s_star,thirdTerm);
  }
  float numMToMove;
  if (agent.v == 0 and agent.delta_v == 0 and agent.posInLaneM > 0 and
      agent.s < 5) {
    numMToMove = 0; // stopped at the middle
  } else {
    float dv_dt =
        agent.a *
        (1.0f - std::pow((agent.v / agent.maxSpeedMperSec), 4) - thirdTerm);
    agent.dv_dt = dv_dt;

    // 2.1.3 update values
    numMToMove =
        fmax(0.0f, agent.v * deltaTime + 0.5f * (dv_dt)*deltaTime * deltaTime);

    //    agent.thirdTerm = agent.v;
    agent.v += dv_dt * deltaTime;
    if (agent.v < 0) {
      agent.v = 0;
      numMToMove = 0;
    }
  }
  agent.m2move = numMToMove;
  agent.cum_length += numMToMove;
  agent.cum_v += agent.v;
  agent.posInLaneM += numMToMove;
}

__device__ void change_lane(LC::Agent &agent, uchar *laneMap,
                            uint mapToReadShift, uchar *trafficLights) {

  if (agent.posInLaneM > agent.length) { // skip if will go to next edge
    return;
  }
  if (agent.edgeNumLanes < 2 || agent.nextEdge == -1 ||
      agent.v > 0.9 * agent.maxSpeedMperSec) {
    return; // skip if reach the destination/have no lane to change/cruising
            // (avoid periodic lane changing)
  }

  if (agent.v > 3.0f &&           // at least 10km/h to try to change lane
      agent.delta_v > -0.1 &&     // decelerating or stuck
      agent.num_steps % 5 == 0) { // just check every (5 steps) 5 seconds

    bool leftLane = agent.numOfLaneInEdge > 0; // at least one lane on the left
    bool rightLane =
        agent.numOfLaneInEdge < agent.edgeNumLanes - 1; // at least one lane

    if (leftLane && rightLane) {
      if (int(agent.v) % 2 == 0) { // pseudo random for change lane
        leftLane = false;
      }
    }

    ushort laneToCheck = agent.numOfLaneInEdge - 1;
    if (rightLane) {
      laneToCheck = agent.numOfLaneInEdge + 1;
    }

    uchar v_a, v_b;
    float gap_a = 1000.0f, gap_b = 1000.0f;
    calculateGaps(mapToReadShift, laneMap, agent, laneToCheck, gap_a, gap_b,
                  v_a, v_b);
    // printf("p %u LC 1 %u\n",p,laneToCheck);
    //        uchar trafficLightState =
    //            trafficLights[agent.currentEdge + agent.numOfLaneInEdge];

    //        calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
    //        laneToCheck,
    //                        agent.currentEdge, agent.posInLaneM,
    //                        agent.length, v_a, v_b, gap_a, gap_b);
    // Safe distance calculation
    float b1A = 0.05, b2A = 0.15;
    float b1B = 0.15, b2B = 0.40;
    // simParameters.s_0-> critical lead gap
    float g_na_D =
        fmax(agent.s_0, agent.s_0 + b1A * agent.v + b2A * (agent.v - v_a));
    float g_bn_D =
        fmax(agent.s_0, agent.s_0 + b1B * v_b + b2B * (v_b - agent.v));
    if (gap_b < g_bn_D || gap_a < g_na_D) { // gap smaller than critical gap
      return;
    }

    agent.numOfLaneInEdge = laneToCheck; // CHANGE LINE
    agent.num_lane_change += 1;
  }
}

__device__ uint find_intersetcion_id(LC::Agent &agent,
                                     LC::B18EdgeData *edgesData) {
  // find the intersection id
  auto &current_edge = edgesData[agent.currentEdge];
  auto &next_edge = edgesData[agent.nextEdge];
  for (unsigned i = 0; i < 2; i++) {
    auto vid = current_edge.vertex[i];
    for (unsigned j = 0; j < 2; j++) {
      if (next_edge.vertex[j] == vid) {
        return vid;
      }
    }
  }
  return 0;
}

__device__ uint find_queue_id(LC::Agent &agent,
                              LC::B18IntersectionData &intersection) {
  for (unsigned i = 0; i < intersection.num_queue; i++) {
    if (agent.currentEdge == intersection.start_edge[i] and
        agent.nextEdge == intersection.end_edge[i]) {
      return i;
    }
  }
  //  int pos0, pos1;
  //  for (unsigned i = 0; i < intersection.num_edge; i++) {
  //    if (agent.currentEdge == intersection.lanemap_id[i]) {
  //      pos0 = i;
  //    }
  //    if (agent.nextEdge == intersection.lanemap_id[i]) {
  //      pos1 = i;
  //    }
  //  }
  //  int base_idx = 0;
  //  if (pos0 > pos1) {
  //    base_idx += intersection.num_queue;
  //    int temp = pos1;
  //    pos1 = pos0;
  //    pos0 = temp;
  //  }
  //  int idx = (pos0 * intersection.num_edge - (pos0 * (pos0 + 1)) / 2 + pos1 -
  //             pos0 - 1);
  //  return idx;
}

__shared__ int mutex;

__device__ bool update_intersection(int agent_id, LC::Agent &agent,
                                    LC::B18EdgeData *edgesData,
                                    LC::B18IntersectionData *intersections) {
  if (agent.posInLaneM < agent.length) { // does not reach an intersection
    return false;
  }
  if (agent.nextEdge == -1) { // reach destination
    agent.active = 2;
    atomicAdd(&(edgesData[agent.currentEdge].downstream_veh_count), 1);
    return false;
  }
  auto intersetcion_id = find_intersetcion_id(agent, edgesData);
  auto &intersection = intersections[intersetcion_id];
  int queue_id = find_queue_id(agent, intersection);
  auto &queue = intersection.queue[queue_id];
  auto &queue_ptr = intersection.pos[queue_id];
  agent.in_queue = true;
  agent.v = 0; // in queue vehicle is stopped.
  // Synchronization Control
  bool isSet = false;
  do {
    if (isSet = atomicCAS(&mutex, 0, 1) == 0) {
      queue[queue_ptr] = agent_id;
      atomicAdd(&(queue_ptr), 1);
      atomicAdd(&(edgesData[agent.currentEdge].downstream_veh_count), 1);
    }
    if (isSet) {
      atomicExch(&mutex, 0);
      __syncthreads();
    }
  } while (!isSet);
  return true;
}

__device__ void write2lane_map(LC::Agent &agent, LC::B18EdgeData *edgesData,
                               uint *indexPathVec, uchar *laneMap,
                               uint mapToWriteShift) {
  // write to the lanemap if still on the edge
  auto posToSample = lanemap_pos(agent.currentEdge, agent.length,
                                 agent.numOfLaneInEdge, agent.posInLaneM);
  uchar vInMpS = (uchar)(agent.v * 3); // speed in m/s to fit in uchar
  laneMap[mapToWriteShift + posToSample] = vInMpS;
}

// Kernel that executes on the CUDA device
__global__ void kernel_trafficSimulation(
    int numPeople, float currentTime, uint mapToReadShift, uint mapToWriteShift,
    LC::Agent *trafficPersonVec, uint *indexPathVec, LC::B18EdgeData *edgesData,
    uchar *laneMap, LC::B18IntersectionData *intersections,
    uchar *trafficLights, float deltaTime, const IDMParameters simParameters) {

  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= numPeople) {
    return; // CUDA check (inside margins)
  }
  if (threadIdx.x == 0) {
    mutex = 0;
  }
  __syncthreads();

  auto &agent = trafficPersonVec[p];
  // 1. initialization
  if (agent.active == 2) { // agent is already finished
    return;
  }
  // 1.1. check if person should still wait or should start
  if (agent.active == 0) {
    if (agent.time_departure > currentTime) { // wait
      return;
    } else { // its your turn
      initialize_agent(agent, edgesData, indexPathVec, laneMap, mapToReadShift,
                       mapToWriteShift);
      return;
    }
  }

  // 2. Moving
  agent.num_steps++;
  agent.nextEdge = indexPathVec[agent.indexPathCurr + 1];
  if (agent.in_queue) {
    agent.num_steps_in_queue += 1;
    return;
  }

  // 2.1.1 Find front car
  check_front_car(agent, laneMap, deltaTime, mapToReadShift);
  // 2.1.2 Update agent information using the front car info
  update_agent_info(agent, deltaTime);
  //  2.1.3 Perform lane changing if necessary
  change_lane(agent, laneMap, mapToReadShift, trafficLights);
  // 2.14 check intersection
  bool added2queue = update_intersection(p, agent, edgesData, intersections);
  //  // 2.1.4 write the updated agent info to lanemap
  if (not added2queue) {
    write2lane_map(agent, edgesData, indexPathVec, laneMap, mapToWriteShift);
  }

} //

/*
__global__ void kernel_intersectionSTOPSimulation(
     uint numIntersections,
     float currentTime,
     LC::B18IntersectionData *intersections,
     uchar *trafficLights,
     LC::B18EdgeData* edgesData,//for the length
     uchar* laneMap,//to check if there are cars
     uint mapToReadShift) {
     int i = blockIdx.x * blockDim.x + threadIdx.x;
     if (i<numIntersections) {//CUDA check (inside margins)

     const float deltaEvent = 0.0f;

     //if(i==0)printf("i %d\n",i);
     if (currentTime > intersections[i].nextEvent &&
intersections[i].totalInOutEdges > 0) { uint edgeOT =
intersections[i].edge[intersections[i].state]; uchar numLinesO = edgeOT >> 24;
       uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF

       // red old traffic lights
       for (int nL = 0; nL < numLinesO; nL++) {
         trafficLights[edgeONum + nL] = 0x00; //red old traffic light
       }

       for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1; iN++) {
//to give a round intersections[i].state = (intersections[i].state + 1) %
           intersections[i].totalInOutEdges;//next light

         if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) ==
kMaskInEdge) {  // 0x800000 uint edgeIT =
intersections[i].edge[intersections[i].state]; uint edgeINum = edgeIT &
kMaskLaneMap; //get edgeI 0xFFFFF uchar numLinesI = edgeIT >> 24;
           /// check if someone in this edge
           int rangeToCheck = 5.0f; //5m
           ushort firstPosToCheck = edgesData[edgeINum].length -
intersectionClearance; //last po bool atLeastOneStopped = false;

           for (int posCheck = firstPosToCheck; rangeToCheck >= 0 && posCheck
>= 0; posCheck--, rangeToCheck--) { //as many cells as the rangeToCheck says
for (int nL = 0; nL < numLinesI; nL++) {
               //int cellNum = mapToReadShift + maxWidth * (edgeINum + nL) +
posCheck; const uint posToSample = mapToReadShift + kMaxMapWidthM * (edgeINum
+
(((int) (posCheck / kMaxMapWidthM)) * numLinesI) + nL) + posCheck %
kMaxMapWidthM;


               if (laneMap[posToSample] == 0) { //car stopped
                 trafficLights[edgeINum + nL] = 0x0F; // STOP SIGN 0x0F--> Let
pass atLeastOneStopped = true;
               }
             }
           }

           if (atLeastOneStopped == true) {
             intersections[i].nextEvent = currentTime + deltaEvent; //just
move forward time if changed (otherwise check in next iteration) break;
           }
         }
       }
     }
     ///
   }

}//
*/
__device__ bool check_space(int space, int eid, int edge_length, uchar *laneMap,
                            uint mapToReadShift) {
  for (auto b = 0; b < space; b++) {
    // just right LANE !!!!!!!
    auto pos = lanemap_pos(eid, edge_length, 0, b);
    auto laneChar =
        laneMap[mapToReadShift + pos]; // get byte of edge (proper line)
    if (laneChar != 0xFF) {
      return false;
    }
  }
  return true;
}

__device__ void move2nextEdge(LC::Agent &agent, int numMToMove,
                              LC::B18EdgeData *edgesData, uint *indexPathVec,
                              uchar *laneMap, uint mapToWriteShift) {

  agent.indexPathCurr++;
  agent.maxSpeedMperSec = agent.nextEdgemaxSpeedMperSec;
  agent.edgeNumLanes = agent.nextEdgeNumLanes;
  agent.edgeNextInters = agent.nextEdgeNextInters;
  agent.length = agent.nextEdgeLength;
  agent.posInLaneM = numMToMove;
  agent.currentEdge = indexPathVec[agent.indexPathCurr];
  atomicAdd(&(edgesData[agent.currentEdge].upstream_veh_count), 1);
  if (agent.numOfLaneInEdge >= agent.edgeNumLanes) {
    agent.numOfLaneInEdge =
        agent.edgeNumLanes - 1; // change line if there are less roads
  }
  ////////////
  // update next edge
  uint nextNEdge = indexPathVec[agent.indexPathCurr + 1];
  agent.nextEdge = nextNEdge;
  if (nextNEdge != -1) {
    // trafficPersonVec[p].nextPathEdge++;
    agent.LC_initOKLanes = 0xFF;
    agent.LC_endOKLanes = 0xFF;
    // 2.2.3 update person edgeData
    // trafficPersonVec[p].nextEdge=nextEdge;
    agent.nextEdgemaxSpeedMperSec = edgesData[nextNEdge].maxSpeedMperSec;
    agent.nextEdgeNumLanes = edgesData[nextNEdge].numLines;
    agent.nextEdgeNextInters = edgesData[nextNEdge].nextIntersMapped;
    agent.nextEdgeLength = edgesData[nextNEdge].length;
  }
  //
  agent.LC_stateofLaneChanging = 0;
  auto posToSample = lanemap_pos(agent.currentEdge, agent.length,
                                 agent.numOfLaneInEdge, agent.posInLaneM);
  uchar vInMpS = (uchar)(agent.v * 3); // speed in m/s to fit in uchar
  laneMap[mapToWriteShift + posToSample] = vInMpS;
  agent.in_queue = false;
}
__device__ bool empty_queue(LC::B18IntersectionData &intersection,
                            int queue_ptr, LC::Agent *trafficPersonVec,
                            LC::B18EdgeData *edgesData, uint *indexPathVec,
                            uchar *laneMap, uint mapToReadShift,
                            uint mapToWriteShift) {
  auto &q1 = intersection.queue[queue_ptr];
  auto &n1 = intersection.pos[queue_ptr];
  unsigned eid1 = intersection.end_edge[queue_ptr];
  int numMToMove1 = (n1 + 1) * 3;
  int edge_length = edgesData[eid1].length;
  bool enough_space1 = check_space(numMToMove1 + 6, eid1, edge_length, laneMap,
                                   mapToReadShift); // check 6m ahead
  intersection.max_queue = max(intersection.max_queue, n1);
  if (enough_space1) {
    for (int i = 0; i < n1; ++i) {
      auto &agent = trafficPersonVec[q1[i]];
      move2nextEdge(agent, numMToMove1, edgesData, indexPathVec, laneMap,
                    mapToWriteShift); // move to the next edge
      numMToMove1 -= 3;
    }
    intersection.pos[queue_ptr] = 0; // cleared, reset the pointer
    return true;
  }
  return false;
}

__device__ void place_stop(LC::Agent &agent, LC::B18EdgeData *edgesData,
                           uchar *laneMap, uint mapToWriteShift) {
  auto &edge = edgesData[agent.currentEdge];
  for (int j = 0; j < 5; ++j) {
    auto pos = agent.length - j;
    for (int i = 0; i < edge.numLines; ++i) {
      auto posToSample = lanemap_pos(agent.currentEdge, agent.length, i, pos);
      laneMap[mapToWriteShift + posToSample] = 0;
    }
  }
}

__global__ void kernel_intersectionOneSimulation(
    uint numIntersections, uint mapToWriteShift, uint mapToReadShift,
    LC::B18EdgeData *edgesData, LC::B18IntersectionData *intersections,
    uint *indexPathVec, LC::Agent *trafficPersonVec, uchar *laneMap) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= numIntersections) {
    return; // CUDA check (inside margins)
  }
  auto &intersection = intersections[i];
  auto &queues = intersection.queue;
  auto &queue_counter = intersection.pos;
  unsigned start_ptr = intersection.queue_ptr;
  unsigned end_ptr = intersection.num_queue / 2;
  while (intersection.queue_ptr + 1 != start_ptr) {
    unsigned n1 = queue_counter[intersection.queue_ptr];
    unsigned n2 = queue_counter[end_ptr + intersection.queue_ptr];
    if (n1 + n2 > 0) {
      bool empty1 = empty_queue(intersection, intersection.queue_ptr,
                                trafficPersonVec, edgesData, indexPathVec,
                                laneMap, mapToReadShift, mapToWriteShift);
      bool empty2 = empty_queue(intersection, end_ptr + intersection.queue_ptr,
                                trafficPersonVec, edgesData, indexPathVec,
                                laneMap, mapToReadShift, mapToWriteShift);
      //
      if (!empty1 or !empty2) {
        if (intersection.queue_ptr == 0) {
          intersection.queue_ptr = end_ptr - 1;
        } else {
          intersection.queue_ptr -= 1;
        }
      } // if not clear, try it again for the next time
      break;
    }
    if (intersection.queue_ptr + 1 < end_ptr) {
      intersection.queue_ptr += 1;
    } else {
      intersection.queue_ptr = 0;
      if (start_ptr == 0)
        break;
    }
  }
  //     add stop sign for full queues
  for (unsigned j = 0; j < intersection.num_queue; j++) {
    auto num_cars = queue_counter[j];
    if (num_cars > 3) {
      auto &q1 = intersection.queue[j];
      auto &agent = trafficPersonVec[q1[0]];
      auto eid = intersection.start_edge[j];
      agent.located_eid = eid;
      place_stop(agent, edgesData, laneMap, mapToWriteShift);
    }
  }
}

//  if (i < numIntersections) {       // CUDA check (inside margins)
//    const float deltaEvent = 20.0f; /// !!!!
//    if (currentTime > intersections[i].nextEvent &&
//        intersections[i].totalInOutEdges > 0) {
//
//      uint edgeOT = intersections[i].edge[intersections[i].state];
//      uchar numLinesO = edgeOT >> 24;
//      uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF;
//
//      // red old traffic lights
//      if ((edgeOT & kMaskInEdge) ==
//          kMaskInEdge) { // Just do it if we were in in
//        for (int nL = 0; nL < numLinesO; nL++) {
//          trafficLights[edgeONum + nL] = 0x00; // red old traffic light
//        }
//      }
//
//      for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1;
//           iN++) { // to give a round
//        intersections[i].state = (intersections[i].state + 1) %
//                                 intersections[i].totalInOutEdges; // next
//                                 light
//
//        if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) ==
//            kMaskInEdge) { // 0x800000
//          // green new traffic lights
//          uint edgeIT = intersections[i].edge[intersections[i].state];
//          uint edgeINum = edgeIT & kMaskLaneMap; //  0xFFFFF; //get edgeI
//          uchar numLinesI = edgeIT >> 24;
//
//          for (int nL = 0; nL < numLinesI; nL++) {
//            trafficLights[edgeINum + nL] = 0xFF;
//          }
//
//          // trafficLights[edgeINum]=0xFF;
//          break;
//        }
//      } // green new traffic light
//
//      intersections[i].nextEvent = currentTime + deltaEvent;
//    }
//    //////////////////////////////////////////////////////
//  }

//} //

// Kernel that executes on the CUDA device
__global__ void kernel_sampleTraffic(
    int numPeople, LC::Agent *trafficPersonVec, uint *indexPathVec,
    float *accSpeedPerLinePerTimeInterval,
    float *numVehPerLinePerTimeInterval, // this could have been int
    uint offset) {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p < numPeople) {                     // CUDA check (inside margins)
    if (trafficPersonVec[p].active == 1) { // just active
      int edgeNum = indexPathVec[trafficPersonVec[p].indexPathCurr];
      accSpeedPerLinePerTimeInterval[edgeNum + offset] +=
          trafficPersonVec[p].v / 3.0f;
      numVehPerLinePerTimeInterval[edgeNum + offset]++;
    }
  }
}
__global__ void kernel_resetPeople(int numPeople, LC::Agent *trafficPersonVec) {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p < numPeople) { // CUDA check (inside margins)
    trafficPersonVec[p].active = 0;
  }
}

void b18GetSampleTrafficCUDA(std::vector<float> &accSpeedPerLinePerTimeInterval,
                             std::vector<float> &numVehPerLinePerTimeInterval) {
  // copy back people
  size_t size = accSpeedPerLinePerTimeInterval.size() * sizeof(float);
  cudaMemcpy(accSpeedPerLinePerTimeInterval.data(),
             accSpeedPerLinePerTimeInterval_d, size, cudaMemcpyDeviceToHost);

  size_t sizeI = numVehPerLinePerTimeInterval.size() * sizeof(uchar);
  cudaMemcpy(numVehPerLinePerTimeInterval.data(),
             numVehPerLinePerTimeInterval_d, sizeI, cudaMemcpyDeviceToHost);
}

void b18ResetPeopleLanesCUDA(uint numPeople) {
  kernel_resetPeople<<<ceil(numPeople / 1024.0f), 1024>>>(numPeople,
                                                          trafficPersonVec_d);
  cudaMemset(&laneMap_d[0], -1, halfLaneMap * sizeof(unsigned char));
  cudaMemset(&laneMap_d[halfLaneMap], -1, halfLaneMap * sizeof(unsigned char));
}

void b18SimulateTrafficCUDA(float currentTime, uint numPeople,
                            uint numIntersections, float deltaTime,
                            const IDMParameters simParameters, int numBlocks,
                            int threadsPerBlock) {
  intersectionBench.startMeasuring();
  const uint numStepsTogether = 12; // change also in density (10 per hour)
  ////////////////////////////////////////////////////////////
  // 1. CHANGE MAP: set map to use and clean the other
  if (readFirstMapC) {
    mapToReadShift = 0;
    mapToWriteShift = halfLaneMap;
    gpuErrchk(
        cudaMemset(&laneMap_d[halfLaneMap], -1,
                   halfLaneMap * sizeof(unsigned char))); // clean second half
  } else {
    mapToReadShift = halfLaneMap;
    mapToWriteShift = 0;
    gpuErrchk(
        cudaMemset(&laneMap_d[0], -1,
                   halfLaneMap * sizeof(unsigned char))); // clean first half
  }
  readFirstMapC = !readFirstMapC; // next iteration invert use

  // Simulate intersections.
  kernel_intersectionOneSimulation<<<numBlocks, threadsPerBlock>>>(
      numIntersections, mapToWriteShift, mapToReadShift, edgesData_d,
      intersections_d, indexPathVec_d, trafficPersonVec_d, laneMap_d);
  gpuErrchk(cudaPeekAtLastError());

  intersectionBench.stopMeasuring();

  peopleBench.startMeasuring();
  // Simulate people.
  kernel_trafficSimulation<<<numBlocks, threadsPerBlock>>>(
      numPeople, currentTime, mapToReadShift, mapToWriteShift,
      trafficPersonVec_d, indexPathVec_d, edgesData_d, laneMap_d,
      intersections_d, trafficLights_d, deltaTime, simParameters);
  gpuErrchk(cudaPeekAtLastError());
  peopleBench.stopMeasuring();

  // Sample if necessary.
  //  if ((((float)((int)currentTime)) == (currentTime)) &&
  //      ((int)currentTime % ((int)30)) == 0) { // 3min //(sample double each
  //      3min)
  //    int samplingNumber = (currentTime - startTime) / (30 *
  //    numStepsTogether); uint offset = numIntersections * samplingNumber;
  //    // printf("Sample %d\n", samplingNumber);
  //    kernel_sampleTraffic<<<ceil(numPeople / 1024.0f), 1024>>>(
  //        numPeople, trafficPersonVec_d, indexPathVec_d,
  //        accSpeedPerLinePerTimeInterval_d, numVehPerLinePerTimeInterval_d,
  //        offset);
  //    gpuErrchk(cudaPeekAtLastError());
  //  }
} //
