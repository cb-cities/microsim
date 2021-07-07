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
                    std::vector<LC::B18EdgeData> &edgesData) {
  // copy back people
  size_t size = trafficPersonVec.size() * sizeof(LC::Agent);
  size_t size_edges = edgesData.size() * sizeof(LC::B18EdgeData);
  cudaMemcpy(trafficPersonVec.data(), trafficPersonVec_d, size,
             cudaMemcpyDeviceToHost); // cudaMemcpyHostToDevice
  cudaMemcpy(edgesData.data(), edgesData_d, size_edges,
             cudaMemcpyDeviceToHost); // cudaMemcpyHostToDevice
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

__device__ void initialize_agent(LC::Agent *trafficPersonVec, int p,
                                 LC::B18EdgeData *edgesData, uint *indexPathVec,
                                 uchar *laneMap,
                                 const IDMParameters simParameters,
                                 uint mapToReadShift, uint mapToWriteShift) {
  int kMaxMapWidthM = 1024;
  // start
  // printf("p %d edge = %u\n", p, trafficPersonVec[p].indexPathInit);
  // 1.2 find first edge
  trafficPersonVec[p].indexPathCurr =
      trafficPersonVec[p].indexPathInit; // reset index.
  uint firstEdge = indexPathVec[trafficPersonVec[p].indexPathCurr];
  uint nextEdge = indexPathVec[trafficPersonVec[p].indexPathCurr + 1];
  // printf("indexPathVec %d = %u nextEdge = %u\n", p,
  // indexPathVec[trafficPersonVec[p].indexPathCurr],
  // indexPathVec[trafficPersonVec[p].indexPathCurr + 1]);

  if (firstEdge == -1) {
    trafficPersonVec[p].active = 2;
    // printf("0xFFFF\n");
    return;
  }

  // 1.3 update person edgeData
  // COPY DATA FROM EDGE TO PERSON
  trafficPersonVec[p].edgeNumLanes = edgesData[firstEdge].numLines;
  trafficPersonVec[p].edgeNextInters = edgesData[firstEdge].nextIntersMapped;
  // printf("edgeNextInters %u = %u\n", firstEdge,
  // edgesData[firstEdge].nextIntersMapped);

  trafficPersonVec[p].length = edgesData[firstEdge].length;

  // printf("edgesData length %f\n",edgesData[firstEdge].length);
  trafficPersonVec[p].maxSpeedMperSec = edgesData[firstEdge].maxSpeedMperSec;
  // printf("edgesData %.10f\n",edgesData[firstEdge].maxSpeedMperSec);

  // 1.4 try to place the car
  ushort lN = trafficPersonVec[p].edgeNumLanes - 1;
  bool enough_space = true;
  for (auto b = 0; b < simParameters.s_0; b++) {
    // just right LANE !!!!!!!
    auto laneChar = laneMap[mapToReadShift + kMaxMapWidthM * (firstEdge + lN) +
                            b]; // get byte of edge (proper line)
    if (laneChar != 0xFF) {
      enough_space = false;
      break;
    }
  }
  if (enough_space) {
    trafficPersonVec[p].v = 0;
    trafficPersonVec[p].LC_stateofLaneChanging = 0;
    trafficPersonVec[p].numOfLaneInEdge = lN;
    trafficPersonVec[p].posInLaneM = 0; // m
    uchar vInMpS = (uchar)(trafficPersonVec[p].v *
                           3); // speed in m/s *3 (to keep more precision
    laneMap[mapToWriteShift + kMaxMapWidthM * (firstEdge + lN)] = vInMpS;
    atomicAdd(&(edgesData[indexPathVec[trafficPersonVec[p].indexPathCurr]]
                    .upstream_veh_count),
              1);
  } else {
    trafficPersonVec[p].num_steps++;
    trafficPersonVec[p].waited_steps++;
    return;
  }

  // 1.5 active car

  trafficPersonVec[p].active = 1;
  trafficPersonVec[p].isInIntersection = 0;
  trafficPersonVec[p].co = 0.0f;
  trafficPersonVec[p].gas = 0.0f;
  // trafficPersonVec[p].nextPathEdge++;//incremet so it continues in next
  // edge

  // trafficPersonVec[p].nextEdge=nextEdge;
  if (nextEdge != -1) {
    trafficPersonVec[p].nextEdgemaxSpeedMperSec =
        edgesData[nextEdge].maxSpeedMperSec;
    trafficPersonVec[p].nextEdgeNumLanes = edgesData[nextEdge].numLines;
    trafficPersonVec[p].nextEdgeNextInters =
        edgesData[nextEdge].nextIntersMapped;
    trafficPersonVec[p].nextEdgeLength = edgesData[nextEdge].length;
    // trafficPersonVec[p].nextPathEdge++;
    trafficPersonVec[p].LC_initOKLanes = 0xFF;
    trafficPersonVec[p].LC_endOKLanes = 0xFF;
  }
}

__device__ void check_front_car(LC::Agent *trafficPersonVec, int p,
                                uint *indexPathVec, uchar *laneMap,
                                float deltaTime, uint mapToReadShift,
                                float *front_car_info) {

  int kMaxMapWidthM = 1024;
  uint currentEdge = indexPathVec[trafficPersonVec[p].indexPathCurr];
  uint nextEdge = indexPathVec[trafficPersonVec[p].indexPathCurr + 1];

  int numCellsCheck = fmax(50.0f, trafficPersonVec[p].v * deltaTime *
                                      2); // 30 or double of the speed*time
  ushort byteInLine = (ushort)floor(trafficPersonVec[p].posInLaneM);
  ushort numOfCells = ceil((trafficPersonVec[p].length) - 2);

  // a) SAME LINE (BEFORE SIGNALING)
  bool found = false;
  float s{0};
  float delta_v{0};
  for (ushort b = byteInLine + 2; (b < numOfCells) && (numCellsCheck > 0);
       b++, numCellsCheck--) {
    const uint posToSample =
        mapToReadShift +
        kMaxMapWidthM * (currentEdge + trafficPersonVec[p].numOfLaneInEdge) + b;
    auto laneChar = laneMap[posToSample];
    if (laneChar != 0xFF) {
      s = ((float)(b - byteInLine)); // m
      delta_v =
          trafficPersonVec[p].v -
          (laneChar / 3.0f); // laneChar is in 3*ms (to save space in array)
      found = true;
      break;
    }
  }
  // NEXT LINE
  // e) MOVING ALONG IN THE NEXT EDGE
  if (!found && numCellsCheck > 0) { // check if in next line
    if ((nextEdge != -1) &&
        (trafficPersonVec[p].edgeNextInters !=
         trafficPersonVec[p].end_intersection)) { // we haven't arrived to
      // destination next line)
      ushort nextEdgeLaneToBe =
          trafficPersonVec[p].numOfLaneInEdge; // same lane

      // printf("trafficPersonVec[p].numOfLaneInEdge
      // %u\n",trafficPersonVec[p].numOfLaneInEdge);
      if (nextEdgeLaneToBe >= trafficPersonVec[p].nextEdgeNumLanes) {
        nextEdgeLaneToBe = trafficPersonVec[p].nextEdgeNumLanes -
                           1; // change line if there are less roads
      }

      // printf("2trafficPersonVec[p].numOfLaneInEdge
      // %u\n",trafficPersonVec[p].numOfLaneInEdge);
      ushort numOfCells = ceil(trafficPersonVec[p].nextEdgeLength);

      for (ushort b = 0; (b < numOfCells) && (numCellsCheck > 0);
           b++, numCellsCheck--) {
        // laneChar = laneMap[mapToReadShift + maxWidth * (nextEdge +
        // nextEdgeLaneToBe) + b];
        const uint posToSample = mapToReadShift +
                                 kMaxMapWidthM * (nextEdge + nextEdgeLaneToBe) +
                                 b; // b18 not changed since we check
        auto laneChar = laneMap[posToSample];

        if (laneChar != 0xFF) {
          s = ((float)(b)); // m
          delta_v = trafficPersonVec[p].v -
                    (laneChar / 3.0f); // laneChar is in 3*ms (to save space in
          break;
        }
      }
    }
  }
  front_car_info[0] = s;
  front_car_info[1] = delta_v;
}

__device__ void update_agent_info(LC::Agent *trafficPersonVec, int p,
                                  float deltaTime,
                                  const IDMParameters simParameters,
                                  float *front_car_info) {
  auto s = front_car_info[0];
  auto delta_v = front_car_info[1];

  // update speed
  float thirdTerm = 0;
  if (delta_v > 1) { // car in front and slower than us
    // if (found == true) { //car in front and slower than us
    // 2.1.2 calculate dv_dt
    float s_star =
        simParameters.s_0 +
        fmax(0.0f,
             (trafficPersonVec[p].v * trafficPersonVec[p].T +
              (trafficPersonVec[p].v * delta_v) /
                  (2 * sqrtf(trafficPersonVec[p].a * trafficPersonVec[p].b))));
    thirdTerm = powf(((s_star) / (s)), 2);
    trafficPersonVec[p].slow_down_steps++;
    // printf("s_star[%d] = %f\n", p, s_star);
    // printf(">FOUND s_star %f thirdTerm %f!!!!\n",s_star,thirdTerm);
  }

  float dv_dt =
      trafficPersonVec[p].a *
      (1.0f -
       std::pow((trafficPersonVec[p].v / trafficPersonVec[p].maxSpeedMperSec),
                4) -
       thirdTerm);

  // 2.1.3 update values
  auto numMToMove = fmax(0.0f, trafficPersonVec[p].v * deltaTime +
                                   0.5f * (dv_dt)*deltaTime * deltaTime);
  trafficPersonVec[p].cum_length += numMToMove;
  trafficPersonVec[p].v += dv_dt * deltaTime;
  if (trafficPersonVec[p].v < 0) {
    trafficPersonVec[p].v = 0;
    dv_dt = 0.0f;
  }
  trafficPersonVec[p].cum_v += trafficPersonVec[p].v;
  trafficPersonVec[p].posInLaneM = trafficPersonVec[p].posInLaneM + numMToMove;
}

__device__ void change_lane(LC::Agent *trafficPersonVec, int p,
                            const IDMParameters simParameters,
                            uint *indexPathVec, uchar *laneMap,
                            uint mapToReadShift,
                            uchar *trafficLights) {

  if (trafficPersonVec[p].posInLaneM >
      trafficPersonVec[p].length) { // skip if will go to next edge
    return;
  }

  uint currentEdge = indexPathVec[trafficPersonVec[p].indexPathCurr];
  uint nextEdge = indexPathVec[trafficPersonVec[p].indexPathCurr + 1];
  if (trafficPersonVec[p].edgeNumLanes < 1 && nextEdge == -1) {
    return; // skip if reach the end or have no lane to change
  }

  if (trafficPersonVec[p].v > 3.0f && // at least 10km/h to try to change lane
      trafficPersonVec[p].num_steps % 5 ==
          0) { // just check every (5 steps) 5 seconds and make sure the agent
               // has enough speed
    // LC 1 update lane changing status
    if (trafficPersonVec[p].LC_stateofLaneChanging == 0) {
      // 2.2-exp((x-1)^2)
      float x = trafficPersonVec[p].posInLaneM / trafficPersonVec[p].length;

      if (x > 0.4f) { // just after 40% of the road
        float probabiltyMandatoryState = 2.2 - exp((x - 1) * (x - 1));

        // if (((float) qrand() / RAND_MAX) < probabiltyMandatoryState) {
        if ((((int)(x * 100) % 100) / 100.0f) <
            probabiltyMandatoryState) { // pseudo random number
          trafficPersonVec[p].LC_stateofLaneChanging = 1;
        }
      }
    }

    ////////////////////////////////////////////////////
    // LC 2 NOT MANDATORY STATE
    if (trafficPersonVec[p].LC_stateofLaneChanging == 0) {
      // discretionary change: v slower than the current road limit and
      // deccelerating and moving
      if ((trafficPersonVec[p].v <
           (trafficPersonVec[p].maxSpeedMperSec * 0.8f)) &&
          trafficPersonVec[p].v > 3.0f) {
        bool leftLane = trafficPersonVec[p].numOfLaneInEdge >
                        0; // at least one lane on the left
        bool rightLane =
            trafficPersonVec[p].numOfLaneInEdge <
            trafficPersonVec[p].edgeNumLanes - 1; // at least one lane

        if (leftLane && rightLane) {
          if (int(trafficPersonVec[p].v) % 2 ==
              0) { // pseudo random for change lane
            leftLane = false;
          }
        }

        ushort laneToCheck;
        if (leftLane) {
          laneToCheck = trafficPersonVec[p].numOfLaneInEdge - 1;
        } else {
          laneToCheck = trafficPersonVec[p].numOfLaneInEdge + 1;
        }

        uchar v_a, v_b;
        float gap_a, gap_b;
        // printf("p %u LC 1 %u\n",p,laneToCheck);
        uchar trafficLightState =
            trafficLights[currentEdge + trafficPersonVec[p].numOfLaneInEdge];

        calculateGapsLC(mapToReadShift, laneMap, trafficLightState, laneToCheck,
                        currentEdge, trafficPersonVec[p].posInLaneM,
                        trafficPersonVec[p].length, v_a, v_b, gap_a, gap_b);

        if (gap_a == 1000.0f && gap_b == 1000.0f) { // lag and lead car very far
          trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE

        } else { // NOT ALONE
          float b1A = 0.05f, b2A = 0.15f;
          float b1B = 0.15f, b2B = 0.40f;
          // simParameters.s_0-> critical lead gap
          float g_na_D, g_bn_D;
          bool acceptLC = true;

          if (gap_a != 1000.0f) {
            g_na_D = fmax(simParameters.s_0,
                          simParameters.s_0 + b1A * trafficPersonVec[p].v +
                              b2A * (trafficPersonVec[p].v - v_a * 3.0f));

            if (gap_a < g_na_D) { // gap smaller than critical gap
              acceptLC = false;
            }
          }

          if (acceptLC && gap_b != 1000.0f) {
            g_bn_D = fmax(simParameters.s_0,
                          simParameters.s_0 + b1B * v_b * 3.0f +
                              b2B * (v_b * 3.0f - trafficPersonVec[p].v));

            if (gap_b < g_bn_D) { // gap smaller than critical gap
              acceptLC = false;
            }
          }

          if (acceptLC) {
            trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
          }
        }
      }

    } // Discretionary
  }
}

__device__ void write2lane_map(LC::Agent *trafficPersonVec, int p,
                               LC::B18EdgeData *edgesData, uint *indexPathVec,
                               uchar *laneMap, uint mapToWriteShift) {
  int kMaxMapWidthM = 1024;
  uint currentEdge = indexPathVec[trafficPersonVec[p].indexPathCurr];
  uint nextEdge = indexPathVec[trafficPersonVec[p].indexPathCurr + 1];
  // write to the lanemap if still on the edge
  if (trafficPersonVec[p].posInLaneM <
      trafficPersonVec[p].length) { // does not reach an intersection
    uchar vInMpS =
        (uchar)(trafficPersonVec[p].v * 3); // speed in m/s to fit in uchar
    ushort posInLineCells = (ushort)(trafficPersonVec[p].posInLaneM);
    const uint posToSample =
        mapToWriteShift +
        kMaxMapWidthM * (currentEdge + trafficPersonVec[p].numOfLaneInEdge) +
        posInLineCells;
    laneMap[posToSample] = vInMpS;
    return;
  }
  // 2.2.1 find next edge
  auto numMToMove = trafficPersonVec[p].posInLaneM - trafficPersonVec[p].length;
  trafficPersonVec[p].dist_traveled += trafficPersonVec[p].length;
  atomicAdd(&(edgesData[indexPathVec[trafficPersonVec[p].indexPathCurr]]
                  .downstream_veh_count),
            1);

  if (nextEdge == -1) {             // if(curr_intersection==end_intersection)
    trafficPersonVec[p].active = 2; // finished
    return;
  }
  // move to the next edge
  trafficPersonVec[p].indexPathCurr++;
  trafficPersonVec[p].maxSpeedMperSec =
      trafficPersonVec[p].nextEdgemaxSpeedMperSec;
  trafficPersonVec[p].edgeNumLanes = trafficPersonVec[p].nextEdgeNumLanes;
  trafficPersonVec[p].edgeNextInters = trafficPersonVec[p].nextEdgeNextInters;
  trafficPersonVec[p].length = trafficPersonVec[p].nextEdgeLength;
  trafficPersonVec[p].posInLaneM = numMToMove;

  atomicAdd(&(edgesData[indexPathVec[trafficPersonVec[p].indexPathCurr]]
                  .upstream_veh_count),
            1);
  if (trafficPersonVec[p].numOfLaneInEdge >= trafficPersonVec[p].edgeNumLanes) {
    trafficPersonVec[p].numOfLaneInEdge =
        trafficPersonVec[p].edgeNumLanes -
        1; // change line if there are less roads
  }

  ////////////
  // update next edge
  uint nextNEdge = indexPathVec[trafficPersonVec[p].indexPathCurr + 1];
  // trafficPersonVec[p].nextEdge=nextEdge;
  if (nextNEdge != -1) {
    // trafficPersonVec[p].nextPathEdge++;
    trafficPersonVec[p].LC_initOKLanes = 0xFF;
    trafficPersonVec[p].LC_endOKLanes = 0xFF;

    // 2.2.3 update person edgeData
    // trafficPersonVec[p].nextEdge=nextEdge;
    trafficPersonVec[p].nextEdgemaxSpeedMperSec =
        edgesData[nextNEdge].maxSpeedMperSec;
    trafficPersonVec[p].nextEdgeNumLanes = edgesData[nextNEdge].numLines;
    trafficPersonVec[p].nextEdgeNextInters =
        edgesData[nextNEdge].nextIntersMapped;
    trafficPersonVec[p].nextEdgeLength = edgesData[nextNEdge].length;
  }

  trafficPersonVec[p].LC_stateofLaneChanging = 0;
  uchar vInMpS =
      (uchar)(trafficPersonVec[p].v * 3); // speed in m/s to fit in uchar
  ushort posInLineCells = (ushort)(trafficPersonVec[p].posInLaneM);
  const uint posToSample =
      mapToWriteShift +
      kMaxMapWidthM * (currentEdge + trafficPersonVec[p].numOfLaneInEdge) +
      posInLineCells;
  laneMap[posToSample] = vInMpS;
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

  // 1. initialization
  if (trafficPersonVec[p].active == 2) { // finished
    return;
  }
  // 1.1. check if person should still wait or should start
  if (trafficPersonVec[p].active == 0) {
    if (trafficPersonVec[p].time_departure > currentTime) { // wait
      return;
    } else { // its your turn
      initialize_agent(trafficPersonVec, p, edgesData, indexPathVec, laneMap,
                       simParameters, mapToReadShift, mapToWriteShift);
      return;
    }
  }

  // 2. Moving
  trafficPersonVec[p].num_steps++;
  trafficPersonVec[p].nextEdge =
      indexPathVec[trafficPersonVec[p].indexPathCurr + 1];

  // 2.1.1 Find front car
  float front_car_info[] = {0, 0};
  check_front_car(trafficPersonVec, p, indexPathVec, laneMap, deltaTime,
                  mapToReadShift, front_car_info);

  // 2.1.2 Update agent information using the front car info
  update_agent_info(trafficPersonVec, p, deltaTime, simParameters,
                    front_car_info);

  // 2.1.3 Perform lane changing if necessary
//  change_lane(trafficPersonVec, p, simParameters, indexPathVec, laneMap,
//              mapToReadShift, trafficLights);
  // 2.1.4 write the updated agent info to lanemap
  write2lane_map(trafficPersonVec, p, edgesData, indexPathVec, laneMap,
                 mapToWriteShift);

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

__global__ void
kernel_intersectionOneSimulation(uint numIntersections, float currentTime,
                                 LC::B18IntersectionData *intersections,
                                 uchar *trafficLights) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < numIntersections) {       // CUDA check (inside margins)
    const float deltaEvent = 20.0f; /// !!!!
    if (currentTime > intersections[i].nextEvent &&
        intersections[i].totalInOutEdges > 0) {

      uint edgeOT = intersections[i].edge[intersections[i].state];
      uchar numLinesO = edgeOT >> 24;
      uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF;

      // red old traffic lights
      if ((edgeOT & kMaskInEdge) ==
          kMaskInEdge) { // Just do it if we were in in
        for (int nL = 0; nL < numLinesO; nL++) {
          trafficLights[edgeONum + nL] = 0x00; // red old traffic light
        }
      }

      for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1;
           iN++) { // to give a round
        intersections[i].state = (intersections[i].state + 1) %
                                 intersections[i].totalInOutEdges; // next light

        if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) ==
            kMaskInEdge) { // 0x800000
          // green new traffic lights
          uint edgeIT = intersections[i].edge[intersections[i].state];
          uint edgeINum = edgeIT & kMaskLaneMap; //  0xFFFFF; //get edgeI
          uchar numLinesI = edgeIT >> 24;

          for (int nL = 0; nL < numLinesI; nL++) {
            trafficLights[edgeINum + nL] = 0xFF;
          }

          // trafficLights[edgeINum]=0xFF;
          break;
        }
      } // green new traffic light

      intersections[i].nextEvent = currentTime + deltaEvent;
    }
    //////////////////////////////////////////////////////
  }

} //

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
  if (readFirstMapC == true) {
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
  kernel_intersectionOneSimulation<<<ceil(numIntersections / 512.0f), 512>>>(
      numIntersections, currentTime, intersections_d, trafficLights_d);
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
  if ((((float)((int)currentTime)) == (currentTime)) &&
      ((int)currentTime % ((int)30)) == 0) { // 3min //(sample double each 3min)
    int samplingNumber = (currentTime - startTime) / (30 * numStepsTogether);
    uint offset = numIntersections * samplingNumber;
    // printf("Sample %d\n", samplingNumber);
    kernel_sampleTraffic<<<ceil(numPeople / 1024.0f), 1024>>>(
        numPeople, trafficPersonVec_d, indexPathVec_d,
        accSpeedPerLinePerTimeInterval_d, numVehPerLinePerTimeInterval_d,
        offset);
    gpuErrchk(cudaPeekAtLastError());
  }
} //
