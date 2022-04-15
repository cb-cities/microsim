#ifndef _ABM_CONFIG_H_
#define _ABM_CONFIG_H_

namespace abm {
namespace graph {
//! Vertex id type
using vertex_t = long long;
using edge_id_t = long long;
//! Weight type, that can be added with +
using weight_t = double;
} // namespace graph
} // namespace abm

namespace LC {
enum AgentType { CAR };

//! Number of threads per cuda block
const int CUDAThreadsPerBlock {384};
//! Length of each cell in the lane map
const int kMaxMapWidthM_{1024}; // size of the bin
//! Safe space between cars
const int SOCIAL_DIST{3};
//! Initial speed (m/s)
const float INIT_SPEED{2};
//! Initial queue cap (for agent initialization)
const short INIT_QUEUE_CAP{10};


struct IDMParametersCar {
  float a = 0.557040909258405;    // acceleration
  float b = 2.9020578588167;      // break
  float T = 0.5433027817144876;   // Time heading
  float s_0 = 3.3807498735425845; // min car following dis
};

} // namespace LC

#endif // _ABM_CONFIG_H_
