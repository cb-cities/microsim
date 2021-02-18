#ifndef _ABM_CONFIG_H_
#define _ABM_CONFIG_H_

namespace abm {
namespace graph {
//! Vertex id type
using vertex_t = long long;
using edge_id_t = long long;
//! Weight type, that can be added with +
using weight_t = double;
}  // namespace graph
}  // namespace abm

struct IDMParameters{
    float a = 0.557040909258405;
    float b = 2.9020578588167;
    float T = 0.5433027817144876;
    float s_0 = 1.3807498735425845;
    float dt = 0.5;
};
#endif  // _ABM_CONFIG_H_


