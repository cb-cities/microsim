#ifndef LC_B18_TRAFFIC_PERSON_H
#define LC_B18_TRAFFIC_PERSON_H

#include "config.h"

namespace LC {
//! Agent Class
//! \brief Class that stores all the information about the agent
struct Agent {
  Agent() = default;
  // Constructor for the simulator class
  //! \param[in] srcvertex start node id
  //! \param[in] tgtvertex end node id
  //! \param[in] type agent type
  //! \param[in] dep_time departure time
  Agent(uint srcvertex, uint tgtvertex, AgentType type, float dep_time) {
    init_intersection = srcvertex;
    end_intersection = tgtvertex;
    time_departure = dep_time;
    agent_type = type;
    if (agent_type == CAR) {
      IDMParametersCar param;
      a = param.a;
      b = param.b;
      T = param.T;
      s_0 = param.s_0;
    }
  }

  // Route information
  unsigned int init_intersection;
  unsigned int end_intersection;
  float time_departure;
  //! Route of the agent (array of lanemap ids)
  uint route[500];
  unsigned short route_size{0};
  int route_ptr{-1};

  // Agent information
  AgentType agent_type;
  unsigned short active = 0; // 0 inactive 1 active 2 finished
                             //  int aid{-1};

  // IDM
  float v{0};         // current velocity
  float delta_v{-10}; // velocity difference to the front car
  float s{0};         // space to the front car
  float a;            // acceleration
  float b;            // break
  float T;            // Time heading
  float s_0;          // min car following dist
  float dv_dt{0};

  // Road (edge) information
  unsigned int edge_id;
  unsigned int edge_mid;
  float posInLaneM{-1};
  float max_speed{-1};
  float edge_length{-1};

  // Intersection information
  bool in_queue{false};
  unsigned int num_steps_in_queue{0};
  unsigned short lane{}; // number of lane in that edge
  int intersection_id{-1};
  int queue_idx{-1};


  // Simulation information
  float cum_length{0};
  unsigned int num_steps{0};
  unsigned int slow_down_steps{0};
  float cum_v{0}; // Cumulative velocity of each person across all iterations
  unsigned int num_lane_change = 0;
  unsigned int num_steps_entering_edge{0};
  unsigned int initial_waited_steps{0};
};

} // namespace LC

#endif // LC_B18_TRAFFIC_PERSON_H
