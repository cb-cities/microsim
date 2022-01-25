#ifndef MICROSIM_OD_H
#define MICROSIM_OD_H
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agent.h"
#include "config.h"

namespace LC {
//! Origin Destination Class
//! \brief Base class for origin and destination (encoded as agents)
class OD {
public:
  // Constructor with odfile path
  //! \param[in] odFileName od file path
  explicit OD(const std::string &odFileName);

  //! Destructor
  ~OD() = default;

  //! Get agents
  std::vector<Agent> &agents() { return agents_; }

  //! Number of agents
  const unsigned int num_agents(){return num_agents_;}

private:
  //! agents (vehicles)
  std::vector<Agent> agents_;

  unsigned int num_agents_;

  //! Parse OD pairs from the input file
  //! \param[in] odFileName od file path
  //! \retval vector of od pairs
  std::vector<std::vector<unsigned int>>
  read_od_pairs_(const std::string &odFileName);

  //! Parse departure times from the input file
  //! \param[in] odFileName od file path
  //! \retval vector of departure times
  std::vector<float> read_dep_times_(const std::string &odFileName);

  //! Parse agent types from the input file
  //! \param[in] odFileName od file path
  //! \retval vector of agent types
  std::vector<AgentType> read_agent_types_(const std::string &odFileName);
};

} // namespace LC

#endif // MICROSIM_OD_H
