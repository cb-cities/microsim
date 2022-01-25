#include "od.h"
#include "network.h"

namespace LC {
LC::OD::OD(const std::string &odFileName) {
  auto od_pairs = read_od_pairs_(odFileName);
  auto dep_times = read_dep_times_(odFileName);
  auto agent_types = read_agent_types_(odFileName);
  num_agents_ = od_pairs.size();

  for (int j = 0; j < num_agents_; ++j) {
    auto od_pair = od_pairs[j];
    auto agent_type = agent_types[j];
    auto dep_time = dep_times[j];
    Agent agent(od_pair[0], od_pair[1], agent_type, dep_time);
    agents_.emplace_back(agent);
  }
}

std::vector<std::vector<unsigned int>>
OD::read_od_pairs_(const std::string &odFileName) {
  std::vector<std::vector<unsigned int>> od_pairs;
  try {
    csvio::CSVReader<2> in(odFileName);
    in.read_header(csvio::ignore_extra_column, "origin", "destination");
    abm::graph::vertex_t v1, v2;

    while (in.read_row(v1, v2)) {
      std::vector<unsigned int> od = {v1, v2};
      od_pairs.emplace_back(od);
    }
    return od_pairs;
  } catch (std::exception &exception) {
    std::cout << "Read OD pairs: " << exception.what() << "\n";
    abort();
  }
}

std::vector<float> OD::read_dep_times_(const std::string &odFileName) {
  std::vector<float> dep_times;
  try {
    csvio::CSVReader<1> in(odFileName);
    in.read_header(csvio::ignore_extra_column, "dep_time");
    float dep_time;
    while (in.read_row(dep_time)) {
      // printf("dep time %f\n", dep_time);
      dep_times.emplace_back(dep_time);
    }
    return dep_times;
  } catch (std::exception &exception) {
    std::cout << "Read departure time: " << exception.what() << "\n";
    abort();
  }
}

std::vector<AgentType> OD::read_agent_types_(const std::string &odFileName) {
  std::vector<AgentType> types;
  try {
    csvio::CSVReader<1> in(odFileName);
    in.read_header(csvio::ignore_extra_column, "dep_time");
    float dep_time;
    while (in.read_row(dep_time)) {
      // printf("dep time %f\n", dep_time);
      types.emplace_back(CAR);
    }
    return types;
  } catch (std::exception &exception) {
    std::cout << "Read agent type: " << exception.what() << "\n";
    abort();
  }
}

} // namespace LC
