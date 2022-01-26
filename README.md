# MicroSim

MicroSim is an agent-based car following traffic simulator. It employs a highly parallelized GPU implementation that is fast enough to run simulations on large-scale demand and networks within a few minutes - metropolitan and regional scale with hundreds of thousands of nodes and edges and millions of trips

## Dependencies

 - CMAKE 3.10
 - Boost 1.65
 - CUDA (used versions: 10.0 in Ubuntu)
 - gcc (used versions: 7.5.0 in Ubuntu)
 - g++ (used versions: 7.5.0 in Ubuntu)
 - Qt5 (used versions: 5.9.7 in Ubuntu)
 - Python (used versions: 3.6.5 in Ubuntu)


## Installation & Compilation

Once the necessary dependencies are installed, add CUDA lib path to system paths:
```bash
export PATH=/usr/local/cuda-10.0/bin:$PATH
export LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LIBRARY_PATH 
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH 
```

You can also add the `export` lines at the end of your user's `~/.bashrc` to
avoid re-entering them in each session.

Clone the repo in your home directory with:
```bash
git clone https://github.com/cb-cities/microsim.git ~/microsim && cd ~/microsim
```
Create Makefile:
```bash
mkdir build && cd build && cmake ../src
```
Compile:
```bash
make clean && make -j8
```

## Design Principles

<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/high_level.png" alt="high_level" class="design-primary" width="600px">
</p>

There are three levels abstraction for the program: **Agent** **Intersection** **Edge** . 

An Agent is a vehicle with a certain type (only car at current version). An Intersection (node in the network) stores queues at all directions. An edge (link in the network) is the road that agents interact with each other. 

An agent exists **either** in an intersection **or** on an edge.

Agents interact with each other on an edge with **fixed simple rules** (IDM). 

Intersections are **source** or **sink** that feed/free agents into/from edges. 

It is possible to have different types of agent, and different types of interactions among them. (future works)

<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/computation_illustration.png" alt="high_level" class="design-primary" width="600px">
</p>


### Agent:
<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/agent.png" alt="high_level" class="design-primary" width="200px">
</p>

Property: 
IDM parameters: abT -> Can be different -> Car/Truck/Else 

Interaction Rules: 
IDM car following + Lane change. (All on the edge) 

<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/car_interactions.png" alt="high_level" class="design-primary" width="600px">
</p>

### Intersection (bundle of queues):
<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/intersection.png" alt="high_level" class="design-primary" width="600px">
</p>

Property: 
List of queues at every possible direction 

Interaction Rule (with edges):
Place a stop sign at the entrance if the queue is full (Upstream edge); Feed into downstream edge if it has enough space in a round robin fashion (Downstream edge). 


### Edge (bundle of lanes):
Property: 
Array of lanes with agents on them; upstream/downstream counts; average travel time (weight)

Interaction Rule (with intersections): 
Feed by the upstream intersection, supply agents to the downstream intersection. Queuing occurs if the corresponding queue in the downstream intersection is full. 


## Program Architecture
### Input Output (IO)
<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/io_illustration.png" alt="high_level" class="design-primary" width="600px">
</p>
**Input:**
nodes.csv 
Header: osmid,x,y,ref,highway,index
Note: index must be unique and sequential (from 0 to n) 
edges.csv 
Header: uniqueid,osmid_u,osmid_v,edge_length,lanes,speed_mph,u,v
Note: uniqueid must be unique! On default, each edge will be constructed twice (two directions); u ,v correspond to node index 
Od.csv
Header: origin,destination,dep_time
Note: origin, destination correspond to node index, dep_time is in seconds 

**Output:**
Edge_data_time.csv 
Header: eid,u,v,upstream_count,downstream_count,average_travel_time(s)
Agents_data_time.csv
Header: aid,ori,dest,type,status,travel_dist(m),travel_time(s),ave_speed(m/s),num_slowdown,num_lane_change,num_in_queue

**Simulation Configuration:**
Command_line_options.init 
NETWORK_PATH=../tests/scenarios/case1/
OD_PATH =../tests/scenarios/case1/od.csv
START=32400 
END=61200
SHOW_BENCHMARKS=false
SAVE_PATH=./case1_results/
SAVE_INTERVAL = 100
Note: all time unit is in second. 

**Parameter Configuration:**
struct IDMParametersCar {
  float a = 0.557040909258405;    // acceleration
  float b = 2.9020578588167;      // break
  float T = 0.5433027817144876;   // Time heading
  float s_0 = 1.3807498735425845; // min car following dis
};

### Submodules
#### Network  
All the information about the network is stored here. 
Initialization: 
Read node and edge data -> construct network using the sp code -> prepare initial edge weights (free flow travel time) 
Play with the network_test.cpp for a deeper understanding 

#### OD
All the information about the agent (origin, destination, vehicle type, departure time) is stored here 
Initialization: 
Read origin destination -> read departure time -> read agent types -> construct a vector of agents (defined in agent.h)
Play with the od_test.cpp for a deeper understanding 

#### Lanemap
Convert the 2d network to 1d lanemap (edge data; intersection data) for GPU access. 
Initialization (edge_data): 
Iterate through each edge -> copy the edge information from network (graph_) to lanemap (edgesData_) -> construct the correspondence id map (mid2eid_,eid2mid_) -> calculate the flattened length of the edge (number of cells used) -> go to the next edge 
Initialization (intersection_data): 
Iterate through each node ->  iterate each in_edge/out_edge pairs -> construct each in/out pair as a queue for the intersection
Note: each vertex (node) is an intersection. 
Play with the lanemap_test.cpp for a deeper understanding 

#### Simulator
Route_finding <-> simulation 

Route_finding:
Use the contraction hierarchy algorithm for route finding:
Initialize the ch using the network object -> go through all the agents to construct the od pairs -> run rh to find routes for each agent -> record each agent’s designed path (agent.route, a sequence of lanemap edge ids) 

SimulateInGPU: 
1. Allocate an appropriate amount of memory on the cuda device then copy the CPU data to there (init_cuda) 
2. From start_time to end_time, simulate the movement of each agent at the simulation time step (time resolution 0.5 second) (cuda_simulate)
3. Save the simulation results (time-snap for agents and edges) at each save_interval 
4. Free GPU memory (finish_cuda)

Cuda_simulate: 
1. Change map (switch the read and write map: read from the previous time-step map and write to an empty map) 
2. Simulate agents movements on edges (kernel_trafficSimulation)
3. Simulate agents movements on intersections (kernel_intersectionOneSimulation)

**kernel_trafficSimulation**
GPU parallel computation for each agent

<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/edge_simulation.png" alt="high_level" class="design-primary" width="600px">
</p>


**kernel_intersectionOneSimulation** 
GPU parallel computation for each intersection

<p align="center">
<img src="https://github.com/cb-cities/microsim/blob/master/figures/node_simulation.png" alt="high_level" class="design-primary" width="400px">
</p>




## Acknowledgments

This repository and code have been developed and maintained by Renjie Wu. This work heavily derives from Pavan Yedavalli's [Microsimulation Analysis for Network Traffic Assignment](https://github.com/UDST/manta/) and Ignacio Garcia Dorado's [Automatic Urban Modeling project](http://www.ignaciogarciadorado.com/p/2014_EG/2014_EG.html).






