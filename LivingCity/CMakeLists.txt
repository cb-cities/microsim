project(microsim LANGUAGES CXX )

# Require C++17-compliant compiler; only available for CMake v. 3.1 and up
set(CMAKE_CXX_STANDARD 17)

cmake_minimum_required(VERSION 3.1)

SET(CMAKE_COLOR_MAKEFILE ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# General compile settings
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Debug")
    # SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

# GNU Specific settings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -pthread")
endif()

# Clang Specific settings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -pthread")
endif()

option(MICROSIM_TEST_BUILD_TESTING "enable testing for microsim" ON)

# CMake seems to have no way to enable/disable testing per subproject,
# so we provide an option similar to BUILD_TESTING, but just for CITYSCAPE.
#option(PIPE_NETWORK_TEST_BUILD_TESTING "enable testing for cityscape" ON)

# CMake Modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# microsim executable
SET(microsim_src
        traffic/sp/graph.cc
        traffic/network.cpp
        traffic/od.cpp
#        traffic/simulation_interface.cpp
#        traffic/traffic_simulator.cpp
#        traffic/lanemap.cpp
#        src/benchmarker.cpp

        RoadGraph/roadGraph.cpp
        RoadGraph/roadGraphEdge.cpp
        RoadGraph/roadGraphVertex.cpp
        Geometry/block.cpp
        Geometry/building.cpp
        Geometry/client_geometry.cpp
        Geometry/parcel.cpp
        Geometry/parcelBuildingAttributes.cpp
        Geometry/placeTypeInstances.cpp
        Geometry/zone.cpp
        misctools/bounding_box.cpp
        misctools/misctools.cpp
        misctools/polygon_3D.cpp
        global.cpp
#        RoadGraph/roadGraph.cpp
#        RoadGraph/roadGraphEdge.cpp
#        RoadGraph/roadGraphVertex.cpp


#        traffic/b18GridPollution.cpp
#        traffic/b18TrafficDijkstra.cpp
#        traffic/b18TrafficJohnson.cpp
#        traffic/b18TrafficSP.cpp
#        traffic/b18TrafficLaneMap.cpp
#        traffic/b18TrafficOD.cpp
#
#
#        src/linux_host_memory_logger.cpp

        )

#        Geometry/block.cpp
#        Geometry/building.cpp
#        Geometry/client_geometry.cpp
#        Geometry/parcel.cpp
#        Geometry/parcelBuildingAttributes.cpp
#        Geometry/placeTypeInstances.cpp
#        Geometry/zone.cpp
#        RoadGraph/roadGraph.cpp
#        RoadGraph/roadGraphEdge.cpp
#        RoadGraph/roadGraphVertex.cpp
#        global.cpp
#        misctools/bounding_box.cpp
#        misctools/misctools.cpp
#        misctools/polygon_3D.cpp
#
#        traffic/b18GridPollution.cpp
#        traffic/b18TrafficDijkstra.cpp
#        traffic/b18TrafficJohnson.cpp
#        traffic/b18TrafficSP.cpp
#        traffic/b18TrafficLaneMap.cpp
#        traffic/b18TrafficOD.cpp
#        traffic/b18TrafficSimulator.cpp
#        src/benchmarker.cpp
#        src/linux_host_memory_logger.cpp
#        )

# Find dependencies for the library

# Pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads)

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()

# OpenCV
find_package(OpenCV REQUIRED )

# QT5
find_package(Qt5Widgets REQUIRED)

# Boost
find_package(Boost REQUIRED)

# Cuda
find_package(CUDA REQUIRED)



# contraction hierachy lib
find_library(CHROUTING_LIB chrouting ${microsim_SOURCE_DIR}/pandana_ch/)

find_library(KITROUTING_LIB routingkit ${microsim_SOURCE_DIR}/RoutingKit/lib)

# include directories
add_library(lmicrosim SHARED ${microsim_src})
target_include_directories(lmicrosim
        PUBLIC
        ${microsim_SOURCE_DIR}
        ${microsim_SOURCE_DIR}/traffic/
        ${microsim_SOURCE_DIR}/external/
        ${microsim_SOURCE_DIR}/glew/include/
        ${microsim_SOURCE_DIR}/RoutingKit/include/
        ${OpenCV_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        )

# link libs
target_link_libraries(lmicrosim
        PUBLIC
        ${CMAKE_THREAD_LIBS_INIT}
        ${LAPACK_LIBRARIES}
        ${OpenCV_LIBS}
        ${CHROUTING_LIB}
        ${KITROUTING_LIB}
        boost_system boost_filesystem
        Qt5::Widgets)


cuda_add_library(lmicrosim_cuda
        traffic/simulation_interface.h
        traffic/b18CUDA_trafficSimulator.cu
        OPTIONS -arch sm_50)

target_link_libraries(lmicrosim_cuda ${CUDA_LIBRARIES})

target_compile_options(lmicrosim_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
        --compiler-options
        -fno-strict-aliasing
        -use_fast_math
        --ptxas-options=-v
        -Xcompiler
        -fopenmp)


add_executable(microsim ${microsim_SOURCE_DIR}/LC_main.cpp)
target_link_libraries(microsim
        PUBLIC
        lmicrosim
        lmicrosim_cuda)

include_directories(SYSTEM ${microsim_SOURCE_DIR} Qt5::Widgets)

# Testing
if(MICROSIM_TEST_BUILD_TESTING)
    # Unit test
    SET(microsim_test_src
            tests/network_test.cpp
            tests/od_test.cpp
#            tests/traffic_simulator_test.cpp
#            tests/lanemap_test.cpp
#            tests/scenario_testing.cpp
            )

    add_executable(microsim_test tests/main_test.cpp ${microsim_test_src})
    target_link_libraries(microsim_test lmicrosim)
    add_test(NAME microsim_test COMMAND $<TARGET_FILE:microsim_test>)
    enable_testing()
endif()