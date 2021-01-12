QT += core

# Project build directories
DESTDIR     = $$PWD
OBJECTS_DIR = $$DESTDIR/obj

unix {
    LIBS += -L/opt/local/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lchrouting
    INCLUDEPATH += \
      /usr/include/opencv2 \
      /opt/local/include/ \ 
      /usr/local/boost_1_59_0/ \
      $$PWD/glew/include/

    CONFIG += debug
}

HEADERS += \
    Geometry/block.h \
    Geometry/building.h \
    Geometry/client_geometry.h \
    Geometry/parcel.h \
    Geometry/parcelBuildingAttributes.h \
    Geometry/placeTypeInstances.h \
    Geometry/zone.h \
    RoadGraph/roadGraph.h \
    RoadGraph/roadGraphEdge.h \
    RoadGraph/roadGraphVertex.h \
    global.h \
    misctools/bounding_box.h \
    misctools/common.h \
    misctools/misctools.h \
    misctools/polygon_3D.h \
    roadGraphB2018Loader.h \
    traffic/b18CUDA_trafficSimulator.h \
    traffic/b18CommandLineVersion.h \
    traffic/b18EdgeData.h \
    traffic/b18GridPollution.h \
    traffic/b18TrafficDijkstra.h \
    traffic/b18TrafficJohnson.h \
    traffic/b18TrafficSP.h \
    traffic/b18TrafficLaneMap.h \
    traffic/b18TrafficOD.h \
    traffic/b18TrafficPerson.h \
    traffic/b18TrafficSimulator.h \
    src/benchmarker.h \
    src/linux_host_memory_logger.h \
    traffic/sp/config.h \
    traffic/sp/external/csv.h \
    traffic/sp/graph.h \
    traffic/sp/mpi_wrapper.h \
    traffic/sp/unordered_map_tuple_hash.h \
    traffic/sp/external/tsl/robin_growth_policy.h \
    traffic/sp/external/tsl/robin_hash.h \
    traffic/sp/external/tsl/robin_map.h \
    traffic/sp/external/tsl/robin_set.h \
    pandana_ch/accessibility.h\

SOURCES += \
    Geometry/block.cpp \
    Geometry/building.cpp \
    Geometry/client_geometry.cpp \
    Geometry/parcel.cpp \
    Geometry/parcelBuildingAttributes.cpp \
    Geometry/placeTypeInstances.cpp \
    Geometry/zone.cpp \
    LC_main.cpp \
    RoadGraph/roadGraph.cpp \
    RoadGraph/roadGraphEdge.cpp \
    RoadGraph/roadGraphVertex.cpp \
    global.cpp \
    misctools/bounding_box.cpp \
    misctools/misctools.cpp \
    misctools/polygon_3D.cpp \
    roadGraphB2018Loader.cpp \
    traffic/b18CommandLineVersion.cpp \
    traffic/b18GridPollution.cpp \
    traffic/b18TrafficDijkstra.cpp \
    traffic/b18TrafficJohnson.cpp \
    traffic/b18TrafficSP.cpp \
    traffic/b18TrafficLaneMap.cpp \
    traffic/b18TrafficOD.cpp \
    traffic/b18TrafficSimulator.cpp \
    src/benchmarker.cpp \
    src/linux_host_memory_logger.cpp \
    traffic/sp/graph.cc

OTHER_FILES += \
        traffic/b18CUDA_trafficSimulator.cu \


###################################################################
## CUDA
###################################################################

unix {
  # Cuda sources
  CUDA_SOURCES += traffic/b18CUDA_trafficSimulator.cu
  # Path to cuda toolkit install
  CUDA_DIR = /usr/local/cuda-9.2
  INCLUDEPATH += $$CUDA_DIR/include
  QMAKE_LIBDIR += $$CUDA_DIR/lib64
  # GPU architecture
  CUDA_ARCH = sm_50
  # NVCC flags
  NVCCFLAGS = --compiler-options -fno-strict-aliasing -use_fast_math --ptxas-options=-v -Xcompiler -fopenmp
  # Path to libraries
  LIBS += -lcudart -lcuda -lgomp
  QMAKE_CXXFLAGS += -fopenmp
  #LIBS += -fopenmp
  # join the includes in a line
  CUDA_INC = $$join(INCLUDEPATH,' -I','-I',' ')
  cuda.commands = $$CUDA_DIR/bin/nvcc -m64 -O3 -arch=$$CUDA_ARCH -c $$NVCCFLAGS $$CUDA_INC $$LIBS ${QMAKE_FILE_NAME} -o ${QMAKE_FILE_OUT}
  cuda.dependcy_type = TYPE_C
  cuda.depend_command = $$CUDA_DIR/bin/nvcc -O3 -M $$CUDA_INC $$NVCCFLAGS      ${QMAKE_FILE_NAME}

  cuda.input = CUDA_SOURCES
  cuda.output = ${OBJECTS_DIR}${QMAKE_FILE_BASE}_cuda.o
  # Tell Qt that we want add more stuff to the Makefile
  QMAKE_EXTRA_COMPILERS += cuda
}
