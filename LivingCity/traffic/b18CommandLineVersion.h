#pragma once

#include <string>
#include <QSettings>
#include <qt5/QtCore/qcoreapplication.h>

#include "traffic/traffic_simulator.h"
#include "Geometry/client_geometry.h"

#include "network.h"
#include "pandana_ch/accessibility.h"
#include "sp/graph.h"
#include "src/benchmarker.h"
//#include "traffic/b18TrafficSP.h"

namespace LC {

class B18CommandLineVersion{
  public:
    void runB18Simulation();
};

}  // namespace LC
