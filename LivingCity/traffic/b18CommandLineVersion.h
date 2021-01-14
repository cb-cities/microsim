#pragma once

#include <qt5/QtCore/QString>
#include <string>
#include <qt5/QtCore/QSettings>

#include "traffic/b18TrafficSimulator.h"
//#include "Geometry/client_geometry.h"
#include "qcoreapplication.h"
#include "../roadGraphB2018Loader.h"
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
