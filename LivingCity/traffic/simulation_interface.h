#pragma once

#include <string>
#include <QSettings>
#include <qt5/QtCore/qcoreapplication.h>

#include "traffic/traffic_simulator.h"
//#include "traffic/b18TrafficSP.h"

namespace LC {

class SimulationInterface{
  public:
    void run_simulation();
};

}  // namespace LC
