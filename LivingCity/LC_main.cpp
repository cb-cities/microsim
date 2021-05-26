#define BOOST_TT_HAS_OPERATOR_HPP_INCLUDED

#include "src/benchmarker.h"
#include "src/linux_host_memory_logger.h"
#include "traffic/b18CommandLineVersion.h"
#include <QDebug>

// NOTE: Check command_line_options for default options.

int main(int argc, char *argv[]) {

  mainBench.startMeasuring();

  QCoreApplication a(argc, argv);
  QSettings settings(QCoreApplication::applicationDirPath() +
                         "/command_line_options.ini",
                     QSettings::IniFormat);

  LC::B18CommandLineVersion cl;
  cl.runB18Simulation();
  printf(">>Simulation Ended\n");

  mainBench.stopAndEndBenchmark();
  intersectionBench.endBenchmark();
  peopleBench.endBenchmark();

//  memory_logger.End();
  return 0;
}
