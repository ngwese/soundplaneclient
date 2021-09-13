#include <chrono>
#include <memory>
#include <thread>

#include "Client.h"
#include "ConsoleOutput.h"
#include "Logging.h"
#include "MLProjectInfo.h"
#include "Zone.h"

namespace sp = soundplane;

int main(int argc, char *argv[]) {
  sp::Console() << "starting client v" << MLProjectInfo::versionString << "\n";

  ml::Timers timers;
  timers.start(true);

  sp::ConsoleOutput output(1000);
  sp::Client client(output);
  client.setProperty("max_touches", 8);
  client.setZone(sp::Zone::presetChromatic());

  std::this_thread::sleep_for(std::chrono::seconds(600));

  sp::Console() << "quitting\n";
}
