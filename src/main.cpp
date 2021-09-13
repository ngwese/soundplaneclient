#include <chrono>
#include <memory>
#include <thread>

#include "Client.h"
#include "ConsoleOutput.h"
#include "Logging.h"
#include "MLProjectInfo.h"

int main(int argc, char *argv[]) {
  MLConsole() << "starting client v" << MLProjectInfo::versionString << "\n";
  ml::SharedResourcePointer<ml::Timers> t;
  t->start(true);

  ConsoleOutput output(1000);
  Client client(output);

  std::this_thread::sleep_for(std::chrono::seconds(600));

  MLConsole() << "quitting\n";
}
