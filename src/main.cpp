#include <chrono>
#include <memory>
#include <thread>

#include "Client.h"
#include "Logging.h"
#include "MLProjectInfo.h"

int main(int argc, char *argv[]) {
  MLConsole() << "starting client v" << MLProjectInfo::versionString << "\n";
  ml::SharedResourcePointer<ml::Timers> t;
  t->start(true);

  std::unique_ptr<Client> client = std::unique_ptr<Client>(new Client());

  std::this_thread::sleep_for(std::chrono::seconds(60));

  MLConsole() << "quitting\n";
}