#include <iostream>

#include "ConsoleOutput.h"
#include "Logging.h"

void ConsoleOutput::beginOutputFrame(time_point<system_clock> now) {
  MLConsole() << "begin: " /*<< now*/ << std::endl;
}

void ConsoleOutput::processTouch(int i, int offset, const Touch &t) {
  MLConsole() << "   t: " << i << " " << offset /*<< " " << t*/ << std::endl;
}

void ConsoleOutput::processController(int zoneID, int offset, const ZoneMessage &m) {
  MLConsole() << "   c: " << zoneID << " " << offset /*<< " " << m*/ << std::endl;
}

void ConsoleOutput::endOutputFrame() {
  MLConsole() << "end" << std::endl;
}
