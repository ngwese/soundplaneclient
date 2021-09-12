#ifndef __OUTPUT_H__
#define __OUTPUT_H__

#include "Touch.h"
#include "Zone.h"

class Output {
  virtual void beginOutputFrame(std::chrono::time_point<std::chrono::system_clock> now) = 0;
  virtual void processTouch(int i, int offset, const Touch &t) = 0;
  virtual void processController(int zoneID, int offset, const ZoneMessage &m) = 0;
  virtual void endOutputFrame() = 0;
};

#endif