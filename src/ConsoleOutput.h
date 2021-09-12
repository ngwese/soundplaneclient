#ifndef __CONSOLE_OUTPUT_H__
#define __CONSOLE_OUTPUT_H__

#include "Output.h"

class ConsoleOutput : public Output {
  virtual void beginOutputFrame(time_point<system_clock> now) override;
  virtual void processTouch(int i, int offset, const Touch &t) override;
  virtual void processController(int zoneID, int offset, const ZoneMessage &m) override;
  virtual void endOutputFrame() override;
};

#endif