#ifndef __CONSOLE_OUTPUT_H__
#define __CONSOLE_OUTPUT_H__

#include <chrono>

#include "Output.h"

class ConsoleOutput : public Output {
public:
  ConsoleOutput(unsigned int interval) : mOutputInterval(interval), mFrames(0) {};

  virtual void beginOutputFrame(std::chrono::time_point<std::chrono::system_clock> now) override;
  virtual void processTouch(int i, int offset, const Touch &t) override;
  virtual void processController(int zoneID, int offset,
                                 const ZoneMessage &m) override;
  virtual void endOutputFrame() override;

private:
  unsigned int mOutputInterval;
  unsigned int mFrames;
  std::chrono::time_point<std::chrono::system_clock> mIntervalStart;
};

#endif