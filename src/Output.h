#ifndef __OUTPUT_H__
#define __OUTPUT_H__

#include "MLMatrix.h"

#include "Touch.h"
#include "Zone.h"

namespace soundplane {

class Output {
  public:
  virtual void beginOutputFrame(std::chrono::time_point<std::chrono::system_clock> now) = 0;
  virtual void processTouch(int i, int offset, const Touch &t) = 0;
  virtual void processController(int zoneID, int offset, const ZoneMessage &m) = 0;
  virtual void endOutputFrame() = 0;

  virtual void processMatrix(const ml::Matrix&) {};

  virtual bool isActive() { return true; };

  virtual void setMaxTouches(int) {};
  virtual void setDataRate(int) {};

  virtual void doInfrequentTasks() {};
};

} // namespace soundplane

#endif