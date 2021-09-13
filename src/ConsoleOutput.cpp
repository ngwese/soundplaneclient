#include <iostream>

#include "ConsoleOutput.h"
#include "Logging.h"

void ConsoleOutput::beginOutputFrame(time_point<system_clock> now) {
  if (mFrames == 0) {
    mIntervalStart = std::chrono::system_clock::now();
  }

  mFrames++;

  if (mFrames % mOutputInterval == 0) {
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<seconds>(now - mIntervalStart).count();
    MLConsole() << "begin[" << mFrames << "], " << mOutputInterval / elapsed << " fr/sec" << std::endl;
    mIntervalStart = now;
  }
}

void ConsoleOutput::processTouch(int i, int offset, const Touch &t) {
  if (mFrames % 2 == 0) {
    if (touchIsActive(t)) {
      MLConsole() << "   t: " << i << " " << offset
        << " x: " << t.x
        << " y: " << t.y
        << " z: " << t.z
        << " state: " << t.state
        // << " kx: " << t.kx
        // << " ky: " << t.ky
        << " n: " << t.note
        // << " vib: " << t.vibrato
        // << " idx: " << t.voiceIdx
      /*<< " " << t*/ << std::endl;
    }
  }
}

void ConsoleOutput::processController(int zoneID, int offset, const ZoneMessage &m) {
  if (mFrames % 10 == 0) {
    MLConsole() << "   c: " << zoneID << " " << offset /*<< " " << m*/ << std::endl;
  }
}

void ConsoleOutput::endOutputFrame() {
  if (mFrames % mOutputInterval == 0) {
    MLConsole() << "end" << std::endl;
  }
}
