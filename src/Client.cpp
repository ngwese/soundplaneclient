#include "Client.h"

#include <iostream>

Client::Client() {
  mpDriver = SoundplaneDriver::create(*this);
  mpDriver->start();
}

Client::~Client() { mpDriver = nullptr; }

void Client::onStartup() {
  // get serial number and auto calibrate noise on sync detect
  // const unsigned long instrumentModel = 1; // Soundplane A
  // mOSCOutput.setSerialNumber((instrumentModel << 16) |
  // mpDriver->getSerialNumber());

  // // connected but not calibrated -- disable output.
  // enableOutput(false);
  // // output will be enabled at end of calibration.
  // mNeedsCalibrate = true;
}

// we need to return as quickly as possible from driver callback.
// just put the new frame in the queue.
void Client::onFrame(const SensorFrame &frame) {
  // if(!mTestTouchesOn)
  // {
  // 	mSensorFrameQueue->push(frame);
  // }
}

void Client::onError(int error, const char *errStr) {
  std::cerr << "error: " << error << ", " << errStr << std::endl;
}

void Client::onClose() {}
