
// Part of the Soundplane client software by Madrona Labs.
// Copyright (c) 2013 Madrona Labs LLC. http://www.madronalabs.com
// Distributed under the MIT license: http://madrona-labs.mit-license.org/

#include "Client.h"
#include "MLProjectInfo.h"
#include "SensorFrame.h"
#include "SoundplaneModelA.h"
#include "ThreadUtility.h"

#include "Logging.h"

using namespace soundplane;

const int kModelDefaultCarriersSize = 40;
const unsigned char kModelDefaultCarriers[kModelDefaultCarriersSize] = {
  // 40 default carriers.  avoiding 32 (gets aliasing from 16)
  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
  17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
  31, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43
};

// make one of the possible standard carrier sets, skipping a range of carriers
// out of the middle of the 40 defaults.
//
static const int kStandardCarrierSets = 16;
static void makeStandardCarrierSet(SoundplaneDriver::Carriers &carriers,
                                   int set) {
  int startOffset = 2;
  int skipSize = 2;
  int gapSize = 4;
  int gapStart = set * skipSize + startOffset;
  carriers[0] = carriers[1] = 0;
  for (int i = startOffset; i < gapStart; ++i) {
    carriers[i] = kModelDefaultCarriers[i];
  }
  for (int i = gapStart; i < kSoundplaneNumCarriers; ++i) {
    carriers[i] = kModelDefaultCarriers[i + gapSize];
  }
}

void touchArrayToFrame(const TouchArray *pArray, ml::Matrix *pFrame) {
  // get references for syntax
  const TouchArray &array = *pArray;
  ml::Matrix &frame = *pFrame;

  for (int i = 0; i < kMaxTouches; ++i) {
    Touch t = array[i];
    frame(xColumn, i) = t.x;
    frame(yColumn, i) = t.y;
    frame(zColumn, i) = t.z;
    frame(dzColumn, i) = t.dz;
    frame(ageColumn, i) = t.age;
  }
}

ml::Matrix sensorFrameToSignal(const SensorFrame &f) {
  ml::Matrix out(SensorGeometry::width, SensorGeometry::height);
  for (int j = 0; j < SensorGeometry::height; ++j) {
    const float *srcStart = f.data() + SensorGeometry::width * j;
    const float *srcEnd = srcStart + SensorGeometry::width;
    std::copy(srcStart, srcEnd, out.getBuffer() + out.row(j));
  }
  return out;
}

SensorFrame signalToSensorFrame(const ml::Matrix &in) {
  SensorFrame out;
  for (int j = 0; j < SensorGeometry::height; ++j) {
    const float *srcStart = in.getConstBuffer() + (j << in.getWidthBits());
    const float *srcEnd = srcStart + in.getWidth();
    std::copy(srcStart, srcEnd, out.data() + SensorGeometry::width * j);
  }
  return out;
}

// --------------------------------------------------------------------------------
//
#pragma mark Client

Client::Client(Output &output)
    : mOutputEnabled(false), mOutput(output),
      // mSurface(SensorGeometry::width, SensorGeometry::height),
      // mRawSignal(SensorGeometry::width, SensorGeometry::height),
      // mCalibratedSignal(SensorGeometry::width, SensorGeometry::height),
      // mSmoothedSignal(SensorGeometry::width, SensorGeometry::height),
      mCalibrating(false), mTestTouchesOn(false), mTestTouchesWasOn(false),
      mSelectingCarriers(false), mHasCalibration(false),
      mZoneIndexMap(kSoundplaneAKeyWidth, kSoundplaneAKeyHeight),
      mHistoryCtr(0), mCarrierMaskDirty(false), mNeedsCarriersSet(false),
      mNeedsCalibrate(false), mLastInfrequentTaskTime(0),
      mCarriersMask(0xFFFFFFFF), mDoOverrideCarriers(false)
{
  mpDriver = SoundplaneDriver::create(*this);

  for (int i = 0; i < kMaxTouches; ++i) {
    mCurrentKeyX[i] = -1;
    mCurrentKeyY[i] = -1;
  }

  // setup default carriers in case there are no saved carriers
  for (int car = 0; car < kSoundplaneNumCarriers; ++car) {
    mCarriers[car] = kModelDefaultCarriers[car];
  }

  clearZones();
  setAllPropertiesToDefaults();

  // mTouchFrame.setDims(kSoundplaneTouchWidth, kMaxTouches);
  // mTouchHistory.setDims(kSoundplaneTouchWidth, kMaxTouches,
  //                       kSoundplaneHistorySize);

  // make zone presets collection
  // File zoneDir = getDefaultFileLocation(kPresetFiles,
  // MLProjectInfo::makerName,
  // MLProjectInfo::projectName).getChildFile("ZonePresets"); debug() <<
  // "LOOKING for zones in " << zoneDir.getFileName() << "\n"; mZonePresets =
  // std::unique_ptr<MLFileCollection>(new MLFileCollection("zone_preset",
  // zoneDir, "json")); mZonePresets->processFilesImmediate();
  // mZonePresets->dump();

  // now that the driver is active, start polling for changes in properties
  mTerminating = false;

  startModelTimer();

  mSensorFrameQueue = std::unique_ptr<Queue<SensorFrame>>(
      new Queue<SensorFrame>(kSensorFrameQueueSize));

  mProcessThread = std::thread(&Client::processThread, this);
  SetPriorityRealtimeAudio(mProcessThread.native_handle());

  mpDriver->start();
}

Client::~Client() {
  // signal threads to shut down
  mTerminating = true;

  if (mProcessThread.joinable()) {
    mProcessThread.join();
    Console() << "Client: mProcessThread terminated.\n";
  }

  mpDriver = nullptr;
}

void Client::doPropertyChangeAction(ml::Symbol p,
                                             const ml::Value &newVal) {
  Console() << "Client::doPropertyChangeAction: " << p << " -> " <<
  newVal << "\n";

  int propertyType = newVal.getType();
  switch (propertyType) {
  case ml::Value::kFloatValue: {
    float v = newVal.getFloatValue();
    if (ml::textUtils::stripFinalNumber(p) == ml::Symbol("carrier_toggle")) {
      // toggles changed -- mute carriers
      unsigned long mask = 0;
      for (int i = 0; i < 32; ++i) {
        ml::Symbol tSym =
            ml::textUtils::addFinalNumber(ml::Symbol("carrier_toggle"), i);
        bool on = (int)(getFloatProperty(tSym));
        mask = mask | (on << i);
      }

      mCarriersMask = mask;
      mCarrierMaskDirty = true; // trigger carriers set in a second or so
    }
    else if (p == "all_toggle") {
      bool on = (bool)(v);
      for (int i = 0; i < 32; ++i) {
        ml::Symbol tSym =
            ml::textUtils::addFinalNumber(ml::Symbol("carrier_toggle"), i);
        setProperty(tSym, on);
      }
      mCarriersMask = on ? ~0 : 0;
      mCarrierMaskDirty = true; // trigger carriers set in a second or so
    }
    else if (p == "max_touches") {
      mMaxTouches = v;
      mOutput.setMaxTouches(v);
    }
    else if (p == "lopass_z") {
      mTracker.setLopassZ(v);
    }
    else if (p == "z_thresh") {
      mTracker.setThresh(v);
    }
    else if (p == "snap") {
      sendParametersToZones();
    }
    else if (p == "vibrato") {
      sendParametersToZones();
    }
    else if (p == "lock") {
      sendParametersToZones();
    }
    else if (p == "data_rate") {
      mDataRate = v;
      mOutput.setDataRate(v);
    }
    else if (p == "quantize") {
      sendParametersToZones();
    }
    else if (p == "rotate") {
      bool b = v;
      mTracker.setRotate(b);
    }
    else if (p == "transpose") {
      sendParametersToZones();
    }
    else if (p == "verbose") {
      bool b = v;
      mVerbose = b;
    }
    else if (p == "override_carriers") {
      bool b = v;
      mDoOverrideCarriers = b;
      mNeedsCarriersSet = true;
    }
    else if (p == "override_carrier_set") {
      makeStandardCarrierSet(mOverrideCarriers, v);
      mNeedsCarriersSet = true;
    }
    else if (p == "test_touches") {
      bool b = v;
      Console() << "test touches: " << b << "\n";
      mTestTouchesOn = b;
      mSensorFrameQueue->clear();
      mRequireSendNextFrame = true;
    }
  } break;
  case ml::Value::kTextValue:
    break;
  case ml::Value::kMatrixValue: {
    const ml::Matrix &sig = newVal.getMatrixValue();
    if (p == ml::Symbol("carriers")) {
      // get carriers from signal
      assert(sig.getSize() == kSoundplaneNumCarriers);
      for (int i = 0; i < kSoundplaneNumCarriers; ++i) {
        if (mCarriers[i] != sig[i]) {
          mCarriers[i] = sig[i];
          mNeedsCarriersSet = true;
        }
      }
    }
  } break;
  default:
    break;
  }
}

void Client::onStartup() {
  Console() << "onStartup() called\n";
  // get serial number and auto calibrate noise on sync detect
  const unsigned long instrumentModel = 1; // Soundplane A
  mSerialNumber = mpDriver->getSerialNumber();

  Console() << "     state: " << mpDriver->getDeviceState() << "\n";
  Console() << "  firmware: " << mpDriver->getFirmwareVersion() << "\n";
  Console() << "    serial: " << mSerialNumber << "\n";
  Console() << "  carriers: " << mpDriver->getCarriers() << "\n";

  // connected but not calibrated -- disable output.
  enableOutput(false);
  // output will be enabled at end of calibration.
  mNeedsCalibrate = true;
}

// we need to return as quickly as possible from driver callback.
// just put the new frame in the queue.
void Client::onFrame(const SensorFrame &frame) {
  if (!mTestTouchesOn) {
    mSensorFrameQueue->push(frame);
  }
}

void Client::onError(int error, const char *errStr) {
  switch (error) {
  case kDevDataDiffTooLarge:
    Console() << "error: frame difference too large: " << errStr << "\n";
    beginCalibrate();
    break;
  case kDevGapInSequence:
    if (mVerbose) {
      Console() << "note: gap in sequence " << errStr << "\n";
    }
    break;
  case kDevReset:
    if (mVerbose) {
      Console() << "isoch stalled, resetting " << errStr << "\n";
    }
    break;
  case kDevPayloadFailed:
    if (mVerbose) {
      Console() << "payload failed at sequence " << errStr << "\n";
    }
    break;
  case kDevNoInterface:
    Console() << "error: could not create device interface: " << errStr
                << "\n";
    break;
  case kDevInsufficientPower:
    Console() << "error: insufficient USB power: " << errStr << "\n";
    break;
  case kDevUnableToOpenDevice:
    Console() << "error: unable to open device: " << errStr << "\n";
    break;
  default:
    Console() << "unknown error: " << errStr << "\n";

    break;
  }
}

void Client::onClose() { enableOutput(false); }

void Client::processThread() {
  time_point<system_clock> previous, now;
  previous = now = system_clock::now();
  mPrevProcessTouchesTime = now; // TODO interval timer object
  static unsigned int debugFirstQueueFull = 0;

  while (!mTerminating) {
    now = system_clock::now();
    process(now);
    mProcessCounter++;

    size_t queueSize = mSensorFrameQueue->elementsAvailable();
    if (queueSize > mMaxRecentQueueSize) {
      mMaxRecentQueueSize = queueSize;
      debugFirstQueueFull = mProcessCounter;
    }

    if (mProcessCounter >= 1000) {
      if (mVerbose) {
        if (mMaxRecentQueueSize >= kSensorFrameQueueSize) {
          Console() << "warning: input queue full; " << debugFirstQueueFull << " of 1000\n";
        }
      }

      mProcessCounter = 0;
      mMaxRecentQueueSize = 0;
      debugFirstQueueFull = 0;
    }

    // sleep, less than one frame interval
    std::this_thread::sleep_for(std::chrono::microseconds(500));

    // do infrequent tasks every second
    int secondsInterval = duration_cast<seconds>(now - previous).count();
    if (secondsInterval >= 1) {
      previous = now;
      doInfrequentTasks();
    }
  }
}

void Client::process(time_point<system_clock> now) {
  static int tc = 0;
  tc++;

  TouchArray touches{};
  if (mTestTouchesOn || mTestTouchesWasOn) {
    touches = getTestTouchesFromTracker(now);
    mTestTouchesWasOn = mTestTouchesOn;
    outputTouches(touches, now);
  } else {
    if (mSensorFrameQueue->pop(mSensorFrame)) {
      /// mSurface = sensorFrameToSignal(mSensorFrame);

      // store surface for raw output
      // {
      //   std::lock_guard<std::mutex> lock(mRawSignalMutex);
      //   mRawSignal.copy(mSurface);
      // }

      if (mCalibrating) {
        mStats.accumulate(mSensorFrame);
        if (mStats.getCount() >= kSoundplaneCalibrateSize) {
          endCalibrate();
        }
      } else if (mSelectingCarriers) {
        mStats.accumulate(mSensorFrame);

        if (mStats.getCount() >= kSoundplaneCalibrateSize) {
          nextSelectCarriersStep();
        }
      } else if (mOutputEnabled) {
        if (mHasCalibration) {
          mCalibratedFrame =
              subtract(multiply(mSensorFrame, mCalibrateMeanInv), 1.0f);

          TouchArray touches = trackTouches(mCalibratedFrame);
          outputTouches(touches, now);
          // Console() << ".";
        }
      }
    }
  }
}

void Client::outputTouches(TouchArray touches,
                           time_point<system_clock> now) {
  // let Zones process touches. This is always done at the controller's frame
  // rate.
  sendTouchesToZones(touches);

  // determine if incoming frame could start or end a touch
  bool notesChangedThisFrame = findNoteChanges(touches, mTouchArray1);
  mTouchArray1 = touches;

  const int dataPeriodMicrosecs = 1000 * 1000 / mDataRate;
  int microsSinceSend =
      duration_cast<microseconds>(now - mPrevProcessTouchesTime).count();
  bool timeForNewFrame = (microsSinceSend >= dataPeriodMicrosecs);
  if (notesChangedThisFrame || timeForNewFrame || mRequireSendNextFrame) {
    mRequireSendNextFrame = false;
    mPrevProcessTouchesTime = now;
    sendFrameToOutputs(now);
  }
}

// send raw touches to zones in order to generate touch and controller states
// within the Zones.
//
void Client::sendTouchesToZones(TouchArray touches) {
  // const int maxTouches = getFloatProperty("max_touches");
  const float hysteresis = getFloatProperty("hysteresis");

  // clear incoming touches and push touch history in each zone
  for (auto &zone : mZones) {
    zone.newFrame();
  }

  // add any active touches to the Zones they are over
  // MLTEST for(int i=0; i<maxTouches; ++i)

  // iterate on all possible touches so touches will turn off when max_touches
  // is lowered
  for (int i = 0; i < kMaxTouches; ++i) {
    float x = touches[i].x;
    float y = touches[i].y;

    if (touchIsActive(touches[i])) {
      // Console() << i << ":" << age << "\n";
      // get fractional key grid position (Soundplane A)
      Vec2 keyXY(x, y);

      // get integer key
      int ix = (int)x;
      int iy = (int)y;

      // apply hysteresis to raw position to get current key
      // hysteresis: make it harder to move out of current key
      if (touches[i].state == kTouchStateOn) {
        mCurrentKeyX[i] = ix;
        mCurrentKeyY[i] = iy;
      } else {
        float hystWidth = hysteresis * 0.25f;
        MLRect currentKeyRect(mCurrentKeyX[i], mCurrentKeyY[i], 1, 1);
        currentKeyRect.expand(hystWidth);
        if (!currentKeyRect.contains(keyXY)) {
          mCurrentKeyX[i] = ix;
          mCurrentKeyY[i] = iy;
        }
      }

      // send index, xyz, dz to zone
      int zoneIdx = mZoneIndexMap(mCurrentKeyX[i], mCurrentKeyY[i]);
      if ((zoneIdx >= 0) && (zoneIdx < mZones.size())) {
        Touch t = touches[i];
        t.kx = mCurrentKeyX[i];
        t.ky = mCurrentKeyY[i];
        mZones[zoneIdx].addTouchToFrame(i, t);
      }
    }
  }

  for (auto &zone : mZones) {
    zone.storeAnyNewTouches();
  }

  std::bitset<kMaxTouches> freedTouches;

  // process note offs for each zone
  // this happens before processTouches() to allow touches to be freed for reuse
  // in this frame
  for (auto &zone : mZones) {
    zone.processTouchesNoteOffs(freedTouches);
  }

  // process touches for each zone
  for (auto &zone : mZones) {
    zone.processTouches(freedTouches);
  }
}

void Client::dumpOutputsByZone() {
  // count touches in zones
  int activeTouches = 0;
  for (auto &zone : mZones) {
    // touches
    for (int i = 0; i < kMaxTouches; ++i) {
      Touch t = zone.mOutputTouches[i];
      if (touchIsActive(t)) {
        activeTouches++;
      }
    }
  }

  if (activeTouches) {
    int zc = 0;

    // send messages to outputs about each zone
    for (auto &zone : mZones) {

      Console() << "[zone " << zc++ << ": ";

      // touches
      for (int i = 0; i < kMaxTouches; ++i) {
        Touch t = zone.mOutputTouches[i];
        if (touchIsActive(t)) {
          Console() << i << ":" << t.state << ":" << t.z << " ";
        }
      }

      Console() << "]";
    }
    Console() << "\n";
  }
}

void Client::sendFrameToOutputs(time_point<system_clock> now) {
  beginOutputFrame(now);

  // send messages to outputs about each zone
  for (auto &zone : mZones) {
    // touches
    for (int i = 0; i < kMaxTouches; ++i) {
      Touch t = zone.mOutputTouches[i];
      if (touchIsActive(t)) {
        sendTouchToOutputs(i, zone.mOffset, t);
      }
    }

    // controllers
    // if(zone.mOutputController.active)

    if (isControllerZoneType(zone.mType)) {
      sendControllerToOutputs(zone.mZoneID, zone.mOffset,
                              zone.mOutputController);
    }
  }

  // send optional calibrated matrix if enabled
  // if (mSendMatrixData) {
  //   ml::Matrix calibratedPressure = getCalibratedSignal();
  //   if (calibratedPressure.getHeight() == SensorGeometry::height) {
  //     mOutput.processMatrix(calibratedPressure);
  //   }
  // }

  endOutputFrame();
}

void Client::beginOutputFrame(time_point<system_clock> now) {
  if (mOutput.isActive()) {
    mOutput.beginOutputFrame(now);
  }
}

void Client::sendTouchToOutputs(int i, int offset, const Touch &t) {
  if (mOutput.isActive()) {
    mOutput.processTouch(i, offset, t);
  }
}

void Client::sendControllerToOutputs(int zoneID, int offset,
                                              const ZoneMessage &m) {
  if (mOutput.isActive()) {
    mOutput.processController(zoneID, offset, m);
  }

}

void Client::endOutputFrame() {
  if (mOutput.isActive()) {
    mOutput.endOutputFrame();
  }
}

void Client::setAllPropertiesToDefaults() {
  // parameter defaults and creation
  setProperty("max_touches", 4);
  setProperty("lopass_z", 100.);

  setProperty("z_thresh", 0.05);
  setProperty("z_scale", 1.);
  setProperty("z_curve", 0.5);
  setProperty("display_scale", 1.);

  setProperty("pairs", 0.);
  setProperty("quantize", 1.);
  setProperty("lock", 0.);
  setProperty("abs_rel", 0.);
  setProperty("snap", 250.);
  setProperty("vibrato", 0.5);

  setProperty("data_rate", 250.);

  setProperty("bend_range", 48);
  setProperty("transpose", 0);
  setProperty("bg_filter", 0.05);

  setProperty("hysteresis", 0.5);
  setProperty("lo_thresh", 0.1);

  // menu param defaults
  setProperty("viewmode", "calibrated");

  // preset menu defaults (TODO use first choices?)
  setProperty("zone_preset", "rows in fourths");
  setProperty("touch_preset", "touch default");

  setProperty("view_page", 0);

  for (int i = 0; i < 32; ++i) {
    setProperty(ml::textUtils::addFinalNumber("carrier_toggle", i), 1);
  }
}

void Client::initialize() {}

int Client::getDeviceState(void) {
  if (!mpDriver.get()) {
    return kNoDevice;
  }

  return mpDriver->getDeviceState();
}

// get a string that explains what Soundplane hardware and firmware and client
// versions are running.
const char *Client::getHardwareStr() {
  // TODO get model name along with dimensions from driver
  const char *ClientName = "Soundplane Model A";

  long v;
  unsigned char a, b, c;
  std::string serial_number;
  switch (getDeviceState()) {
  case kNoDevice:
  case kDeviceUnplugged:
    snprintf(mHardwareStr, kMiscStringSize, "no device");
    break;
  case kDeviceConnected:
  case kDeviceHasIsochSync:
    serial_number = mpDriver->getSerialNumberString();
    v = mpDriver->getFirmwareVersion();
    a = v >> 8 & 0x0F;
    b = v >> 4 & 0x0F, c = v & 0x0F;
    snprintf(mHardwareStr, kMiscStringSize, "%s #%s, firmware %d.%d.%d",
             ClientName, serial_number.c_str(), a, b, c);
    break;
  default:
    snprintf(mHardwareStr, kMiscStringSize, "?");
    break;
  }
  return mHardwareStr;
}

// get the string to report general connection status.
const char *Client::getStatusStr() {
  switch (getDeviceState()) {
  case kNoDevice:
    snprintf(mStatusStr, kMiscStringSize, "waiting for Soundplane...");
    break;

  case kDeviceConnected:
    snprintf(mStatusStr, kMiscStringSize, "waiting for isochronous data...");
    break;

  case kDeviceHasIsochSync:
    snprintf(mStatusStr, kMiscStringSize, "synchronized");
    break;

  case kDeviceUnplugged:
    snprintf(mStatusStr, kMiscStringSize, "unplugged");
    break;

  default:
    snprintf(mStatusStr, kMiscStringSize, "unknown status");
    break;
  }
  return mStatusStr;
}

// remove all zones from the zone list.
void Client::clearZones() {
  mZones.clear();
  mZoneIndexMap.fill(-1);
}

void Client::setZone(Zone zone) {
  std::vector<Zone> zones = { zone };
  setZones(zones);
}

void Client::setZones(std::vector<Zone> zones) {
  assert(zones.size() <= kSoundplaneAMaxZones);
  mOutputEnabled = false;
  clearZones();
  mZones = zones;
  buildZoneIndexMap();
  mOutputEnabled = true;
  sendParametersToZones();
}

void Client::buildZoneIndexMap()
{
  unsigned int zoneIdx = 0;
  for (auto &zone : mZones) {
    zone.setZoneID(zoneIdx);
    MLRect b(zone.getBounds());
    for (int j = b.y(); j < b.y() + b.height(); ++j) {
      for (int i = b.x(); i < b.x() + b.width(); ++i) {
        mZoneIndexMap(i, j) = zoneIdx;
      }
    }
    zoneIdx++;
  }
}

// copy relevant parameters from Model to zones
void Client::sendParametersToZones() {
  // TODO zones should have parameters (really attributes) too, so they can be
  // inspected.
  int zones = mZones.size();
  const float v = getFloatProperty("vibrato");
  const float h = getFloatProperty("hysteresis");
  bool q = getFloatProperty("quantize");
  bool nl = getFloatProperty("lock");
  int t = getFloatProperty("transpose");
  float sf = getFloatProperty("snap");

  for (int i = 0; i < zones; ++i) {
    mZones[i].mVibrato = v;
    mZones[i].mHysteresis = h;
    mZones[i].mQuantize = q;
    mZones[i].mNoteLock = nl;
    mZones[i].mTranspose = t;
    mZones[i].setSnapFreq(sf);
  }
}

// c over [0 - 1] fades response from sqrt(x) -> x -> x^2
//
float responseCurve(float x, float c) {
  float y;
  if (c < 0.5f) {
    y = lerp(x * x, x, c * 2.f);
  } else {
    y = lerp(x, sqrtf(x), c * 2.f - 1.f);
  }
  return y;
}

bool Client::findNoteChanges(TouchArray t0, TouchArray t1) {
  bool anyChanges = false;

  for (int i = 0; i < kMaxTouches; ++i) {
    if ((t0[i].state) != (t1[i].state)) {
      anyChanges = true;
      break;
    }
  }

  return anyChanges;
}

TouchArray Client::scaleTouchPressureData(TouchArray in) {
  TouchArray out = in;

  const float zscale = getFloatProperty("z_scale");
  const float zcurve = getFloatProperty("z_curve");
  const float dzScale = 0.125f;

  for (int i = 0; i < kMaxTouches; ++i) {
    float z = in[i].z;
    z *= zscale;
    z = ml::clamp(z, 0.f, 4.f);
    z = responseCurve(z, zcurve);
    out[i].z = z;

    // for note-ons, use same z scale controls as pressure
    float dz = in[i].dz * dzScale;
    dz *= zscale;
    dz = ml::clamp(dz, 0.f, 1.f);
    dz = responseCurve(dz, zcurve);
    out[i].dz = dz;
  }
  return out;
}

TouchArray Client::trackTouches(const SensorFrame &frame) {
  SensorFrame curvature = mTracker.preprocess(frame);
  TouchArray t = mTracker.process(curvature, mMaxTouches);
  // mSmoothedSignal = sensorFrameToSignal(curvature);
  t = scaleTouchPressureData(t);
  return t;
}

TouchArray Client::getTestTouchesFromTracker(time_point<system_clock> now) {
  TouchArray t = mTracker.getTestTouches(now, mMaxTouches);
  t = scaleTouchPressureData(t);
  return t;
}

void Client::doInfrequentTasks() {
  mOutput.doInfrequentTasks();

  if (getDeviceState() == kDeviceHasIsochSync) {
    if (mCarrierMaskDirty) {
      Console() << "mCarrierMaskDirty: calling enableCarriers()\n";
      enableCarriers(mCarriersMask);
    } else if (mNeedsCarriersSet) {
      Console() << "mNeedsCarriersSet: calling setCarriers() and triggering calibration\n";
      mNeedsCarriersSet = false;
      if (mDoOverrideCarriers) {
        setCarriers(mOverrideCarriers);
      } else {
        setCarriers(mCarriers);
      }

      mNeedsCalibrate = true;
    } else if (mNeedsCalibrate && (!mSelectingCarriers)) {
      Console() << "mNeedsCalibrate: calling beginCalibrate()\n";
      mNeedsCalibrate = false;
      beginCalibrate();
    }
  }

  if (mCalibrating) {
    Console() << "Client::doInfrequentTasks(): calib progress = " << getCalibrateProgress() << "\n";
  }
}

void Client::setDefaultCarriers() {
  ml::Matrix cSig(kSoundplaneNumCarriers);
  for (int car = 0; car < kSoundplaneNumCarriers; ++car) {
    cSig[car] = kModelDefaultCarriers[car];
  }
  setProperty("carriers", cSig);
}

void Client::setCarriers(const SoundplaneDriver::Carriers &c) {
  enableOutput(false);
  mpDriver->setCarriers(c);
}

int Client::enableCarriers(unsigned long mask) {
  mpDriver->enableCarriers(~mask);
  if (mask != mCarriersMask) {
    mCarriersMask = mask;
  }
  mCarrierMaskDirty = false;
  return 0;
}

void Client::dumpCarriers(const SoundplaneDriver::Carriers &carriers) {
  Console() << "\n------------------\n";
  Console() << "carriers: \n";
  for (int i = 0; i < kSoundplaneNumCarriers; ++i) {
    int c = carriers[i];
    Console() << i << ": " << c << " [" << carrierToFrequency(c) << "Hz] \n";
  }
}

void Client::enableOutput(bool b) { mOutputEnabled = b; }

void Client::clear() { mTracker.clear(); }

// --------------------------------------------------------------------------------
#pragma mark surface calibration

// using the current carriers, calibrate the surface by collecting data and
// calculating the mean rest value for each taxel.
//
void Client::beginCalibrate() {
  if (getDeviceState() == kDeviceHasIsochSync) {
    Console() << "Client::beginCalibrate: getDeviceState() == kDeviceHasIsochSync; starting calibration\n";
    mStats.clear();
    mCalibrating = true;
  }
}

// called by process routine when enough samples have been collected.
//
void Client::endCalibrate() {
  Console() << "Client::endCalibrate() called\n";
  SensorFrame mean = clamp(mStats.mean(), 0.0001f, 1.f);
  mCalibrateMeanInv = divide(fill(1.f), mean);
  mCalibrating = false;
  mHasCalibration = true;
  enableOutput(true);
}

float Client::getCalibrateProgress() {
  return mStats.getCount() / (float)kSoundplaneCalibrateSize;
}

// --------------------------------------------------------------------------------
#pragma mark carrier selection

void Client::beginSelectCarriers() {
  // each possible group of carrier frequencies is tested to see which
  // has the lowest overall noise.
  // each step collects kSoundplaneCalibrateSize frames of data.
  //
  if (getDeviceState() == kDeviceHasIsochSync) {
    mSelectCarriersStep = 0;
    mStats.clear();
    mSelectingCarriers = true;
    mTracker.clear();
    mMaxNoiseByCarrierSet.resize(kStandardCarrierSets);
    mMaxNoiseByCarrierSet.clear();
    mMaxNoiseFreqByCarrierSet.resize(kStandardCarrierSets);
    mMaxNoiseFreqByCarrierSet.clear();

    // setup first set of carrier frequencies
    Console() << "testing carriers set " << mSelectCarriersStep << "...\n";
    makeStandardCarrierSet(mCarriers, mSelectCarriersStep);
    setCarriers(mCarriers);
  }
}

float Client::getSelectCarriersProgress() {
  float p;
  if (mSelectingCarriers) {
    p = (float)mSelectCarriersStep / (float)kStandardCarrierSets;
  } else {
    p = 0.f;
  }
  return p;
}

void Client::nextSelectCarriersStep() {
  // analyze calibration data just collected.
  SensorFrame mean = clamp(mStats.mean(), 0.0001f, 1.f);
  SensorFrame stdDev = mStats.standardDeviation();
  SensorFrame variation = divide(stdDev, mean);

  // find maximum noise in any column for this set.  This is the "badness" value
  // we use to compare carrier sets.
  float maxVar = 0;
  float maxVarFreq = 0;
  float variationSum;
  int startSkip = 2;

  for (int col = startSkip; col < kSoundplaneNumCarriers; ++col) {
    variationSum = 0;
    int carrier = mCarriers[col];
    float cFreq = carrierToFrequency(carrier);

    variationSum = getColumnSum(variation, col);
    if (variationSum > maxVar) {
      maxVar = variationSum;
      maxVarFreq = cFreq;
    }
  }

  mMaxNoiseByCarrierSet[mSelectCarriersStep] = maxVar;
  mMaxNoiseFreqByCarrierSet[mSelectCarriersStep] = maxVarFreq;

  Console() << "max noise for set " << mSelectCarriersStep << ": " << maxVar
              << "(" << maxVarFreq << " Hz) \n";

  // set up next step.
  mSelectCarriersStep++;
  if (mSelectCarriersStep < kStandardCarrierSets) {
    // set next carrier frequencies to calibrate.
    Console() << "testing carriers set " << mSelectCarriersStep << "...\n";
    makeStandardCarrierSet(mCarriers, mSelectCarriersStep);
    setCarriers(mCarriers);
  } else {
    endSelectCarriers();
  }

  // clear data
  mStats.clear();
}

void Client::endSelectCarriers() {
  // get minimum of collected noise sums
  float minNoise = 99999.f;
  int minIdx = -1;
  Console() << "------------------------------------------------\n";
  Console() << "carrier select noise results:\n";
  for (int i = 0; i < kStandardCarrierSets; ++i) {
    float n = mMaxNoiseByCarrierSet[i];
    float h = mMaxNoiseFreqByCarrierSet[i];
    Console() << "set " << i << ": max noise " << n << "(" << h << " Hz)\n";
    if (n < minNoise) {
      minNoise = n;
      minIdx = i;
    }
  }

  // set that carrier group
  Console() << "setting carriers set " << minIdx << "...\n";
  makeStandardCarrierSet(mCarriers, minIdx);
  setCarriers(mCarriers);

  // set chosen carriers as model parameter so they will be saved
  // this will trigger a recalibrate
  ml::Matrix cSig(kSoundplaneNumCarriers);
  for (int car = 0; car < kSoundplaneNumCarriers; ++car) {
    cSig[car] = mCarriers[car];
  }
  setProperty("carriers", cSig);
  Console() << "carrier select done.\n";

  mSelectingCarriers = false;
  mNeedsCalibrate = true;
}
