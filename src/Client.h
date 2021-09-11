//
// Client.h
//
// Defines a headless Soundplane client.
//
// This code is derived from the official Soundplane client software available
// at https://github.com/madronalabs/soundplane which bares this copyright:
//
// Part of the Soundplane client software by Madrona Labs.
// Copyright (c) 2013 Madrona Labs LLC. http://www.madronalabs.com
// Distributed under the MIT license: http://madrona-labs.mit-license.org/
//

#pragma once

#include <list>
#include <map>
#include <thread>
#include <memory>

// #include "soundplanelib/SoundplaneModelA.h"
#include "SoundplaneModelA.h"
#include "SoundplaneDriver.h"
#include "SensorFrame.h"

#include "MLMatrix.h"
#include "MLQueue.h"

// soundplane (client)
#include "TouchTracker.h"
// #include "Zone.h"

ml::Matrix sensorFrameToSignal(const SensorFrame &f);

typedef enum
{
	xColumn = 0,
	yColumn = 1,
	zColumn = 2,
	dzColumn = 3,
	ageColumn = 4
} TouchSignalColumns;

const int kSensorFrameQueueSize = 16;

class Client : public SoundplaneDriverListener {
public:

	Client();
	~Client();

	// SoundplaneDriverListener
	void onStartup() override;
	void onFrame(const SensorFrame& frame) override;
	void onError(int error, const char* errStr) override;
	void onClose() override;

	// MLFileCollection& getZonePresetsCollection() { return *mZonePresets; }

	// float getSampleHistory(int x, int y);

	// void getHistoryStats(float& mean, float& stdDev);
	// int getWidth() { return mSurface.getWidth(); }
	// int getHeight() { return mSurface.getHeight(); }

	// void setDefaultCarriers();
	// void setCarriers(const SoundplaneDriver::Carriers& c);
	// int enableCarriers(unsigned long mask);
	// int getNumCarriers() { return kSoundplaneNumCarriers; }
	// void dumpCarriers(const SoundplaneDriver::Carriers& carriers);

	// void enableOutput(bool b);

	// int getStateIndex();
	// const char* getHardwareStr();
	// const char* getStatusStr();
	// const char* getClientStr();

	// int getSerialNumber() const {return mSerialNumber;}

	// void clear();

	// void setRaw(bool b);
	// bool getRaw(){ return mRaw; }

	// void beginCalibrate();
	// bool isCalibrating() { return mCalibrating; }
	// float getCalibrateProgress();
	// void endCalibrate();

	// void beginSelectCarriers();
	// bool isSelectingCarriers() { return mSelectingCarriers; }
	// float getSelectCarriersProgress();
	// void nextSelectCarriersStep();
	// void endSelectCarriers();

	// void setFilter(bool b);

	// void getMinMaxHistory(int n);

	// const ml::Matrix& getTouchFrame() { return mTouchFrame; }
	// const ml::Matrix& getTouchHistory() { return mTouchHistory; }
	// const ml::Matrix getRawSignal() { std::lock_guard<std::mutex> lock(mRawSignalMutex); return mRawSignal; }
	// const ml::Matrix getCalibratedSignal() { std::lock_guard<std::mutex> lock(mCalibratedSignalMutex); return sensorFrameToSignal(mCalibratedFrame); }

	// const ml::Matrix getSmoothedSignal() { std::lock_guard<std::mutex> lock(mSmoothedSignalMutex); return mSmoothedSignal; }

	// const TouchArray& getTouchArray() { return mTouchArray1; }

	// bool isWithinTrackerCalibrateArea(int i, int j);
	// const int getHistoryCtr() { return mHistoryCtr; }

	// const std::vector< Zone >::const_iterator getZonesBegin(){ return mZones.begin(); }
	// const std::vector< Zone >::const_iterator getZonesEnd(){ return mZones.end(); }

	// void setStateFromJSON(cJSON* pNode, int depth);
	// bool loadZonePresetByName(const std::string& name);

	// int getDeviceState(void);
	// int getClientState(void);

private:
  	// TouchArray mTouchArray1{};
	// TouchArray mZoneOutputTouches{};

	std::unique_ptr<SoundplaneDriver> mpDriver;
	std::unique_ptr<ml::Queue<SensorFrame>> mSensorFrameQueue;
};