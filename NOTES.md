cmake -DLINUX_JACK=ON -DCMAKE_CXX_FLAGS="-march=armv7-a -mfpu=neon-vfpv4" ..

SoundplaneModel TODO
====================

- factor out zone file i/o, mZonePresets... or dump zones entirely and handle in lua?
    SoundplaneModel::loadZonesFromString (passing in a vector of zones should be fine)
- factor out touch file i/o, mTouchPresets
- factor out (remove?) Kyma support, instead handle that outside the library
- factor out MIDI support, OSC Support, handle outside the library
    - beginOutputFrame()
    - sendTouchToOutputs()
    - sendControllerToOutputs()
    - endOutputFrame()
    - All the above explictly drive mMIDIOutput and mOSCOutput

The general structure of the SoundplaneModel class appears to be:
- onStartup, onFrame, onError, onClose callbacks run in the SoundplaneDriver thread
- the callbacks push raw frames into a queue

- the SoundplaneMode::processThread pulls frames from the queue
- and runs them through one of (a) calibrarion, (b) carrier selection, or (c) endOutputFrame

** Incoming OSC traffic is for Kyma support (SoundplaneModel::ProcessMessage)

- SoundplaneModel::doInfrequentTasks() appears to be where calibration vs carrier selection is done

** Factor out calibration/carrier/output into implementations of a
"FrameProcessor" class? ===> maybe not since carrier selection uses the tracker?

- Consider removing SoundplaneModel::saveTouchHistory? particularly if its only
purpose is for display.


