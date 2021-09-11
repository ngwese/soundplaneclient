
// MadronaLib: a C++ framework for DSP applications.
// Copyright (c) 2013 Madrona Labs LLC. http://www.madronalabs.com
// Distributed under the MIT license: http://madrona-labs.mit-license.org/

#ifndef ML_PARAMETER_H
#define ML_PARAMETER_H

#include <vector>
#include <list>

#include "MLDSPDeprecated.h"
#include "MLSymbol.h"
#include "MLPath.h"
#include "MLMatrix.h"
#include "MLValue.h"

// MLPublishedParam: a parameter of one of the Procs in a DSP graph that is settable through the plugin wrapper.
// There are two concepts wrapped up in this that should be factored out.
// One is a way to view and manipulate a Property. There is a list of these ways. 
// The other is publishing properties of multiple procs in a graph as one settable entity. 
// the Property might be a string, so there is no way to route it to multiple procs within the graph.
// if there were, we could remove the "alsosets" and use proc connections. 
// "alsosets" should check to see if Properties are of compatible types... sort of an ugliness point. 

typedef enum
{
	kJucePluginParam_Generic,
	kJucePluginParam_Index,
	kJucePluginParam_Seconds,
	kJucePluginParam_Hertz,
	kJucePluginParam_SemiTones,
	kJucePluginParam_Decibels,
	kJucePluginParam_Pan,
	kJucePluginParam_BPM,
}	JucePluginParamUnit;

typedef enum
{
	kJucePluginParam_Linear,
	kJucePluginParam_Exp,
	kJucePluginParam_ExpBipolar
}	JucePluginParamWarpMode;

// ----------------------------------------------------------------
// a published param means: named parameter mParam of mProc is called mPublishedAlias.
//
class MLPublishedParam
{
friend class MLProcContainer;
private:
	class ParamAddress
	{
	public:
		ParamAddress(const ml::Path & alias, const ml::Symbol name) : procAddress(alias), paramName(name) {}
		~ParamAddress() {}
		
		// procAddress is where to send the param.  can resolve to a single MLProc,
		// or a list of processors in the case of multiples.  The address is always relative to
		// the container that publishes the parameters.
		ml::Path procAddress;
		ml::Symbol paramName;
	};

public:	
	MLPublishedParam(const ml::Path & address, const ml::Symbol name, const ml::Symbol alias, const ml::Symbol type, int idx);
	~MLPublishedParam();
	
	void setRange(float low, float high, float interval, bool log, float zt, float offset);
	void addAddress(const ml::Path & address, const ml::Symbol name);

	ml::Symbol getType() { return mType; }
	float getValue();
	
	const ml::Value& getValueProperty();
	void setValueProperty(const ml::Value& val);
	
	float getValueAsLinearProportion() const;
	float setValueAsLinearProportion (float p);

	unsigned getIndex(void) { return mIndex; }
	float getRangeLo(void) const { return mRangeLo; }
	float getRangeHi(void) const { return mRangeHi; }
	float getInterval(void) { return mInterval; }
	float getZeroThresh(void) { return mZeroThreshold; }
	JucePluginParamWarpMode getWarpMode(void) { return mWarpMode; }
	float getDefault(void);
	
	JucePluginParamUnit getUnit(void) { return mUnit; }

	int getGroupIndex(void) { return mGroupIndex; }
	void setGroupIndex(int g) { mGroupIndex = g; }

	bool getAutomatable(void);
	void setAutomatable(bool q);
	
	ml::Symbol getAlias(void) { return mPublishedAlias; }
				
	typedef std::list<ParamAddress>::const_iterator AddressIterator;
	AddressIterator beginAddress() { return mAddresses.begin(); }
	AddressIterator endAddress() { return mAddresses.end(); }	

protected:
	void setDefault(float val);
	
private:
	std::list<ParamAddress> mAddresses;
	ml::Value mParamValue;
	float mTempValue;
	
	ml::Symbol mPublishedAlias;
	ml::Symbol mType;
	unsigned mIndex;
	float mRangeLo;
	float mRangeHi;
	float mInterval;
	float mZeroThreshold;
	float mOffset;
	float mDefault;
	bool mFlip;

	bool mAutomatable;
	JucePluginParamUnit mUnit;
	JucePluginParamWarpMode mWarpMode;
	int mGroupIndex;	// -1 for none
};

typedef std::shared_ptr<MLPublishedParam> MLPublishedParamPtr;

// ----------------------------------------------------------------
#pragma mark named parameter groups

class MLParamGroupMap
{
public:
	MLParamGroupMap();
	~MLParamGroupMap();
	void clear();
	
	// set the current group index to the index matching groupSym. 
	// if an entry for groupSym does not exist, it is made.
	void setGroup(const ml::Symbol groupSym);	
	
	// mark the ParamPtr as belonging to the current group.
	void addParamToCurrentGroup(MLPublishedParamPtr p);	
	
	// get the group name of the indexed parameter.
	const std::string& getGroupName(unsigned index);
	
	std::vector<std::string> mGroupVec;
	int mCurrentGroup;
};

#endif // ML_PARAMETER_H
