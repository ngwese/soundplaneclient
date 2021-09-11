
// MadronaLib: a C++ framework for DSP applications.
// Copyright (c) 2013 Madrona Labs LLC. http://www.madronalabs.com
// Distributed under the MIT license: http://madrona-labs.mit-license.org/

#include "MLParameter.h"

// ----------------------------------------------------------------
#pragma mark published parameters

MLPublishedParam::MLPublishedParam(const ml::Path & procPath, const ml::Symbol name, const ml::Symbol alias, const ml::Symbol type, int idx) :
	mPublishedAlias(alias),
	mIndex(idx),
	mFlip(false),
	mAutomatable(true)
{
	setRange(0.f, 1.f, 0.01f, false, 0.f, 0.f);
	mUnit = kJucePluginParam_Generic;
	mWarpMode = kJucePluginParam_Linear;
	mParamValue = 0.f;
	mDefault = 0.f;
	mZeroThreshold = 0.f - (float)(2 << 16);
	mOffset = 0;
	mGroupIndex = -1;
	addAddress(procPath, name);
	
	// default type is float
	if(type == ml::Symbol())
	{
		mType = ml::Symbol("float");
	}
	else
	{
		mType = type;
	}
}

MLPublishedParam::~MLPublishedParam() 
{

}

void MLPublishedParam::setRange(float low, float high, float v, bool log, float zt, float offset)
{ 
	mRangeLo = low; 
	mRangeHi = high; 
	mInterval = v; 
	mZeroThreshold = zt;
	mOffset = offset;
	mFlip = (low > high);
	
	// TODO take warp mode as arg
	if (log)
	{
		mWarpMode = kJucePluginParam_Exp;
	}
	else
	{
		mWarpMode = kJucePluginParam_Linear;
	}
	
	// setup threshold for nonlinear modes
	if (mWarpMode != kJucePluginParam_Linear)
	{
		if (mRangeLo == 0.)
		{
			// set threshold under which params get set to 0.
			mRangeLo = mInterval;
			if (zt == 0.)
			{
				mZeroThreshold = mInterval;
			}
		}
	}
}

void MLPublishedParam::addAddress(const ml::Path & procPath, const ml::Symbol name)
{
	mAddresses.push_back(ParamAddress(procPath, name));
}

float MLPublishedParam::getDefault(void)
{
	return mDefault;
}
	
void MLPublishedParam::setDefault(float val)
{
	mDefault = val;
}

float MLPublishedParam::getValue(void)
{
	return mParamValue.getFloatValue();
}

const ml::Value& MLPublishedParam::getValueProperty()
{
	return mParamValue;
}

// set the value of the parameter to a float, string or signal property. Once
// allocated initially the property cannot be resized.
void MLPublishedParam::setValueProperty(const ml::Value& paramProp)
{
	ml::Value::Type type = paramProp.getType();
	switch(type)
	{
		case ml::Value::kFloatValue:
		{
			const float val = paramProp.getFloatValue();
			float clampedVal = ml::clamp(val, mRangeLo, mRangeHi);
			if (fabs(clampedVal) <= mZeroThreshold)
			{
				clampedVal = 0.f;
			}
			mParamValue.setValue(clampedVal);
			break;
		}
		case ml::Value::kTextValue:
		{
			mParamValue.setValue(paramProp.getTextValue());
			break;
		}
		case ml::Value::kMatrixValue:
		{
			mParamValue.setValue(paramProp.getMatrixValue());
			break;
		}
		default:
			break;
	}
}

float MLPublishedParam::getValueAsLinearProportion() const
{
	float lo = getRangeLo();
	float hi = getRangeHi();
	float p;
	float val = mParamValue.getFloatValue();
	
	switch (mWarpMode)
	{
		case kJucePluginParam_Linear:
		default:
			p = (val - lo) / (hi - lo);
			p += mOffset;
			break;
		case kJucePluginParam_Exp:
			val = ml::clamp(val, lo, hi);
			val = ml::max(mZeroThreshold, val);
			p = logf(val/lo) / logf(hi/lo);
			p += mOffset;
			break;
		case kJucePluginParam_ExpBipolar:
			bool positiveHalf = val > 0.;
			if (positiveHalf)
			{
				val = ml::clamp(val, lo, hi);
				val = ml::max(mZeroThreshold, val);
				p = logf(val/lo) / logf(hi/lo);
				p = p*0.5f + 0.5f;
			}
			else
			{
				val = -ml::clamp(val, -hi, -lo);
				val = ml::max(mZeroThreshold, val);
				p = logf(val/lo) / logf(hi/lo);
				p = -p*0.5f + 0.5f;
			}
			break;
	}
	if(mFlip)
	{
		p = 1.0f - p;
	}
	
	////debug() << "val = " << val << " -> prop = " << p << "\n";
	return p;
}

float MLPublishedParam::setValueAsLinearProportion (float pIn)
{
	float lo = getRangeLo();
	float hi = getRangeHi();
	float val = 0.f;
	float pBipolar, valExp;

	float p = mFlip ? (1.0f - pIn) : pIn;
	p -= mOffset;
	switch (mWarpMode)
	{
		case kJucePluginParam_Linear:
			val = lo + p*(hi - lo);
			break;
		case kJucePluginParam_Exp:
			valExp = p*(logf(hi)/logf(lo) - 1) + 1;
			val = powf(lo, valExp);
			if (val < mZeroThreshold)
			{
				val = 0.f;
			}
			break;
		case kJucePluginParam_ExpBipolar:
			bool positiveHalf = p > 0.5f;
			pBipolar = positiveHalf ? (p - 0.5f)*2.f : (0.5f - p)*2.f;
			valExp = pBipolar*(logf(hi)/logf(lo) - 1) + 1;
			val = positiveHalf ? powf(lo, valExp) : -powf(lo, valExp);
			if (fabs(val) < mZeroThreshold)
			{
				val = 0.f;
			}
			break;
	}
	
	////debug() << " prop = " << p <<  " -> val = " << val << "\n";
	mParamValue = val;
	return val;
}

bool MLPublishedParam::getAutomatable(void)
{
	return mAutomatable;
}

void MLPublishedParam::setAutomatable(bool a)
{
	mAutomatable = a;
}

// ----------------------------------------------------------------
#pragma mark named parameter groups

MLParamGroupMap::MLParamGroupMap()
{
	mCurrentGroup = -1;
	clear();
}

MLParamGroupMap::~MLParamGroupMap()
{
}

void MLParamGroupMap::clear()
{
	mCurrentGroup = -1;
	mGroupVec.clear();
	mGroupVec.push_back(std::string("null"));
}

void MLParamGroupMap::setGroup(const ml::Symbol groupSym)
{
	unsigned i = 0;
	bool found = false;
	std::string groupStr = std::string(groupSym.getTextFragment().getText());
	for (i=0; i<mGroupVec.size(); i++)
	{
		if (!groupStr.compare(mGroupVec[i]))
		{
			found = true;
			break;
		}
	}
	
	if(!found) 
	{
		// create new group
		mGroupVec.push_back(groupStr);
	}

	mCurrentGroup = i;
}

void MLParamGroupMap::addParamToCurrentGroup(MLPublishedParamPtr p)
{
	p->setGroupIndex(mCurrentGroup);
}

const std::string& MLParamGroupMap::getGroupName(unsigned index)
{
	return mGroupVec[index];
}

