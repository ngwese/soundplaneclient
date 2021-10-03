#pragma once

#include "MLParameter.h"
#include "MLSymbol.h"
#include "MLVectorDeprecated.h"

#include <vector>

namespace soundplane {

class ZoneSpec {
public:
  ZoneSpec() = default;
  ZoneSpec(const TextFragment &name, const Symbol &zoneType, const MLRect &bounds);
  ~ZoneSpec() = default;

  static ZoneSpec buildNoteRow(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t note);
  static ZoneSpec buildToggle(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrl1);
  static ZoneSpec buildXY(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrlx, uint8_t ctrly);
  static ZoneSpec buildX(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrlx);
  static ZoneSpec buildY(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrly);

  static ZoneSpec presetChromatic();
  static std::vector<ZoneSpec> presetRowsInFourths();
  static std::vector<ZoneSpec> presetRowsInOctaves();

  void setBounds(MLRect r);

  const ml::TextFragment &getName() const { return mName; }
  const MLRect &getBounds() const { return mBounds; }
  const Symbol &getType() const { return mType; }
  int getStartNote() const { return mStartNote; }
  int getController1() const { return mControllerNum1; }
  int getController2() const { return mControllerNum2; }
  int getController3() const { return mControllerNum3; }

protected:
  ml::TextFragment mName{};
  Symbol mType{};

  MLRect mBounds;

  int mStartNote{60};

  int mControllerNum1{0};
  int mControllerNum2{0};
  int mControllerNum3{0};

  int mOffset{0};
};

} // namespace soundplane