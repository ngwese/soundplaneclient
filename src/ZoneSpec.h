#pragma once

#include "MLParameter.h"
#include "MLSymbol.h"
#include "MLVectorDeprecated.h"

#include <vector>

namespace soundplane {

class ZoneSpec {
public:
  ZoneSpec();
  ~ZoneSpec() {}

  // ZoneSpec &operator=(const ZoneSpec &b) noexcept;

  static ZoneSpec buildNoteRow(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t note);
  static ZoneSpec presetChromatic();
  static std::vector<ZoneSpec> presetRowsInFourths();
  static std::vector<ZoneSpec> presetRowsInOctaves();

  void setBounds(MLRect r);

  const ml::TextFragment getName() const { return mName; }
  MLRect getBounds() const { return mBounds; }
  Symbol getType() const { return mType; }
  int getStartNote() const { return mStartNote; }

protected:
  ml::TextFragment mName{};
  Symbol mType{};

  MLRect mBounds;

  int mStartNote{60};

  // int mControllerNum1{0};
  // int mControllerNum2{0};
  // int mControllerNum3{0};

  int mOffset{0};
};

} // namespace soundplane