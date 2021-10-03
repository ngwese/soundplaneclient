#include "ZoneSpec.h"

#include "SoundplaneModelA.h"

using namespace soundplane;

static const Symbol noteRowType("note_row");
static const Symbol toggleType("toggle");
static const Symbol xyType("xy");
static const Symbol xType("x");
static const Symbol yType("y");

ZoneSpec::ZoneSpec(const TextFragment &name, const Symbol &zoneType, const MLRect &bounds) : mName(name), mType(zoneType), mBounds(bounds) {
}

ZoneSpec ZoneSpec::buildNoteRow(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t note) {
  ZoneSpec z(TextFragment(name), noteRowType, MLRect(x1, y1, x2, y2));
  z.mStartNote = note;
  return std::move(z);
}

ZoneSpec ZoneSpec::buildToggle(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrl1) {
  ZoneSpec z(TextFragment(name), toggleType, MLRect(x1, y1, x2, y2));
  z.mControllerNum1 = ctrl1;
  return std::move(z);
}

ZoneSpec ZoneSpec::buildXY(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrlx, uint8_t ctrly) {
  ZoneSpec z(TextFragment(name), xyType, MLRect(x1, y1, x2, y2));
  z.mControllerNum1 = ctrlx;
  z.mControllerNum2 = ctrly;
  return std::move(z);
}

ZoneSpec ZoneSpec::buildX(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrlx) {
  ZoneSpec z(TextFragment(name), xType, MLRect(x1, y1, x2, y2));
  z.mControllerNum1 = ctrlx;
  return std::move(z);
};

ZoneSpec ZoneSpec::buildY(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t ctrly) {
  ZoneSpec z(TextFragment(name), yType, MLRect(x1, y1, x2, y2));
  z.mControllerNum1 = ctrly;
  return std::move(z);
}

ZoneSpec ZoneSpec::presetChromatic() {
  return buildNoteRow("chromatic", 0, 0, kSoundplaneAKeyWidth, kSoundplaneAKeyHeight, 57);
}

std::vector<ZoneSpec> ZoneSpec::presetRowsInFourths() {
  std::vector<ZoneSpec> preset = {
    buildNoteRow("B2", 0, 0, 30, 1, 47),
    buildNoteRow("E2", 0, 1, 30, 1, 52),
    buildNoteRow("A2", 0, 2, 30, 1, 57),
    buildNoteRow("D3", 0, 3, 30, 1, 62),
    buildNoteRow("D3", 0, 3, 30, 1, 62),
  };
  return preset;
}

std::vector<ZoneSpec> ZoneSpec::presetRowsInOctaves() {
  std::vector<ZoneSpec> preset = {
    buildNoteRow("A0", 0, 0, 30, 1, 33),
    buildNoteRow("A1", 0, 1, 30, 1, 45),
    buildNoteRow("A2", 0, 2, 30, 1, 57),
    buildNoteRow("A3", 0, 3, 30, 1, 69),
    buildNoteRow("A4", 0, 3, 30, 1, 81),
  };
  return preset;
}

void ZoneSpec::setBounds(MLRect b) {
  mBounds = b;
}

