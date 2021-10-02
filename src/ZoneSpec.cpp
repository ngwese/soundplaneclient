#include "ZoneSpec.h"

#include "SoundplaneModelA.h"

using namespace soundplane;

ZoneSpec::ZoneSpec() {
}

// ZoneSpec &ZoneSpec::operator=(const ZoneSpec &b) noexcept {
//   if (this != &b) {


//   }
//   return *this;
// }

ZoneSpec ZoneSpec::buildNoteRow(const char *name, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t note) {
  ZoneSpec z;
  z.mName = TextFragment(name);
  z.mType = Symbol("note_row");
  z.setBounds(MLRect(x1, y1, x2, y2));
  z.mStartNote = note;
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

