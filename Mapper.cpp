
#include "Mapper.h"
#include <cstdint>

Mapper::Mapper(uint8_t prgBanks, uint8_t chrBanks) {
  nCHRBanks = chrBanks;
  nPRGBanks = prgBanks;
}
Mapper::~Mapper() {}
