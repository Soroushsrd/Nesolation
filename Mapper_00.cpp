
#include "Mapper_00.h"
#include <cstdint>

Mapper_00::Mapper_00(uint8_t prgBanks, uint8_t chrBanks): Mapper(prgBanks, chrBanks) {}

Mapper_00::~Mapper_00() {}

bool Mapper_00::cpuMapWrite(uint16_t address, uint32_t &mapped_addr) {
  if (address >= 0x8000 && address <= 0xFFFF) {
    mapped_addr = address & (nPRGBanks > 1 ? 0x7FFF : 0x3FFF);
    return true;
  }
  return false;
}
bool Mapper_00::cpuMapRead(uint16_t address, uint32_t &mapped_addr) {
  // if PRGROM is 16KB
  //     CPU Address Bus          PRG ROM
  //     0x8000 -> 0xBFFF: Map    0x0000 -> 0x3FFF
  //     0xC000 -> 0xFFFF: Mirror 0x0000 -> 0x3FFF
  // if PRGROM is 32KB
  //     CPU Address Bus          PRG ROM
  //     0x8000 -> 0xFFFF: Map    0x0000 -> 0x7FFF
  if (address >= 0x8000 && address <= 0xFFFF) {
    mapped_addr = address & (nPRGBanks > 1 ? 0x7FFF : 0x3FFF);
    return true;
  }

  return false;
}

// to communicate with the ppu
bool Mapper_00::ppuMapWrite(uint16_t addr, uint32_t &mapped_addr) {
  if (addr >= 0x0000 && addr <= 0x1FFF) {
    if (nCHRBanks == 0) {
      // Treat as RAM
      mapped_addr = addr;
      return true;
    }
  }

  return false;
}

bool Mapper_00::ppuMapRead(uint16_t address, uint32_t &mapped_addr) {
  // There is no mapping required for PPU
  // PPU Address Bus          CHR ROM
  // 0x0000 -> 0x1FFF: Map    0x0000 -> 0x1FFF
  if (address >= 0x0000 && address <= 0x1FFF) {
    mapped_addr = address;
    return true;
  }

  return false;
}
