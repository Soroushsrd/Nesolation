
#include "Ppu.h"
#include <cstdint>
#include "Cartridge.h"

uint8_t Ppu::cpuRead(uint16_t address, bool readonly) {
  uint8_t data = 0x00;

  switch (address) {
    case 0x0000: // control
      break;
    case 0x0001: // mask
      break;
    case 0x0002: // status
      break;
    case 0x0003: // OAM addr
      break;
    case 0x0004: // OAM data
      break;
    case 0x0005: // scroll
      break;
    case 0x0006: // ppu address
      break;
    case 0x0007: // ppu data
      break;
  }
  return data;
}

void Ppu::cpuWrite(uint16_t address, uint8_t data) {
  switch (address) {
    case 0x0000: // control
      break;
    case 0x0001: // mask
      break;
    case 0x0002: // status
      break;
    case 0x0003: // OAM addr
      break;
    case 0x0004: // OAM data
      break;
    case 0x0005: // scroll
      break;
    case 0x0006: // ppu address
      break;
    case 0x0007: // ppu data
      break;
  }
}

uint8_t Ppu::ppuRead(uint16_t address, bool readonly) {
  uint8_t data = 0x00;
  address &= 0x3FFF;

  if (cart->ppuread(address, readonly)) {
    // emptry for now
  }
  return data;
}

void Ppu::ppuWrite(uint16_t address, uint8_t data) {
  // masking to make sure ppu doesnt write to an address beyond its define range!
  address &= 0x3FFF;
  if (cart->ppuwrite(address, data)) {
    // empty for now
  }
}
