
#ifndef NES_PPU_H
#define NES_PPU_H

#include <cstdint>
#include <memory>
#include "Cartridge.h"
class Ppu {
public:
  Ppu();
  ~Ppu();

  // communicate with the main bus
  uint8_t cpuRead(uint16_t address, bool readonly = false);
  void    cpuWrite(uint16_t address, uint8_t data);

  // communicate with the ppu bus
  uint8_t ppuRead(uint16_t address, bool readonly = false);
  void    ppuWrite(uint16_t address, uint8_t data);
  void    clock();
  void    ConnectCartridge(const std::shared_ptr<Cartridge> &cart);

  // ppu has its own bus which contains the Pattern memory, NameTable
private:
  std::shared_ptr<Cartridge> cart;
  // 2kb, Name Table, stores the ids of which pattern to show in the background
  // 0x2000 to 0x2FFF
  uint8_t tblName[2][1024];
  // Pattern memory wh9ch has 8kbs
  // and stores what the graphics look like (sprites)
  // 0x0000 to 0x1FFFF
  uint8_t tblePattern[2][4098];
  // describes which colors should be displayed on the screen
  // 0x3F00 to 0x3FFFF
  uint8_t tblPalette[32];
};

#endif // !NES_PPU_H
