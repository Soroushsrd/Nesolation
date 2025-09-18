
#ifndef NES_CARTRIDGE_H
#define NES_CARTRIDGE_H
#include <cstdint>
class Cartridge {
public:
  Cartridge();
  ~Cartridge();

  // to communicate with the main bus
  bool    cpuWrite(uint16_t address, uint8_t data);
  uint8_t cpuRead(uint16_t address, uint8_t &data);

  // to communicate with the ppu
  bool    ppuWrite(uint16_t address, uint8_t data);
  uint8_t ppuRead(uint16_t address, uint8_t &data);
};

#endif // !NES_CARTRIDGE_H
