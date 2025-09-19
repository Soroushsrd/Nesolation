
#ifndef NES_CARTRIDGE_H
#define NES_CARTRIDGE_H
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
// #include "Mapper.h"
#include "Mapper_00.h"

class Cartridge {
public:
  enum MIRROR {
    HORIZONTAL,
    VERTICAL,
    ONSCREEN_LO,
    ONSCREEN_HI,
  } mirror = HORIZONTAL;

  Cartridge(const std::string &fileName);
  ~Cartridge();

  bool ImageValid();

  // to communicate with the main bus
  bool cpuWrite(uint16_t address, uint8_t data);
  bool cpuRead(uint16_t address, uint8_t &data);

  // to communicate with the ppu
  bool ppuWrite(uint16_t address, uint8_t data);
  bool ppuRead(uint16_t address, uint8_t &data);

private:
  // prg memory banks on  the cartridge accessible by cpu
  std::vector<uint8_t> vPRGMemory;
  // chr memory banks on  the cartridge accessible by ppu
  std::vector<uint8_t>    vCHRMemory;
  std::shared_ptr<Mapper> pMapper;
  // storing which mapper we're using
  uint8_t nMapperID = 0;
  // how many banks each memory contains
  uint8_t nPRGBanks = 0;
  uint8_t nCHRBanks = 0;

  bool bImageValid = false;
};

#endif // !NES_CARTRIDGE_H
