
#ifndef NES_MAPPER_H
#define NES_MAPPER_H
#include <cstdint>
class Mapper {
public:
  Mapper(uint8_t prgBanks, uint8_t chrBanks);
  ~Mapper();

  // mapper is an abstract base class . the purpose of these functions is to take input addr
  // from the respective cpu or ppu bus and transform it into an addr through which i can index
  // the ROMs that i have read in from the file
  // if the addr is successfully mapped, they return true
  // to communicate with the main bus
  virtual bool cpuMapWrite(uint16_t address, uint32_t &data) = 0;
  virtual bool cpuMapRead(uint16_t address, uint32_t &data)  = 0;

  // to communicate with the ppu
  virtual bool ppuMapWrite(uint16_t address, uint32_t &data) = 0;
  virtual bool ppuMapRead(uint16_t address, uint32_t &data)  = 0;

protected:
  uint8_t nPRGBanks = 0;
  uint8_t nCHRBanks = 0;
};

#endif
