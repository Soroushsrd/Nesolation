
#ifndef NES_MAPPER_00_H
#define NES_MAPPER_00_H

#include <cstdint>
#include "Mapper.h"

// the pirpose of the mapper is not to provide any data but to translate the address
// when the cpu writes into itsj
class Mapper_00: public Mapper {
public:
  Mapper_00(uint8_t prgBanks, uint8_t chrBanks);
  ~Mapper_00();

  // to communicate with the main bus
  bool cpuMapWrite(uint16_t address, uint32_t &data);
  bool cpuMapRead(uint16_t address, uint32_t &data);

  // to communicate with the ppu
  bool ppuMapWrite(uint16_t address, uint32_t &data);
  bool ppuMapRead(uint16_t address, uint32_t &data);
};

#endif // !NES_MAPPER_00_H
