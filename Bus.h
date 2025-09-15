//
// Created by rusty on 9/12/25.
//

#ifndef NES_BUS_H
#define NES_BUS_H
#include <array>
#include <cstdint>
#include "Cpu.h"


class Bus {
public:
  // devices on the bus
  Cpu                            cpu;
  std::array<uint8_t, 64 * 1024> ram;

  Bus();

  ~Bus();

  void write(uint16_t address, uint8_t data);

  uint8_t read(uint16_t address, bool bReadOnly = false);
};


#endif //NES_BUS_H
