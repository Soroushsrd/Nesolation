//
// Created by rusty on 9/12/25.
//

#ifndef NES_BUS_H
#define NES_BUS_H
#include <array>
#include <cstdint>
#include <memory>
#include "Cartridge.h"
#include "Cpu.h"
#include "Ppu.h"

//

class Bus {
public:
  // devices on the bus
  Cpu cpu;
  // picture processing unit
  // it has its own bus!
  Ppu ppu;
  // cartridge
  // program ROM. program itself and its graphical needs are stored on the cartridge
  std::shared_ptr<Cartridge> cart;
  // Nes has about 2kbs of RAM
  // accessible from thje range 0x0000 to 0x1FFF (8kb range)
  // the reason bnehind the diff between 2kb and 8kb is that RAM uses mirroring!
  // even though the range of the ram is from 0 to 8kbs, the first 2kb of that is mapped to
  // the hardware and each 2kbs chunk after that map to the same initial 2kb mapped to hardware
  // they MIRROR the first range
  std::array<uint8_t, 2 * 1024> ram;

  Bus();
  ~Bus();

  void    write(uint16_t address, uint8_t data);
  uint8_t read(uint16_t address, bool bReadOnly = false);

  // loads the cartridge into memory
  void insertCartridge(const std::shared_ptr<Cartridge> &cartridge);
  void reset();
  void clock();

private:
  // keeps the amount of time the clock function has been called. it resets when reset is called
  uint32_t nSystemClockCounter = 0;
};

#endif // NES_BUS_H
