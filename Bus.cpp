//
// Created by rusty on 9/12/25.
//

#include "Bus.h"
#include "Cartridge.h"

Bus::Bus() {
  for (auto &i: ram) {
    i = 0x00;
  }
  cpu.ConnectBus(this);
}

Bus::~Bus() {}

void Bus::write(uint16_t address, uint8_t data) {
  if (cart->cpuWrite(address, data)) {
    // The cartridge "sees all" and has the facility to veto
    // the propagation of the bus transaction if it requires.
    // This allows the cartridge to map any address to some
    // other data, including the facility to divert transactions
    // with other physical devices.
  } else if (address >= 0x0000 && address <= 0x1FFF) {
    // System RAM Address Range. The range covers 8KB, though
    // there is only 2KB available. That 2KB is "mirrored"
    // through this address range. Using bitwise AND to mask
    // the bottom 11 bits is the same as addr % 2048.
    ram[address & 0x07FF] = data;

  } else if (address >= 0x2000 && address <= 0x3FFF) {
    // PPU Address range. The PPU only has 8 primary registers
    // and these are repeated throughout this range. We can
    // use bitwise AND operation to mask the bottom 3 bits,
    // which is the equivalent of addr % 8.
    ppu.cpuWrite(address & 0x0007, data);
  }
}

uint8_t Bus::read(uint16_t address, bool bReadOnly) {
  uint8_t data = 0x00;
  if (cart->cpuRead(address, data)) {
    // read cartridge:

    // for ram addresses
  } else if (address >= 0x0000 && address <= 0x1FFF) {
    // implementing the mirroring. we use &0x1FFF which is 2kb range mask to get the
    // address % 2kb result which will always be less than 2kb and a valid address
    data = ram[address & 0x1FFF];
    // for ppu addresses
  } else if (address >= 0x2000 && address <= 0x3FFF) {
    data = ppu.cpuRead(address & 0x0007, bReadOnly);
  }

  return data;
}

void Bus::insertCartridge(const std::shared_ptr<Cartridge> &cartridge) {
  this->cart = cartridge;
  ppu.ConnectCartridge(cartridge);
}
void Bus::reset() {
  cpu.reset();
  nSystemClockCounter = 0;
}

void Bus::clock() {
  // the running freq is controlled by what ever calls this function.
  // we divide the clock as necessary and call the peripheral devices clock()
  // function at the correct times.
  // the fastest clock freq that the digital system caresa about is eq to the ppu clock
  // so the ppu is clocked each time this function is called!
  ppu.clock();

  // cpu runs 3 times slower than ppu so we only call its clock function every 3 times this
  // function is called!
  if (nSystemClockCounter % 3 == 0) {
    cpu.Clock();
  }

  nSystemClockCounter++;
}
