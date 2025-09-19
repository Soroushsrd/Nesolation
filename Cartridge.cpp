
#include "Cartridge.h"
#include <cstdint>
#include <fstream>
#include <ios>
#include <string>
#include "Mapper_00.h"

Cartridge::Cartridge(const std::string &fileName) {
  // iNES format header
  struct sHeader {
    char    name[4];
    uint8_t prg_rom_chunks;
    uint8_t chr_rom_chunks;
    uint8_t mapper1;
    uint8_t mapper2;
    uint8_t prg_ram_size;
    uint8_t tv_system1;
    uint8_t tv_system2;
    char    unused[5];
  } header;

  bImageValid = false;

  std::ifstream ifs;
  ifs.open(fileName, std::ifstream::binary);
  if (ifs.is_open()) {

    ifs.read((char *) &header, sizeof(sHeader));

    // if a trainer exists we just need to read past it
    // before we get to the actuall data
    // we should read from 512 bytes forward
    if (header.mapper1 & 0x04) {
      ifs.seekg(512, std::ios_base::cur);
    }
    // then we extract which mapper we are going to use
    nMapperID = ((header.mapper2 >> 4) << 4) | (header.mapper1 >> 4);
    mirror    = (header.mapper1 & 0x01) ? VERTICAL : HORIZONTAL;

    // discover file format
    // there are 3 types of file format , 0,1 and 2
    uint8_t nFileType = 1;

    if (nFileType == 0) {
    }

    if (nFileType == 1) {
      // we see how many banks of memory there are for each PRG and CHR
      // then we resize our vectors to that size
      // and read the data into that vector
      nPRGBanks = header.prg_rom_chunks;
      // a single bank of PRG memory is 16kbs
      vPRGMemory.resize(nPRGBanks * 16384);
      ifs.read((char *) vPRGMemory.data(), vPRGMemory.size());

      nCHRBanks = header.chr_rom_chunks;
      // a single bank of CHR memory is 8kbs
      vCHRMemory.resize(nCHRBanks * 8192);
      ifs.read((char *) vCHRMemory.data(), vCHRMemory.size());
    }

    if (nFileType == 2) {
    }

    switch (nMapperID) {
      case 0:
        pMapper = std::make_shared<Mapper_00>(nPRGBanks, nCHRBanks);
        break;
    }

    bImageValid = true;
    ifs.close();
  }
}

Cartridge::~Cartridge() = default;

bool Cartridge::ImageValid() { return bImageValid; }
// WARNING:
bool Cartridge::cpuRead(uint16_t addr, uint8_t &data) {

  uint32_t mapped_addr = 0;
  // if mapper says that info has to come from the cartridge:
  if (pMapper->cpuMapRead(addr, mapped_addr)) {
    data = vPRGMemory[mapped_addr];
    return true;
  }
  return false;
}
bool Cartridge::cpuWrite(uint16_t addr, uint8_t data) {
  uint32_t mapped_addr = 0;
  if (pMapper->cpuMapWrite(addr, mapped_addr)) {
    vPRGMemory[mapped_addr] = data;
    return true;
  } else
    return false;
}

bool Cartridge::ppuRead(uint16_t addr, uint8_t &data) {
  uint32_t mapped_addr = 0;
  // ppu read and writes care about the character memory instead of program memory
  if (pMapper->ppuMapRead(addr, mapped_addr)) {
    data = vCHRMemory[mapped_addr];
    return true;
  } else
    return false;
}

bool Cartridge::ppuWrite(uint16_t addr, uint8_t data) {
  uint32_t mapped_addr = 0;
  if (pMapper->ppuMapRead(addr, mapped_addr)) {
    vCHRMemory[mapped_addr] = data;
    return true;
  } else
    return false;
}
