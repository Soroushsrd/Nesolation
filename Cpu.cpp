//
// Created by rusty on 9/12/25.
//

#include "Cpu.h"
#include <cstdint>

#include "Bus.h"

// reminder:
// Hex | Decimal | Binary
// ----|---------|-------
// 0   |    0    | 0000
// 1   |    1    | 0001
// 2   |    2    | 0010
// 3   |    3    | 0011
// 4   |    4    | 0100
// 5   |    5    | 0101
// 6   |    6    | 0110
// 7   |    7    | 0111
// 8   |    8    | 1000
// 9   |    9    | 1001
// A   |   10    | 1010
// B   |   11    | 1011
// C   |   12    | 1100
// D   |   13    | 1101
// E   |   14    | 1110
// F   |   15    | 1111

Cpu::Cpu() {

  using a = Cpu;
  lookup  = {
      {"BRK", &a::BRK, &a::IMM, 7}, {"ORA", &a::ORA, &a::IZX, 6}, {"???", &a::XXX, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 3}, {"ORA", &a::ORA, &a::ZP0, 3},
      {"ASL", &a::ASL, &a::ZP0, 5}, {"???", &a::XXX, &a::IMP, 5}, {"PHP", &a::PHP, &a::IMP, 3},
      {"ORA", &a::ORA, &a::IMM, 2}, {"ASL", &a::ASL, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 2},
      {"???", &a::NOP, &a::IMP, 4}, {"ORA", &a::ORA, &a::ABS, 4}, {"ASL", &a::ASL, &a::ABS, 6},
      {"???", &a::XXX, &a::IMP, 6}, {"BPL", &a::BPL, &a::REL, 2}, {"ORA", &a::ORA, &a::IZY, 5},
      {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 4},
      {"ORA", &a::ORA, &a::ZPX, 4}, {"ASL", &a::ASL, &a::ZPX, 6}, {"???", &a::XXX, &a::IMP, 6},
      {"CLC", &a::CLC, &a::IMP, 2}, {"ORA", &a::ORA, &a::ABY, 4}, {"???", &a::NOP, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 7}, {"???", &a::NOP, &a::IMP, 4}, {"ORA", &a::ORA, &a::ABX, 4},
      {"ASL", &a::ASL, &a::ABX, 7}, {"???", &a::XXX, &a::IMP, 7}, {"JSR", &a::JSR, &a::ABS, 6},
      {"AND", &a::AND, &a::IZX, 6}, {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8},
      {"BIT", &a::BIT, &a::ZP0, 3}, {"AND", &a::AND, &a::ZP0, 3}, {"ROL", &a::ROL, &a::ZP0, 5},
      {"???", &a::XXX, &a::IMP, 5}, {"PLP", &a::PLP, &a::IMP, 4}, {"AND", &a::AND, &a::IMM, 2},
      {"ROL", &a::ROL, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 2}, {"BIT", &a::BIT, &a::ABS, 4},
      {"AND", &a::AND, &a::ABS, 4}, {"ROL", &a::ROL, &a::ABS, 6}, {"???", &a::XXX, &a::IMP, 6},
      {"BMI", &a::BMI, &a::REL, 2}, {"AND", &a::AND, &a::IZY, 5}, {"???", &a::XXX, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 4}, {"AND", &a::AND, &a::ZPX, 4},
      {"ROL", &a::ROL, &a::ZPX, 6}, {"???", &a::XXX, &a::IMP, 6}, {"SEC", &a::SEC, &a::IMP, 2},
      {"AND", &a::AND, &a::ABY, 4}, {"???", &a::NOP, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 7},
      {"???", &a::NOP, &a::IMP, 4}, {"AND", &a::AND, &a::ABX, 4}, {"ROL", &a::ROL, &a::ABX, 7},
      {"???", &a::XXX, &a::IMP, 7}, {"RTI", &a::RTI, &a::IMP, 6}, {"EOR", &a::EOR, &a::IZX, 6},
      {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 3},
      {"EOR", &a::EOR, &a::ZP0, 3}, {"LSR", &a::LSR, &a::ZP0, 5}, {"???", &a::XXX, &a::IMP, 5},
      {"PHA", &a::PHA, &a::IMP, 3}, {"EOR", &a::EOR, &a::IMM, 2}, {"LSR", &a::LSR, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 2}, {"JMP", &a::JMP, &a::ABS, 3}, {"EOR", &a::EOR, &a::ABS, 4},
      {"LSR", &a::LSR, &a::ABS, 6}, {"???", &a::XXX, &a::IMP, 6}, {"BVC", &a::BVC, &a::REL, 2},
      {"EOR", &a::EOR, &a::IZY, 5}, {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8},
      {"???", &a::NOP, &a::IMP, 4}, {"EOR", &a::EOR, &a::ZPX, 4}, {"LSR", &a::LSR, &a::ZPX, 6},
      {"???", &a::XXX, &a::IMP, 6}, {"CLI", &a::CLI, &a::IMP, 2}, {"EOR", &a::EOR, &a::ABY, 4},
      {"???", &a::NOP, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 7}, {"???", &a::NOP, &a::IMP, 4},
      {"EOR", &a::EOR, &a::ABX, 4}, {"LSR", &a::LSR, &a::ABX, 7}, {"???", &a::XXX, &a::IMP, 7},
      {"RTS", &a::RTS, &a::IMP, 6}, {"ADC", &a::ADC, &a::IZX, 6}, {"???", &a::XXX, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 3}, {"ADC", &a::ADC, &a::ZP0, 3},
      {"ROR", &a::ROR, &a::ZP0, 5}, {"???", &a::XXX, &a::IMP, 5}, {"PLA", &a::PLA, &a::IMP, 4},
      {"ADC", &a::ADC, &a::IMM, 2}, {"ROR", &a::ROR, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 2},
      {"JMP", &a::JMP, &a::IND, 5}, {"ADC", &a::ADC, &a::ABS, 4}, {"ROR", &a::ROR, &a::ABS, 6},
      {"???", &a::XXX, &a::IMP, 6}, {"BVS", &a::BVS, &a::REL, 2}, {"ADC", &a::ADC, &a::IZY, 5},
      {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 4},
      {"ADC", &a::ADC, &a::ZPX, 4}, {"ROR", &a::ROR, &a::ZPX, 6}, {"???", &a::XXX, &a::IMP, 6},
      {"SEI", &a::SEI, &a::IMP, 2}, {"ADC", &a::ADC, &a::ABY, 4}, {"???", &a::NOP, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 7}, {"???", &a::NOP, &a::IMP, 4}, {"ADC", &a::ADC, &a::ABX, 4},
      {"ROR", &a::ROR, &a::ABX, 7}, {"???", &a::XXX, &a::IMP, 7}, {"???", &a::NOP, &a::IMP, 2},
      {"STA", &a::STA, &a::IZX, 6}, {"???", &a::NOP, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 6},
      {"STY", &a::STY, &a::ZP0, 3}, {"STA", &a::STA, &a::ZP0, 3}, {"STX", &a::STX, &a::ZP0, 3},
      {"???", &a::XXX, &a::IMP, 3}, {"DEY", &a::DEY, &a::IMP, 2}, {"???", &a::NOP, &a::IMP, 2},
      {"TXA", &a::TXA, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 2}, {"STY", &a::STY, &a::ABS, 4},
      {"STA", &a::STA, &a::ABS, 4}, {"STX", &a::STX, &a::ABS, 4}, {"???", &a::XXX, &a::IMP, 4},
      {"BCC", &a::BCC, &a::REL, 2}, {"STA", &a::STA, &a::IZY, 6}, {"???", &a::XXX, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 6}, {"STY", &a::STY, &a::ZPX, 4}, {"STA", &a::STA, &a::ZPX, 4},
      {"STX", &a::STX, &a::ZPY, 4}, {"???", &a::XXX, &a::IMP, 4}, {"TYA", &a::TYA, &a::IMP, 2},
      {"STA", &a::STA, &a::ABY, 5}, {"TXS", &a::TXS, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 5},
      {"???", &a::NOP, &a::IMP, 5}, {"STA", &a::STA, &a::ABX, 5}, {"???", &a::XXX, &a::IMP, 5},
      {"???", &a::XXX, &a::IMP, 5}, {"LDY", &a::LDY, &a::IMM, 2}, {"LDA", &a::LDA, &a::IZX, 6},
      {"LDX", &a::LDX, &a::IMM, 2}, {"???", &a::XXX, &a::IMP, 6}, {"LDY", &a::LDY, &a::ZP0, 3},
      {"LDA", &a::LDA, &a::ZP0, 3}, {"LDX", &a::LDX, &a::ZP0, 3}, {"???", &a::XXX, &a::IMP, 3},
      {"TAY", &a::TAY, &a::IMP, 2}, {"LDA", &a::LDA, &a::IMM, 2}, {"TAX", &a::TAX, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 2}, {"LDY", &a::LDY, &a::ABS, 4}, {"LDA", &a::LDA, &a::ABS, 4},
      {"LDX", &a::LDX, &a::ABS, 4}, {"???", &a::XXX, &a::IMP, 4}, {"BCS", &a::BCS, &a::REL, 2},
      {"LDA", &a::LDA, &a::IZY, 5}, {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 5},
      {"LDY", &a::LDY, &a::ZPX, 4}, {"LDA", &a::LDA, &a::ZPX, 4}, {"LDX", &a::LDX, &a::ZPY, 4},
      {"???", &a::XXX, &a::IMP, 4}, {"CLV", &a::CLV, &a::IMP, 2}, {"LDA", &a::LDA, &a::ABY, 4},
      {"TSX", &a::TSX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 4}, {"LDY", &a::LDY, &a::ABX, 4},
      {"LDA", &a::LDA, &a::ABX, 4}, {"LDX", &a::LDX, &a::ABY, 4}, {"???", &a::XXX, &a::IMP, 4},
      {"CPY", &a::CPY, &a::IMM, 2}, {"CMP", &a::CMP, &a::IZX, 6}, {"???", &a::NOP, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 8}, {"CPY", &a::CPY, &a::ZP0, 3}, {"CMP", &a::CMP, &a::ZP0, 3},
      {"DEC", &a::DEC, &a::ZP0, 5}, {"???", &a::XXX, &a::IMP, 5}, {"INY", &a::INY, &a::IMP, 2},
      {"CMP", &a::CMP, &a::IMM, 2}, {"DEX", &a::DEX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 2},
      {"CPY", &a::CPY, &a::ABS, 4}, {"CMP", &a::CMP, &a::ABS, 4}, {"DEC", &a::DEC, &a::ABS, 6},
      {"???", &a::XXX, &a::IMP, 6}, {"BNE", &a::BNE, &a::REL, 2}, {"CMP", &a::CMP, &a::IZY, 5},
      {"???", &a::XXX, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 4},
      {"CMP", &a::CMP, &a::ZPX, 4}, {"DEC", &a::DEC, &a::ZPX, 6}, {"???", &a::XXX, &a::IMP, 6},
      {"CLD", &a::CLD, &a::IMP, 2}, {"CMP", &a::CMP, &a::ABY, 4}, {"NOP", &a::NOP, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 7}, {"???", &a::NOP, &a::IMP, 4}, {"CMP", &a::CMP, &a::ABX, 4},
      {"DEC", &a::DEC, &a::ABX, 7}, {"???", &a::XXX, &a::IMP, 7}, {"CPX", &a::CPX, &a::IMM, 2},
      {"SBC", &a::SBC, &a::IZX, 6}, {"???", &a::NOP, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 8},
      {"CPX", &a::CPX, &a::ZP0, 3}, {"SBC", &a::SBC, &a::ZP0, 3}, {"INC", &a::INC, &a::ZP0, 5},
      {"???", &a::XXX, &a::IMP, 5}, {"INX", &a::INX, &a::IMP, 2}, {"SBC", &a::SBC, &a::IMM, 2},
      {"NOP", &a::NOP, &a::IMP, 2}, {"???", &a::SBC, &a::IMP, 2}, {"CPX", &a::CPX, &a::ABS, 4},
      {"SBC", &a::SBC, &a::ABS, 4}, {"INC", &a::INC, &a::ABS, 6}, {"???", &a::XXX, &a::IMP, 6},
      {"BEQ", &a::BEQ, &a::REL, 2}, {"SBC", &a::SBC, &a::IZY, 5}, {"???", &a::XXX, &a::IMP, 2},
      {"???", &a::XXX, &a::IMP, 8}, {"???", &a::NOP, &a::IMP, 4}, {"SBC", &a::SBC, &a::ZPX, 4},
      {"INC", &a::INC, &a::ZPX, 6}, {"???", &a::XXX, &a::IMP, 6}, {"SED", &a::SED, &a::IMP, 2},
      {"SBC", &a::SBC, &a::ABY, 4}, {"NOP", &a::NOP, &a::IMP, 2}, {"???", &a::XXX, &a::IMP, 7},
      {"???", &a::NOP, &a::IMP, 4}, {"SBC", &a::SBC, &a::ABX, 4}, {"INC", &a::INC, &a::ABX, 7},
      {"???", &a::XXX, &a::IMP, 7},

  };
}

Cpu::~Cpu() {}

uint8_t Cpu::read(uint16_t a) { return bus->read(a, false); }

void Cpu::write(uint16_t a, uint8_t v) { bus->write(a, v); }

// The Pattern
// Clock() increments PC once for the opcode
// Each addressing mode increments PC for each additional byte it needs to read
// The number of increments depends on the addressing mod
void Cpu::Clock() {
  if (cycles == 0) {
    // always set unused flag to 1
    SetFlag(U, true);
    opcode = read(pc);
    // increment pc to read the next instruction
    pc++;
    cycles = lookup[opcode].cycles;
    // some opcodes require additional cycles b ased on the addrmode or operation
    // visible in the cpu sheet
    // any additional pc increments happens inside addressing mode. they can increment pc
    // by 0,1,or 2 if they need to read or write data(?)
    uint8_t additional_cyc1 = (this->*lookup[opcode].addrmode)();
    uint8_t additional_cyc2 = (this->*lookup[opcode].operate)();
    cycles += (additional_cyc1 + additional_cyc2);
  }
  // everytime an opcode is executed, a cycle has been elapsed
  cycles--;
}

//////////// FLAG FUNCTIONS

// Returns the value of a specific bit of the status register
uint8_t Cpu::GetFlag(FLAGS6502 f) const { return ((status & f) > 0) ? 1 : 0; }

// Sets or clears a specific bit of the status register
void Cpu::SetFlag(FLAGS6502 f, bool v) {
  if (v)
    status |= f;
  else
    status &= ~f;
}

//////////// Addressing Modes

// The 6502 can address between 0x0000 - 0xFFFF. The high byte is often referred
// to as the "page", and the low byte is the offset into that page. This implies
// there are 256 pages, each containing 256 bytes.
//
// Several addressing modes have the potential to require an additional clock
// cycle if they cross a page boundary. This is combined with several instructions
// that enable this additional clock cycle. So each addressing function returns
// a flag saying it has potential, as does each instruction. If both instruction
// and address function return 1, then an additional clock cycle is required.

// Address Mode: Implied
// There is no additional data required for this instruction. The instruction
// does something very simple like like sets a status bit.
uint8_t Cpu::IMP() {
  fetched = a;
  return 0;
}

// Adress Mode: Immediate
// the instruction expects the next byte to be used as a value, so we read the address to
// point to the next byte
uint8_t Cpu::IMM() {
  address_abs = pc++;
  return 0;
}

// Address Mode: Zero Page
// To save program bytes, zero page addressing allows you to absolutely address
// a location in first 0xFF bytes of address range.
uint8_t Cpu::ZP0() {
  address_abs = read(pc);
  pc++;
  // masks the address to keep only the lower byte
  address_abs &= 0x00FF;
  return 0;
}

// Address Mode: Zero page with X offset
// the same az ZP0 but the content of X register is added to the supplied single byte address
// used to iterate through ranges within the first page
uint8_t Cpu::ZPX() {
  address_abs = read(pc) + x;
  pc++;
  address_abs &= 0x00FF;
  return 0;
}

// Address Mode: Zero page with y offset
uint8_t Cpu::ZPY() {
  address_abs = read(pc) + y;
  pc++;
  address_abs &= 0x00FF;
}

// Address Mode Relative
// this address mode is exclusive to branch instructions. the addr must reside within
// -128 and +128 of branch instruction-> you cant directly branch to any address in the addressable
// range
// this is used by branch instructions to calculate where to jump, relative to current pos.
uint8_t Cpu::REL() {
  address_rel = read(pc);
  pc++;
  // if bit 7 is set
  if (address_rel & 0x80) {
    address_rel |= 0xFF00;
  }
  return 0;
}

// Addressing Mode: absolute
// a full 16 bit address is loaded and used
uint8_t Cpu::ABS() {
  // 6502 can read one byte at a time. so we read two bytes and map each one to two 16bit values
  // and then using left shift, create one 16bit address
  // Byte 1: Opcode
  // Byte 2: Address low byte
  // Byte 3: Address high byte
  uint16_t lo = read(pc);
  pc++;
  uint16_t hi = read(pc);
  pc++;
  address_abs = (hi << 8) | lo;
}

// Address Mode: Absolute with X offset
// if the resulting address changes the page, we need an additional clock cycle
uint8_t Cpu::ABX() {
  uint16_t lo = read(pc);
  pc++;
  uint16_t hi = read(pc);
  pc++;

  address_abs = (hi << 8) | lo;
  address_abs += x;

  // if the high 8 bits fit into addr absolute, then it doesnt need a page change
  if ((address_abs & 0xFF00) != (hi << 8)) {
    return 1;
  } else {
    return 0;
  }
}

// Addressing Mode: Absolute with Y offset
uint8_t Cpu::ABY() {
  uint16_t lo = read(pc);
  pc++;
  uint16_t hi = read(pc);
  pc++;

  address_abs = (hi << 8) | lo;
  address_abs += y;

  if ((address_abs & 0xFF00) != (hi << 8)) {
    return 1;
  } else {
    return 0;
  }
}

// Addressing Mode: Indirect
// The supplied 16-bit address is read to get the actual 16-bit address. This is
// instruction is unusual in that it has a bug in the hardware! To emulate its
// function accurately, we also need to emulate this bug. If the low byte of the
// supplied address is 0xFF, then to read the high byte of the actual address
// we need to cross a page boundary. This doesnt actually work on the chip as
// designed, instead it wraps back around in the same page, yielding an
// invalid actual address
uint8_t Cpu::IND() {

  // WARNING:
  uint16_t ptr_lo = read(pc);
  pc++;
  uint16_t ptr_hi = read(pc);
  pc++;

  uint16_t ptr = (ptr_hi << 8) | ptr_lo;

  if (ptr_lo == 0x00FF) // simulate page boundary hardware bug!
  {
    address_abs = (read(ptr & 0xFF00) << 8) | read(ptr + 0);
  } else {
    address_abs = (read(ptr + 1) << 8) | read(ptr + 0);
  }
  return 0;
}

// Addressing Mode: Indirect X
// the supplied 8 bit addr is offset by X register to index a location in page 0x00.
// the actual 16bit address is read from this location
uint8_t Cpu::IZX() {
  uint16_t t = read(pc);
  pc++;

  uint16_t lo = read((uint16_t) (t + (uint16_t) x) & 0x00FF);
  uint16_t hi = read((uint16_t) (t + (uint16_t) x + 1) & 0xFF00);

  address_abs = (hi << 8) | lo;
  return 0;
}

// Address Mode: Indirect Y
// The supplied 8-bit address indexes a location in page 0x00. From
// here the actual 16-bit address is read, and the contents of
// Y Register is added to it to offset it. If the offset causes a
// change in page then an additional clock cycle is required.
uint8_t Cpu::IZY() {
  // WARNING:
  uint16_t t = read(pc);
  pc++;

  uint16_t lo = read(t & 0x00FF);
  uint16_t hi = read((t + 1) & 0x00FF);

  address_abs = (hi << 8) | lo;
  address_abs += y;

  if ((address_abs & 0xFF00) != (hi << 8))
    return 1;
  else
    return 0;
}

// sources the data used by the instruction into a convenient numeric variable.
// some instructions dont have to fetch data as the source is implied by the
// instruction. for others, the data resides at the location held within address_abs.
// "fetched" is a global variable for the cpu and is set by this method. it can also
// return it!
uint8_t Cpu::fetch() {
  if (!(lookup[opcode].addrmode == &Cpu::IMP)) {
    fetched = read(address_abs);
  }
  return fetched;
}

//////////// Instructions

// Instruction: Add with carry in
// A = A + N + M + C
// flags out: C, V, N, Z
// A= accumulator register
// M = a value from memory
// C = the carry flag (0 or 1)
//
// this method adds a value to the accumulator and a carry bit. if the result is
// greater than 255, there is an overflow setting in the carry bit! This allows us
// to chain together ADC instructions to add numbers larger than 8 bits. 6502 also
// supports Negative/Positive and signed overflow!
//
// 10000100 = 128 + 4 = 132 normally, but 6502 can assume this word as something else
// if those 8 bits represent the range -127 to +127 (signed). thus the out come of
// 132 is actually wrapped around to -124!
//
// // In principle under the -128 to 127 range:
// 10000000 = -128, 11111111 = -1, 00000000 = 0, 00000000 = +1, 01111111 = +127
// therefore negative numbers have the most significant set, positive numbers do not
//
// To assist us, the 6502 can set the overflow flag, if the result of the addition has
// wrapped around. V <- ~(A^M) & A^(A+M+C)
//
//       Positive Number + Positive Number = Negative Result -> Overflow
//       Negative Number + Negative Number = Positive Result -> Overflow
//       Positive Number + Negative Number = Either Result -> Cannot Overflow
//       Positive Number + Positive Number = Positive Result -> OK! No Overflow
//       Negative Number + Negative Number = Negative Result -> OK! NO Overflow
uint8_t Cpu::ADC() {
  // getting the data that we are adding to the a
  fetch();

  temp = (uint16_t) a + (uint16_t) fetched + (uint16_t) GetFlag(C);
  // the carry flag out is in the high byte bit 0!
  SetFlag(C, temp > 255);
  // if the result is zero, Zero flag is set
  SetFlag(Z, (temp & 0x00FF) == 0);

  // the signed overflow flag is set
  SetFlag(V, (~((uint16_t) a ^ (uint16_t) fetched) & ((uint16_t) a ^ (uint16_t) temp)) & 0x0080);

  // the negative flag is set to the most significant bit of the result
  SetFlag(N, temp & 0x80);

  // setting the 8bit result into acumulator
  a = temp & 0x00FF;
  // can require an additional cycle
  return 1;
}

// Instruction: Subtraction with borrow In
// A = A - M - (1-C)
// flags out = C,V,N,Z
//
// the same as ADC but uses subtraction by multiplying the data by -1.
// A = A - M - (1 - C)  ->  A = A + -1 * (M - (1 - C))  ->  A = A + (-M + 1 + C)
//
// to make a signed positive, negative, we can invert the bits and add 1
//  5 = 00000101
// -5 = 11111010 + 00000001 = 11111011 (or 251 in our 0 to 255 range)
//
uint8_t Cpu::SBC() {
  fetch();

  // using 16 bits to capture carry out
  // Flip all bits of the fetched value (this is ~M)
  // 0x00FF = 11111111 in binary
  // XOR with 0x00FF flips every bit in the bottom 8 bits
  uint16_t value = ((uint16_t) fetched) ^ 0x00FF;
  // Add A + (~M) + C
  //  This gives us A + (~M) + C = A - M - (1-C) = A - M - 1 + C
  //  The -1 part comes from the fact that ~M = -M - 1 in two's complement
  temp = (uint16_t) a + value + (uint16_t) GetFlag(C);
  // if result is overflown, its higher bits will be set
  // setting carry flag (overflown or not)
  SetFlag(C, temp & 0xFF00);
  // setting zero flag
  // checks if lower bits are all zeroes!
  SetFlag(Z, ((temp & 0x00FF) == 0));

  // overflow flag: detecting if signed arithmetic overflowing has occured
  // The XOR operations detect when signs change unexpectedly:
  // (temp ^ a): Did the result have a different sign than A?
  // (temp ^ value): Did the result have a different sign than the inverted value?
  // Both conditions AND bit 7 (0x0080) checks the sign bit
  SetFlag(V, (temp ^ (uint16_t) a) & (temp ^ value) & 0x0080);

  // negative flag: checks if bit 7 is set meaning if this temp is negative
  SetFlag(N, temp & 0x0080);

  // storing only the lower 8 bits back to the accumulator
  a = temp & 0x00FF;
  return 1;
}

// the rest of the instructions are much simpler. they usually work in these steps:
// 1) Fetch the data you are working with
// 2) Perform calculation
// 3) Store the result in desired place
// 4) Set Flags of the status register
// 5) Return if instruction has potential to require additional
//    clock cycle
//

// Instruction: Bitwise logic And
// A = A & M
// flags out : N,Z
uint8_t Cpu::AND() {
  fetch();
  a = a & fetched;

  SetFlag(Z, a == 0x00);

  SetFlag(N, a & 0x80);
  return 1;
}

// Instruction : Arithmetic shift Left
// A = C <- (A<<1)<-0
// flags out = N, Z, C
// shifts all bits one position to left
uint8_t Cpu::ASL() {
  fetch();

  // shift all bits one position to the left
  // equal to multiplying by 2
  temp = (uint16_t) fetched << 1;

  // Set the Carry flag
  //  When we shift left, the original bit 7 gets "shifted out"
  //  If the original bit 7 was 1, it becomes the carry
  //  0xFF00 = 1111111100000000 - checks if bit 8 is set (the shifted-out bit)
  //  Example: 200 (11001000) << 1 = 400 (0000000110010000)
  //           Bit 8 is set, so carry = 1
  SetFlag(Z, (temp & 0xFF00) > 0);

  // set the zero flag
  //  if bottom 8 bits are zero:
  SetFlag(Z, (temp & 0x00FF) == 0x00);

  SetFlag(N, temp & 0x80);

  // storing the result back
  if (lookup[opcode].addrmode == &Cpu::IMP) {
    // if its implied addr mode, we are shifting the accumulator
    a = temp & 0x00FF;
  } else {
    // otherwise shifting a value in memory
    write(address_abs, temp & 0x00FF);
  }
  return 0;
}
