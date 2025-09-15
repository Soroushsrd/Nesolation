//
// Created by rusty on 9/12/25.
//

#ifndef NES_CPU_H
#define NES_CPU_H
#include <cstdint>
#include <string>
#include <vector>

// 6502 CPU
// a cpu will output addresses and will read and write data
// it will also have access to a clock. a clock will force cpu to change its state and its output
// a cpu just responds to the data that is provided and doesnt verify the validity of the data!
//
// A 6502 CPU is capable of outputting a 16-bit wide address and exchanges data 8bits at a time
// a cpu on its own doesnt do anything and must be connected to other components like a bus!

// A bus is a set of wires and connected to this bus is the address lines of the cpu + data lines
// when a cpu sets the address lines to a bus, it expects related devices to respond.
// the full addressable range of the bus is between 0x0000 to 0xFFFF.
// devices connected to the bus must know their address range to be able to respond to it.
// so for example if a device response to an address put on the bus by CPU, it can put data on the
// bus which can then be returned to the CPU.

// INSIDE THE 6502 CPU:
// | A: Accumulator
// | X: register    => these 3 are all 8 bits and store a value.
// | Y: register
//
// Stack pointer    ->  8 bit number that points to addr in the memory
// Program Counter  -> 16 bits and stores the next instruction that cpu must execute
// Status Register

// in 6502 CPU, not all instructions have the same length. there 1,2,and 3 byte instructions
// different instructions take different clock times to take place.
// so each instructions must have a known duration and size.
// 6502 CPU has 56 instructions which can also change their size and duration. this info is set
// in the first byte of the instruction
// so for each instruction, we must emulate its function, cycles and address mode!
//
// instruction table can be shown as a 16 by 16 table (256 instructions) which has 56 filled
// first byte that we read, can be used to index this table: lower 4 bits-> column, higher 4
// bits->row if an instruction matches a field in the table, its a legal one, otherwise considered
// illegal instruction.
//
// 1) read a byte at pc
// 2) opcode[byte] -> addressing mode and cycle numbers
// 3) read additional bytes from the instruction
// 4) execute
// 5) wait and count clock cycles untill instruction is complete

// When to use pc:
//
// Reading the current instruction opcode
// Reading instruction operands(additional bytes that follow the opcode)
// Automatically incremented as you read instruction bytes
// Used for sequential program execution

// forward declaration
class Bus;

class Cpu {
public:
  enum FLAGS6502 {
    C = (1 << 0), // carry bit
    Z = (1 << 1), // zero
    I = (1 << 2), // disable interrupts
    D = (1 << 3), // decimal mode
    B = (1 << 4), // break
    U = (1 << 5), // unused
    V = (1 << 6), // overflow
    N = (1 << 7), // negative
  };

  uint8_t  a      = 0x00; // accumulator register
  uint8_t  x      = 0x00; // x register
  uint8_t  y      = 0x00; // y register
  uint8_t  stkp   = 0x00; // stack pointer
  uint16_t pc     = 0x00; // program counter. this points to the next instruction to be executed
  uint8_t  status = 0x00; // status register

  uint8_t  fetched = 0x00;   // fetched data
  uint16_t temp    = 0x0000; // something to be used everywhere!

  // target memory location that the current instruction will operate on
  // When to use address_abs:
  // After an addressing mode function calculates where data should be read/written
  // For load/store operations, arithmetic operations, etc.
  // It's the "destination" or "source" address for the instruction's data
  uint16_t address_abs = 0x0000; // memory location to be read
  uint16_t address_rel = 0x0000; // relative addr to jump to
  uint8_t  opcode      = 0x00;   // to store the  current opcode
  uint8_t  cycles      = 0x00;   // cycles left for the duration of current instruction

  struct INSTRUCTION {
    std::string name;
    uint8_t (Cpu::*operate)()  = nullptr;
    uint8_t (Cpu::*addrmode)() = nullptr;
    uint8_t cycles             = 0;
  };

  std::vector<INSTRUCTION> lookup;

  Cpu();
  ~Cpu();

  void    ConnectBus(Bus *n) { bus = n; }
  uint8_t GetFlag(FLAGS6502 f) const;
  void    SetFlag(FLAGS6502 f, bool v);

  // External event functions. In hardware these represent pins that are asserted
  // to produce a change in state.
  void reset(); // reset Interrupt forces CPU into known state
  void irq();   // interrupt Request executes an instruction at a specific location
  void nmi();   // non maskable Interrupt Request. As above, but cant be disabled
  void Clock(); // perform one clock cycle's worth of update

  uint8_t fetch(); // used to fetch data

private:
  Bus *bus = nullptr;

  uint8_t read(uint16_t a);
  void    write(uint16_t a, uint8_t v);

  // Addressing Modes =============================================
  // The 6502 has a variety of addressing modes to access data in
  // memory, some of which are direct and some are indirect (like
  // pointers in C++). Each opcode contains information about which
  // addressing mode should be employed to facilitate the
  // instruction, in regards to where it reads/writes the data it
  // uses. The address mode changes the number of bytes that
  // makes up the full instruction, so we implement addressing
  // before executing the instruction, to make sure the program
  // counter is at the correct location, the instruction is
  // primed with the addresses it needs, and the number of clock
  // cycles the instruction requires is calculated. These functions
  // may adjust the number of cycles required depending upon where
  // and how the memory is accessed, so they return the required
  // adjustment.
  uint8_t IMP();
  uint8_t IMM();
  uint8_t ZP0();
  uint8_t ZPX();
  uint8_t ZPY();
  uint8_t REL();
  uint8_t ABS();
  uint8_t ABX();
  uint8_t ABY();
  uint8_t IND();
  uint8_t IZX();
  uint8_t IZY();

  // Opcodes ======================================================
  // There are 56 "legitimate" opcodes provided by the 6502 CPU. As each opcode is
  // defined by 1 byte, there are potentially 256 possible codes.
  // Codes are not used in a "switch case" style on a processor,
  // instead they are repsonisble for switching individual parts of
  // CPU circuits on and off.
  //
  // These functions return 0 normally, but some are capable of
  // requiring more clock cycles when executed under certain
  // conditions combined with certain addressing modes. If that is
  // the case, they return 1.

  uint8_t ADC();
  uint8_t AND();
  uint8_t ASL();
  uint8_t BCC();
  uint8_t BCS();
  uint8_t BEQ();
  uint8_t BIT();
  uint8_t BMI();
  uint8_t BNE();
  uint8_t BPL();
  uint8_t BRK();
  uint8_t BVC();
  uint8_t BVS();
  uint8_t CLC();
  uint8_t CLD();
  uint8_t CLI();
  uint8_t CLV();
  uint8_t CMP();
  uint8_t CPX();
  uint8_t CPY();
  uint8_t DEC();
  uint8_t DEX();
  uint8_t DEY();
  uint8_t EOR();
  uint8_t INC();
  uint8_t INX();
  uint8_t INY();
  uint8_t JMP();
  uint8_t JSR();
  uint8_t LDA();
  uint8_t LDX();
  uint8_t LDY();
  uint8_t LSR();
  uint8_t NOP();
  uint8_t ORA();
  uint8_t PHA();
  uint8_t PHP();
  uint8_t PLA();
  uint8_t PLP();
  uint8_t ROL();
  uint8_t ROR();
  uint8_t RTI();
  uint8_t RTS();
  uint8_t SBC();
  uint8_t SEC();
  uint8_t SED();
  uint8_t SEI();
  uint8_t STA();
  uint8_t STX();
  uint8_t STY();
  uint8_t TAX();
  uint8_t TAY();
  uint8_t TSX();
  uint8_t TXA();
  uint8_t TXS();
  uint8_t TYA();

  // for the illegal opcodes:
  uint8_t XXX();
};

#endif // NES_CPU_H
