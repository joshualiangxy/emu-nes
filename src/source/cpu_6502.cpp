#include "cpu_6502.h"

#include "Bus.h"

const address_t cpu_6502::BASE_STACK_ADDRESS = 0x0100;
const reg8_t cpu_6502::RESET_STACK_POINTER = 0xFD;
const address_t cpu_6502::RESET_PC_POINTER = 0xFFFC;
const address_t cpu_6502::IRQ_PC_POINTER = 0xFFFE;
const address_t cpu_6502::NMI_PC_POINTER = 0xFFFA;
const std::vector<cpu_6502::Instruction> cpu_6502::lookup = {
    /*    */ /* x0              */ /* x1              */ /* x2              */ /* x3              */ /* x4              */ /* x5              */ /* x6              */ /* x7              */ /* x8              */ /* x9              */ /* xA              */ /* xB              */ /* xC              */ /* xD              */ /* xE              */ /* xF              */
    /* 0x */ {"BRK", BRK, IMP, 7}, {"ORA", ORA, IZX, 6}, {"STP", XXX, IMP, 2}, {"SLO", XXX, IZX, 8}, {"NOP", XXX, ZP0, 3}, {"ORA", ORA, ZP0, 3}, {"ASL", ASL, ZP0, 5}, {"SLO", XXX, ZP0, 5}, {"PHP", PHP, IMP, 3}, {"ORA", ORA, IMM, 2}, {"ASL", ASL, IMP, 2}, {"ANC", XXX, IMM, 2}, {"NOP", XXX, ABS, 4}, {"ORA", ORA, ABS, 4}, {"ASL", ASL, ABS, 6}, {"SLO", XXX, ABS, 6},
    /* 1x */ {"BPL", BPL, REL, 2}, {"ORA", ORA, IZY, 5}, {"STP", XXX, IMP, 2}, {"SLO", XXX, IZY, 8}, {"NOP", XXX, ZPX, 4}, {"ORA", ORA, ZPX, 4}, {"ASL", ASL, ZPX, 6}, {"SLO", XXX, ZPX, 6}, {"CLC", CLC, IMP, 2}, {"ORA", ORA, ABY, 4}, {"NOP", XXX, IMP, 2}, {"SLO", XXX, ABY, 7}, {"NOP", XXX, ABX, 4}, {"ORA", ORA, ABX, 4}, {"ASL", ASL, ABX, 7}, {"SLO", XXX, ABX, 7},
    /* 2x */ {"JSR", JSR, ABS, 6}, {"AND", AND, IZX, 6}, {"STP", XXX, IMP, 2}, {"RLA", XXX, IZX, 8}, {"BIT", BIT, ZP0, 3}, {"AND", AND, ZP0, 3}, {"ROL", ROL, ZP0, 5}, {"RLA", XXX, ZP0, 5}, {"PLP", PLP, IMP, 4}, {"AND", AND, IMM, 2}, {"ROL", ROL, IMP, 2}, {"ANC", XXX, IMM, 2}, {"BIT", BIT, ABS, 4}, {"AND", AND, ABS, 4}, {"ROL", ROL, ABS, 6}, {"RLA", XXX, ABS, 6},
    /* 3x */ {"BMI", BMI, REL, 2}, {"AND", AND, IZY, 5}, {"STP", XXX, IMP, 2}, {"RLA", XXX, IZY, 8}, {"NOP", XXX, ZPX, 4}, {"AND", AND, ZPX, 4}, {"ROL", ROL, ZPX, 6}, {"RLA", XXX, ZPX, 6}, {"SEC", SEC, IMP, 2}, {"AND", AND, ABY, 4}, {"NOP", XXX, IMP, 2}, {"RLA", XXX, ABY, 7}, {"NOP", XXX, ABX, 4}, {"AND", AND, ABX, 4}, {"ROL", ROL, ABX, 7}, {"RLA", XXX, ABX, 7},
    /* 4x */ {"RTI", RTI, IMP, 6}, {"EOR", EOR, IZX, 6}, {"STP", XXX, IMP, 2}, {"SRE", XXX, IZX, 8}, {"NOP", XXX, ZP0, 3}, {"EOR", EOR, ZP0, 3}, {"LSR", LSR, ZP0, 5}, {"SRE", XXX, ZP0, 5}, {"PHA", PHA, IMP, 3}, {"EOR", EOR, IMM, 2}, {"LSR", LSR, IMP, 2}, {"ALR", XXX, IMM, 2}, {"JMP", JMP, ABS, 3}, {"EOR", EOR, ABS, 4}, {"LSR", LSR, ABS, 6}, {"SRE", XXX, ABS, 6},
    /* 5x */ {"BVC", BVC, REL, 2}, {"EOR", EOR, IZY, 5}, {"STP", XXX, IMP, 2}, {"SRE", XXX, IZY, 8}, {"NOP", XXX, ZPX, 4}, {"EOR", EOR, ZPX, 4}, {"LSR", LSR, ZPX, 6}, {"SRE", XXX, ZPX, 6}, {"CLI", CLI, IMP, 2}, {"EOR", EOR, ABY, 4}, {"NOP", XXX, IMP, 2}, {"SRE", XXX, ABY, 7}, {"NOP", XXX, ABX, 4}, {"EOR", EOR, ABX, 4}, {"LSR", LSR, ABX, 7}, {"SRE", XXX, ABX, 7},
    /* 6x */ {"RTS", RTS, IMP, 6}, {"ADC", ADC, IZX, 6}, {"STP", XXX, IMP, 2}, {"RRA", XXX, IZX, 8}, {"NOP", XXX, ZP0, 3}, {"ADC", ADC, ZP0, 3}, {"ROR", ROR, ZP0, 5}, {"RRA", XXX, ZP0, 5}, {"PLA", PLA, IMP, 4}, {"ADC", ADC, IMM, 2}, {"ROR", ROR, IMP, 2}, {"ARR", XXX, IMM, 2}, {"JMP", JMP, IND, 5}, {"ADC", ADC, ABS, 4}, {"ROR", ROR, ABS, 6}, {"RRA", XXX, ABS, 6},
    /* 7x */ {"BVS", BVS, REL, 2}, {"ADC", ADC, IZY, 5}, {"STP", XXX, IMP, 2}, {"RRA", XXX, IZY, 8}, {"NOP", XXX, ZPX, 4}, {"ADC", ADC, ZPX, 4}, {"ROR", ROR, ZPX, 6}, {"RRA", XXX, ZPX, 6}, {"SEI", SEI, IMP, 2}, {"ADC", ADC, ABY, 4}, {"NOP", XXX, IMP, 2}, {"RRA", XXX, ABY, 7}, {"NOP", XXX, ABX, 4}, {"ADC", ADC, ABX, 4}, {"ROR", ROR, ABX, 7}, {"RRA", XXX, ABX, 7},
    /* 8x */ {"NOP", XXX, IMM, 2}, {"STA", STA, IZX, 6}, {"NOP", XXX, IMM, 2}, {"SAX", XXX, IZX, 6}, {"STY", STY, ZP0, 3}, {"STA", STA, ZP0, 3}, {"STX", STX, ZP0, 3}, {"SAX", XXX, ZP0, 3}, {"DEY", DEY, IMP, 2}, {"NOP", XXX, IMM, 2}, {"TXA", TXA, IMP, 2}, {"XAA", XXX, IMM, 2}, {"STY", STY, ABS, 4}, {"STA", STA, ABS, 4}, {"STX", STX, ABS, 4}, {"SAX", XXX, ABS, 4},
    /* 9x */ {"BCC", BCC, REL, 2}, {"STA", STA, IZY, 6}, {"STP", XXX, IMP, 2}, {"AHX", XXX, IZY, 6}, {"STY", STY, ZPX, 4}, {"STA", STA, ZPX, 4}, {"STX", STX, ZPY, 4}, {"SAX", XXX, ZPY, 4}, {"TYA", TYA, IMP, 2}, {"STA", STA, ABY, 5}, {"TXS", TXS, IMP, 2}, {"TAS", XXX, ABY, 5}, {"SHY", XXX, ABX, 5}, {"STA", STA, ABX, 5}, {"SHX", XXX, ABY, 5}, {"AHX", XXX, ABY, 5},
    /* Ax */ {"LDY", LDY, IMM, 2}, {"LDA", LDA, IZX, 6}, {"LDX", LDX, IMM, 2}, {"LAX", XXX, IZX, 6}, {"LDY", LDY, ZP0, 3}, {"LDA", LDA, ZP0, 3}, {"LDX", LDX, ZP0, 3}, {"LAX", XXX, ZP0, 3}, {"TAY", TAY, IMP, 2}, {"LDA", LDA, IMM, 2}, {"TAX", TAX, IMP, 2}, {"LAX", XXX, IMM, 2}, {"LDY", LDY, ABS, 4}, {"LDA", LDA, ABS, 4}, {"LDX", LDX, ABS, 4}, {"LAX", XXX, ABS, 4},
    /* Bx */ {"BCS", BCS, REL, 2}, {"LDA", LDA, IZY, 5}, {"STP", XXX, IMP, 2}, {"LAX", XXX, IZY, 5}, {"LDY", LDY, ZPX, 4}, {"LDA", LDA, ZPX, 4}, {"LDX", LDX, ZPY, 4}, {"LAX", XXX, ZPY, 4}, {"CLV", CLV, IMP, 2}, {"LDA", LDA, ABY, 4}, {"TSX", TSX, IMP, 2}, {"LAS", XXX, ABY, 4}, {"LDY", LDY, ABX, 4}, {"LDA", LDA, ABX, 4}, {"LDX", LDX, ABY, 4}, {"LAX", XXX, ABY, 4},
    /* Cx */ {"CPY", CPY, IMM, 2}, {"CMP", CMP, IZX, 6}, {"NOP", XXX, IMM, 2}, {"DCP", XXX, IZX, 8}, {"CPY", CPY, ZP0, 3}, {"CMP", CMP, ZP0, 3}, {"DEC", DEC, ZP0, 5}, {"DCP", XXX, ZP0, 5}, {"INY", INY, IMP, 2}, {"CMP", CMP, IMM, 2}, {"DEX", DEX, IMP, 2}, {"AXS", XXX, IMM, 2}, {"CPY", CPY, ABS, 4}, {"CMP", CMP, ABS, 4}, {"DEC", DEC, ABS, 6}, {"DCP", XXX, ABS, 6},
    /* Dx */ {"BNE", BNE, REL, 2}, {"CMP", CMP, IZY, 5}, {"STP", XXX, IMP, 2}, {"DCP", XXX, IZY, 8}, {"NOP", XXX, ZPX, 4}, {"CMP", CMP, ZPX, 4}, {"DEC", DEC, ZPX, 6}, {"DCP", XXX, ZPX, 6}, {"CLD", CLD, IMP, 2}, {"CMP", CMP, ABY, 4}, {"NOP", XXX, IMP, 2}, {"DCP", XXX, ABY, 7}, {"NOP", XXX, ABX, 4}, {"CMP", CMP, ABX, 4}, {"DEC", DEC, ABX, 7}, {"DCP", XXX, ABX, 7},
    /* Ex */ {"CPX", CPX, IMM, 2}, {"SBC", SBC, IZX, 6}, {"NOP", XXX, IMM, 2}, {"ISC", XXX, IZX, 8}, {"CPX", CPX, ZP0, 3}, {"SBC", SBC, ZP0, 3}, {"INC", INC, ZP0, 5}, {"ISC", XXX, ZP0, 5}, {"INX", INX, IMP, 2}, {"SBC", SBC, IMM, 2}, {"NOP", NOP, IMP, 2}, {"SBC", XXX, IMM, 2}, {"CPX", CPX, ABS, 4}, {"SBC", SBC, ABS, 4}, {"INC", INC, ABS, 6}, {"ISC", XXX, ABS, 6},
    /* Fx */ {"BEQ", BEQ, REL, 2}, {"SBC", SBC, IZY, 5}, {"STP", XXX, IMP, 2}, {"ISC", XXX, IZY, 8}, {"NOP", XXX, ZPX, 4}, {"SBC", SBC, ZPX, 4}, {"INC", INC, ZPX, 6}, {"ISC", XXX, ZPX, 6}, {"SED", SED, IMP, 2}, {"SBC", SBC, ABY, 4}, {"NOP", XXX, IMP, 2}, {"ISC", XXX, ABY, 7}, {"NOP", XXX, ABX, 4}, {"SBC", SBC, ABX, 4}, {"INC", INC, ABX, 7}, {"ISC", XXX, ABX, 7},
};

inline bool isSameAddressPage(address_t addr1, address_t addr2) {
  return (addr1 & 0xFF00) == (addr2 & 0xFF00);
}

cpu_6502::cpu_6502()
    : acc{0x00},
      x{0x00},
      y{0x00},
      status{0x00},
      prog_counter{0x0000},
      stack_pointer{0x00},
      bus{nullptr},
      data{0x00},
      addr_abs{0x0000},
      addr_rel{0x0000},
      opcode{0x00},
      cycles{0} {}

cpu_6502::~cpu_6502() {}

void cpu_6502::connectBus(Bus* _bus) { bus = _bus; }

void cpu_6502::write(address_t addr, reg8_t data) { bus->write(addr, data); }

reg8_t cpu_6502::read(address_t addr) { bus->read(addr); }

void cpu_6502::clock() {
  if (cycles > 0) {
    --cycles;
    return;
  }

  opcode = read(prog_counter++);

  // Get Starting number of cycles
  const Instruction& instruction = lookup[opcode];
  cycles_t addr_mode_cycles = (this->*instruction.addr_mode)();
  cycles_t operation_cycles = (this->*instruction.operate)();

  // Only add 1 clock cycle if both addr_mode and operate includes 1 extra cycle
  // 1 cycles elapsed during clock()
  cycles = instruction.cycles + (addr_mode_cycles & operation_cycles) - 1;
}

void cpu_6502::reset() {
  acc = 0x00;
  x = 0x00;
  y = 0x00;
  stack_pointer = RESET_STACK_POINTER;
  status = 0x00 | FLAGS::U;

  address_t lo = read(RESET_PC_POINTER);
  address_t hi = ((address_t)read(RESET_PC_POINTER + 1)) << 8;
  prog_counter = hi | lo;

  addr_rel = 0x0000;
  addr_abs = 0x0000;
  data = 0x00;

  cycles = 8;
}

inline void cpu_6502::handleInterrupt() {
  pushProgCounterToStack();

  setFlag(FLAGS::B, 0);
  setFlag(FLAGS::U, 1);
  setFlag(FLAGS::I, 1);

  pushToStack(status);

  address_t lo = read(addr_abs);
  address_t hi = ((address_t)read(addr_abs + 1)) << 8;
  prog_counter = hi | lo;
}

void cpu_6502::irq() {
  if (getFlag(FLAGS::I) == 1) {
    return;
  }

  addr_abs = IRQ_PC_POINTER;
  handleInterrupt();
  cycles = 7;
}

void cpu_6502::nmi() {
  addr_abs = NMI_PC_POINTER;
  handleInterrupt();
  cycles = 8;
}

reg8_t cpu_6502::getFlag(FLAGS flag) { return (status & flag) > 0 ? 1 : 0; }

void cpu_6502::setFlag(FLAGS flag, bool set) {
  if (set) {
    status |= flag;
  } else {
    status &= ~flag;
  }
}

/**************************************************
 * Addressing Modes                               *
 **************************************************/

inline void cpu_6502::pushToStack(reg8_t data) { write(BASE_STACK_ADDRESS + stack_pointer--, data); }

inline reg8_t cpu_6502::popFromStack() {
  return read(BASE_STACK_ADDRESS + ++stack_pointer);
}

inline void cpu_6502::pushProgCounterToStack() {
  pushToStack((prog_counter >> 8) & 0x00FF);
  pushToStack(prog_counter & 0x00FF);
}

inline address_t cpu_6502::popProgCounterFromStack() {
  address_t prog_counter = (address_t)popFromStack();
  prog_counter |= (address_t)popFromStack() << 8;

  return prog_counter;
}

inline void cpu_6502::setLogicalFlags(uint8_t result) {
  setFlag(FLAGS::Z, result == 0x00);
  setFlag(FLAGS::N, result & 0x80);
}

inline void cpu_6502::branch() {
  ++cycles;
  addr_abs = prog_counter + addr_rel;

  // If branching to different page, incur another cycle
  if (!isSameAddressPage(addr_abs, prog_counter)) {
    ++cycles;
  }

  prog_counter = addr_abs;
}

cycles_t cpu_6502::IMP() {
  data = acc;
  return 0;
}

cycles_t cpu_6502::IMM() {
  addr_abs = prog_counter++;
  return 0;
}

cycles_t cpu_6502::ZP0() {
  addr_abs = read(prog_counter++) & 0x00FF;
  return 0;
}

cycles_t cpu_6502::ZPX() {
  addr_abs = (read(prog_counter++) + x) & 0x00FF;
  return 0;
}

cycles_t cpu_6502::ZPY() {
  addr_abs = (read(prog_counter++) + y) & 0x00FF;
  return 0;
}

cycles_t cpu_6502::REL() {
  addr_rel = read(prog_counter++);

  // Keeping negative numbers negative ==>
  // read returns 8 bits, if MSB is set, that means number is -ve
  // (2s complement) we should then set upper 8 bits of the 16 bit address_t
  // to keep the number negative
  if (addr_rel & 0x80) {
    addr_rel |= 0xFF00;
  }

  return 0;
}

cycles_t cpu_6502::ABS() {
  address_t lo = read(prog_counter++);
  address_t hi = ((address_t)read(prog_counter++)) << 8;

  addr_abs = hi | lo;
  return 0;
}

cycles_t cpu_6502::ABX() {
  address_t lo = read(prog_counter++);
  address_t hi = ((address_t)read(prog_counter++)) << 8;

  addr_abs = hi | lo;
  addr_abs += x;

  // If page changed (overflowed), additional cycle needed
  return isSameAddressPage(addr_abs, hi) ? 1 : 0;
}

cycles_t cpu_6502::ABY() {
  address_t lo = read(prog_counter++);
  address_t hi = ((address_t)read(prog_counter++)) << 8;

  addr_abs = hi | lo;
  addr_abs += y;

  // If page changed (overflowed), additional cycle needed
  return isSameAddressPage(addr_abs, hi) ? 1 : 0;
}

cycles_t cpu_6502::IND() {
  address_t ptr_lo = read(prog_counter++);
  address_t ptr_hi = ((address_t)read(prog_counter++)) << 8;
  address_t ptr = ptr_hi | ptr_lo;

  address_t lo = read(ptr);
  address_t hi;

  // Simulate page boundary hardware bug
  if (ptr_lo == 0x00FF) {
    hi = read(ptr & 0xFF00);
  } else {
    hi = read(ptr + 1);
  }
  hi <<= 8;

  addr_abs = hi | lo;
  return 0;
}

cycles_t cpu_6502::IZX() {
  address_t ptr = read(prog_counter++);
  address_t ptr_lo = (ptr + (address_t)x) & 0x00FF;
  address_t ptr_hi = (ptr + (address_t)x + 1) & 0x00FF;

  address_t lo = read(ptr_lo);
  address_t hi = ((address_t)read(ptr_hi)) << 8;

  addr_abs = hi | lo;
  return 0;
}

cycles_t cpu_6502::IZY() {
  address_t ptr = read(prog_counter++);
  address_t ptr_lo = ptr & 0x00FF;
  address_t ptr_hi = (ptr + 1) & 0x00FF;

  address_t lo = read(ptr_lo);
  address_t hi = ((address_t)read(ptr_hi)) << 8;

  addr_abs = hi | lo;
  addr_abs += y;

  // If page changed (overflowed), additional cycle needed
  return isSameAddressPage(addr_abs, hi) ? 1 : 0;
}

/**************************************************
 * Instructions                                   *
 **************************************************/

reg8_t cpu_6502::fetch() {
  if (lookup[opcode].addr_mode != IMP) data = read(addr_abs);

  return data;
}

cycles_t cpu_6502::BCC() {
  if (getFlag(FLAGS::C) == 0) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BCS() {
  if (getFlag(FLAGS::C) == 1) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BEQ() {
  if (getFlag(FLAGS::Z) == 1) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BMI() {
  if (getFlag(FLAGS::N) == 1) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BNE() {
  if (getFlag(FLAGS::Z) == 0) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BPL() {
  if (getFlag(FLAGS::N) == 0) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BVC() {
  if (getFlag(FLAGS::V) == 0) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::BVS() {
  if (getFlag(FLAGS::V) == 1) {
    branch();
  }

  return 0;
}

cycles_t cpu_6502::CLC() {
  setFlag(FLAGS::C, false);
  return 0;
}

cycles_t cpu_6502::CLD() {
  setFlag(FLAGS::D, false);
  return 0;
}

cycles_t cpu_6502::CLI() {
  setFlag(FLAGS::I, false);
  return 0;
}

cycles_t cpu_6502::CLV() {
  setFlag(FLAGS::V, false);
  return 0;
}

cycles_t cpu_6502::SEC() {
  setFlag(FLAGS::C, true);
  return 0;
}

cycles_t cpu_6502::SED() {
  setFlag(FLAGS::D, true);
  return 0;
}

cycles_t cpu_6502::SEI() {
  setFlag(FLAGS::I, true);
  return 0;
}
