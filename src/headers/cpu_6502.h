#ifndef GUARD_CPU_6502_H
#define GUARD_CPU_6502_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

typedef uint8_t reg8_t;
typedef uint16_t address_t;
typedef uint8_t cycles_t;

class Bus;

class cpu_6502 {
 public:
  cpu_6502();
  ~cpu_6502();

  void connectBus(Bus* _bus);

 private:
  reg8_t acc;
  reg8_t x;
  reg8_t y;
  reg8_t status;
  address_t prog_counter;
  reg8_t stack_pointer;
  Bus* bus;

  enum FLAGS {
    C = (1 << 0),  // Carry Bit
    Z = (1 << 1),  // Zero
    I = (1 << 2),  // Disable Interrupts
    D = (1 << 3),  // Decimal Mode (unused for now)
    B = (1 << 4),  // Break
    U = (1 << 5),  // Unused
    V = (1 << 6),  // Overflow
    N = (1 << 7),  // Negative
  };

  struct Instruction {
    std::string name;
    cycles_t (cpu_6502::*operate)(void) =
        nullptr;  // Returns 1 clock cycle if operation requires
    cycles_t (cpu_6502::*addr_mode)(void) =
        nullptr;  // Returns 1 clock cycle if addressing mode requires
    cycles_t cycles = 0;
  };

  void write(address_t addr, reg8_t data);
  reg8_t read(address_t addr);

  reg8_t getFlag(FLAGS flag);
  void setFlag(FLAGS flag, bool set);

  // Addressing Modes
  cycles_t IMP(); cycles_t IMM();
  cycles_t ZP0(); cycles_t ZPX();
  cycles_t ZPY(); cycles_t REL();
  cycles_t ABS(); cycles_t ABX();
  cycles_t ABY(); cycles_t IND();
  cycles_t IZX(); cycles_t IZY();

  // Opcodes
  cycles_t ADC(); cycles_t AND(); cycles_t ASL(); cycles_t BCC();
  cycles_t BCS(); cycles_t BEQ(); cycles_t BIT(); cycles_t BMI();
  cycles_t BNE(); cycles_t BPL(); cycles_t BRK(); cycles_t BVC();
  cycles_t BVS(); cycles_t CLC(); cycles_t CLD(); cycles_t CLI();
  cycles_t CLV(); cycles_t CMP(); cycles_t CPX(); cycles_t CPY();
  cycles_t DEC(); cycles_t DEX(); cycles_t DEY(); cycles_t EOR();
  cycles_t INC(); cycles_t INX(); cycles_t INY(); cycles_t JMP();
  cycles_t JSR(); cycles_t LDA(); cycles_t LDX(); cycles_t LDY();
  cycles_t LSR(); cycles_t NOP(); cycles_t ORA(); cycles_t PHA();
  cycles_t PHP(); cycles_t PLA(); cycles_t PLP(); cycles_t ROL();
  cycles_t ROR(); cycles_t RTI(); cycles_t RTS(); cycles_t SBC();
  cycles_t SEC(); cycles_t SED(); cycles_t SEI(); cycles_t STA();
  cycles_t STX(); cycles_t STY(); cycles_t TAX(); cycles_t TAY();
  cycles_t TSX(); cycles_t TXA(); cycles_t TXS(); cycles_t TYA();

  cycles_t XXX();

  inline void pushToStack(reg8_t data);
  inline reg8_t popFromStack();

  inline void pushProgCounterToStack();
  inline address_t popProgCounterFromStack();

  inline void setLogicalFlags(uint8_t result);
  inline void branch();
  inline void handleInterrupt();

  void clock();
  void reset();
  void irq();
  void nmi();

  reg8_t fetch();
  reg8_t data;  // Working input value to the ALU

  address_t addr_abs;  // All used memory addresses ends up here
  address_t addr_rel;  // Represents absolute address following a branch
  uint8_t opcode;
  cycles_t cycles;  // Cycles left for current instruction

  static const address_t BASE_STACK_ADDRESS;
  static const reg8_t RESET_STACK_POINTER;
  static const address_t RESET_PC_POINTER;
  static const address_t IRQ_PC_POINTER;
  static const address_t NMI_PC_POINTER;
  static const std::vector<Instruction> lookup;
};

#endif
