#ifndef GUARD_BUS_H
#define GUARD_BUS_H

#include <array>
#include <cstdint>

#include "cpu_6502.h"

class Bus {
 public:
  Bus();
  ~Bus();

  void write(address_t addr, uint8_t data);
  uint8_t read(address_t addr, bool bReadOnly = false);

 private:
  cpu_6502 cpu;
  std::array<uint8_t, 64 * 1024> ram;
};

#endif
