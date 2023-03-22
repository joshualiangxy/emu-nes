#include "Bus.h"

Bus::Bus() {
  ram.fill(0x00);
  cpu.connectBus(this);
}

Bus::~Bus() {}

void Bus::write(address_t addr, reg8_t data) {
  if (addr < 0x0000 || addr > 0xFFFF) return;

  ram[addr] = data;
}

reg8_t Bus::read(address_t addr, bool bReadOnly) {
  if (addr < 0x0000 || addr > 0xFFFF) return 0x00;

  return ram[addr];
}
