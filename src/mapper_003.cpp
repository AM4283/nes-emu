#include "mapper_003.h"

Mapper_003::Mapper_003(uint8_t prgBanks, uint8_t chrBanks) : Mapper(prgBanks, chrBanks) {}
Mapper_003::~Mapper_003(){}

bool Mapper_003::cpuMapRead(uint16_t addr, uint32_t &mapped_addr, uint8_t &data) {
    if (addr >= 0x8000 && addr <= 0xffff) {
        if (nPRGBanks == 1) // 16kb rom
            mapped_addr = addr & 0x3fff;
        if (nPRGBanks == 2) // 32kb rom
            mapped_addr = addr & 0x7fff;
        return true;
    } else {
        return false;
    }
}

bool Mapper_003::cpuMapWrite(uint16_t addr, uint32_t &mapped_addr, uint8_t data) {
    if (addr >= 0x8000 && addr <= 0xffff) {
        nCHRBankSelect = data & 0x03;
        mapped_addr = addr;
    }
    // mapper handled a write but do not update ROM
    return false;
}

bool Mapper_003::ppuMapRead(uint16_t addr, uint32_t &mapped_addr) {
    if (addr < 0x2000) {
        mapped_addr = nCHRBankSelect * 0x2000 + addr;
        return true;
    } else {
        return false;
    }
}

bool Mapper_003::ppuMapWrite(uint16_t addr, uint32_t &mapped_addr) {
    return false;
}

void Mapper_003::reset() {
    nCHRBankSelect = 0;
}