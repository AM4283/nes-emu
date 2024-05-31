#include "mapper_002.h"

Mapper_002::Mapper_002(uint8_t prgBanks, uint8_t chrBanks) : Mapper (prgBanks, chrBanks) {}
Mapper_002::~Mapper_002(){}

bool Mapper_002::cpuMapRead(uint16_t addr, uint32_t &mapped_addr, uint8_t &data) {
    if (addr >= 0x8000 && addr <= 0xbfff) {
        mapped_addr = nPRGBankSelectLo * 0x4000 + (addr & 0x3fff);
        return true;
    }
    if (addr >= 0xc000 && addr <= 0xffff) {
        mapped_addr = nPRGBankSelectHi * 0x4000 + (addr & 0x3fff);
        return true;
    }
    return false;
}

bool Mapper_002::cpuMapWrite(uint16_t addr, uint32_t &mapped_addr, uint8_t data) {
    if (addr >= 0x8000 && addr <= 0xffff) {
        nPRGBankSelectLo = data & 0x0f;
    }
    // mapper has handled write but do not update ROMs
    return false;
}

bool Mapper_002::ppuMapRead(uint16_t addr, uint32_t &mapped_addr) {
    if (addr < 0x2000) {
        mapped_addr = addr;
        return true;
    } else {
        return true;
    }
}

bool Mapper_002::ppuMapWrite(uint16_t addr, uint32_t &mapped_addr) {
    if (addr < 0x2000) {
        if (nCHRBanks == 0) { // treat as ram
            mapped_addr = addr;
            return true;
        }
    }
    return false;
}

void Mapper_002::reset() {
    nPRGBankSelectLo = 0;
    nPRGBankSelectHi = nPRGBanks - 1;
}