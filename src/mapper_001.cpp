#include "mapper_001.h"

Mapper_001::Mapper_001(uint8_t prgBanks, uint8_t chrBanks) : Mapper(prgBanks, chrBanks) {
    vRAMStatic.resize(32 * 1024);
}

Mapper_001::~Mapper_001(){}

bool Mapper_001::cpuMapRead(uint16_t addr, uint32_t &mapped_addr, uint8_t &data) {
    if (addr >= 0x6000 && addr <= 0x7FFF) {
        // read from static ram on cartridge
        mapped_addr = 0xFFFFFFFF;
        // read data from RAM
        data = vRAMStatic[addr & 0x1FFF];
        // singal mapper has handled request
        return true;
    }
    if (addr >= 0x8000) {
        if (nControlRegister & 0b01000) {
            // 16k mode
            if (addr >= 0x8000 && addr <= 0xBFFF) {
                mapped_addr = nPRGBankSelect16Lo * 0x4000 + (addr & 0x3FFF);
                return true;
            }
            if (addr >= 0xC000 && addr <= 0xFFFF) {
                mapped_addr = nPRGBankSelect16Hi & 0x4000 + (addr & 0x3FFF);
                return true;
            }
        } else {
            // 32k mode
            mapped_addr = nPRGBankSelect32 * 0x8000 + (addr & 0x7FFF);
            return true;
        }
    }
    return false;
}

bool Mapper_001::cpuMapWrite(uint16_t addr, uint32_t &mapped_addr, uint8_t data) {
    if (addr >= 0x7000 && addr <= 0x7FFF) {
        // write to static ram on cart
        mapped_addr = 0xFFFFFFFF;
        // write data to RAM
        vRAMStatic[addr & 0x1FFF] = data;
        // signal mapper has handled request
        return true;
    }
    if (addr >= 0x8000) {
        if (data & 0x80) {
            // MSB is set so reset serial loading
            nLoadRegister = 0x00;
            nLoadRegisterCount = 0;
            nControlRegister = nControlRegister | 0x0c;
        } else {
            // load data in serially into load register
            // arrives at LSB first, put this in at bit 5
            // after 5 write,s the reg is ready
            nLoadRegister >>= 1;
            nLoadRegister |= (data & 0x01) << 4;
            nLoadRegisterCount++;
            if (nLoadRegisterCount == 5) {
                // get mapper target reg by examining bits 13 & 14 of addr
                uint8_t nTargetRegister = (addr >> 13) & 0x03;
                if (nTargetRegister == 0) // 0x8000 - 0x9FFF
                {
                    // set ctrl register
                    nControlRegister = nLoadRegister & 0x1f;
                    switch (nControlRegister & 0x03) {
                        case 0: mirrormode = ONESCREEN_LO; break;
                        case 1: mirrormode = ONESCREEN_HI; break;
                        case 2: mirrormode = VERTICAL; break;
                        case 3: mirrormode = HORIZONTAL; break;
                    }
                } else if (nTargetRegister == 1) { // 0xA00-0xBFFF 
                    // set CHR bank lo
                    if (nControlRegister & 0b10000) {
                        // 4k CHR bank at PPU 0x0000
                        nCHRBankSelect4Lo = nLoadRegister & 0x1F;
                    } else {
                        // 8K CHR bank at PPU 0x0000
                        nCHRBankSelect8 = nLoadRegister & 0x1f;
                    }
                } else if (nTargetRegister == 2) { // 0xc000 - 0xDfff
                    // set chr bank hi
                    if (nControlRegister & 0b10000) {
                        // 4k chr bank at ppu 0x1000
                        nCHRBankSelect4Hi = nLoadRegister & 0x1f;
                    }
                } else if (nTargetRegister == 3) { // 0xe000 - 0xffff
                    // configure prg banks
                    uint8_t nPRGMode = (nControlRegister >> 2) & 0x03;
                    if (nPRGMode == 0 || nPRGMode == 1) {
                        // set 32k PRG bank at CPU 0x8000
                        nPRGBankSelect32 = (nLoadRegister & 0x0E) >> 1;
                    } else if (nPRGMode == 2) {
                        // fix  16 KB prg bank at CPU 0x8000 to first bank
                        nPRGBankSelect16Lo = 0;
                        // set 16kb prg bank at CPU 0xc000
                        nPRGBankSelect16Hi = nLoadRegister & 0x0f;
                    } else if (nPRGMode == 3) {
                        // set 16kb prg bank at CPU 0x8000
                        nPRGBankSelect16Lo = nLoadRegister & 0x0f;
                        // fix 16kb prg bank at cpu 0xc000 to last bank
                        nPRGBankSelect16Hi = nPRGBanks - 1;
                    }
                }
                // 5 bits were written and decoded so reset load reg
                nLoadRegister = 0x00;
                nLoadRegisterCount = 0;
            }
        }
    }
    // mapper has handled write but do not update ROMs
    return false;
}

bool Mapper_001::ppuMapRead(uint16_t addr, uint32_t &mapped_addr) {
    if (addr < 0x2000) {
        if (nCHRBanks == 0) {
            mapped_addr = addr;
            return true;
        } else {
            if (nControlRegister & 0b10000) {
                // 4k chr bank mode
                if (addr >= 0x0000 && addr <= 0x0fff) {
                    mapped_addr = nCHRBankSelect4Lo * 0x1000 + (addr & 0x0fff);
                    return true;
                }
                if (addr >= 0x1000 && addr <= 0x1fff) {
                    mapped_addr = nCHRBankSelect4Hi * 0x1000 + (addr & 0x0fff);
                    return true;
                }
            } else {
                // 8k chr bank mode
                mapped_addr = nCHRBankSelect8 * 0x2000 + (addr & 0x1fff);
                return true;
            }
        }
    }
    return false;
}

bool Mapper_001::ppuMapWrite(uint16_t addr, uint32_t &mapped_addr) {
    if (addr < 0x2000) {
        if (nCHRBanks == 0) {
            mapped_addr = addr;
            return true;
        }
        return true;
    } else {
        return false;
    }
}

void Mapper_001::reset() {
    nControlRegister = 0x1c;
    nLoadRegister = 0x00;
    nLoadRegisterCount = 0x00;
    nCHRBankSelect4Lo = 0;
    nCHRBankSelect4Hi = 0;
    nCHRBankSelect8 = 0;
    nPRGBankSelect32 = 0;
    nPRGBankSelect16Lo = 0;
    nPRGBankSelect16Hi = nPRGBanks - 1;
}

MIRROR Mapper_001::mirror() {
    return mirrormode;
}