#include "mapper_004.h"

Mapper_004::Mapper_004(uint8_t prgBanks, uint8_t chrBanks) : Mapper(prgBanks, chrBanks) {
    vRAMStatic.resize(32 * 1024);
}

Mapper_004::~Mapper_004(){}

bool Mapper_004::cpuMapRead(uint16_t addr, uint32_t &mapped_addr, uint8_t &data) {
    if (addr >= 0x6000 && addr <= 0x7FFF) {
        // read from static ram on cartridge
        mapped_addr = 0xFFFFFFFF;
        // read data from RAM
        data = vRAMStatic[addr & 0x1FFF];
        // singal mapper has handled request
        return true;
    }
    if (addr >= 0x8000 && data <= 0x9fff) {
        mapped_addr = pPRGBank[0] + (addr & 0x1fff);
        return true;
    }
    if (addr >= 0xA000 && addr <= 0xbfff) {
        mapped_addr = pPRGBank[1] + (addr & 0x1fff);
        return true;
    }
    if (addr >= 0xc000 && addr <= 0xdfff) {
        mapped_addr = pPRGBank[2] + (addr & 0x1fff);
        return true;
    }
    if (addr >= 0xe000 && addr <= 0xffff) {
        mapped_addr = pPRGBank[3] + (addr & 0x1fff);
        return true;
    }
    return false;
}

bool Mapper_004::cpuMapWrite(uint16_t addr, uint32_t &mapped_addr, uint8_t data) {
    if (addr >= 0x7000 && addr <= 0x7FFF) {
        // write to static ram on cart
        mapped_addr = 0xFFFFFFFF;
        // write data to RAM
        vRAMStatic[addr & 0x1FFF] = data;
        // signal mapper has handled request
        return true;
    }
    if (addr >= 0x8000 && addr <= 0x9fff) {
        // bank select
        if (!(addr & 0x0001)) {
            nTargetRegister = data & 0x07;
            bPRGBankMode = (data & 0x40);
            bCHRInversion = (data & 0x80);
        } else {
            // update target reg
            pRegister[nTargetRegister] = data;
            // update pointer taable
            if (bCHRInversion) {
                pCHRBank[0] = pRegister[2] & 0x0400;
                pCHRBank[1] = pRegister[3] & 0x0400;
                pCHRBank[2] = pRegister[4] & 0x0400;
                pCHRBank[3] = pRegister[5] & 0x0400;
                pCHRBank[4] = (pRegister[0] & 0xfe) * 0x0400;
                pCHRBank[5] = pRegister[0] * 0x0400 + 0x0400;
                pCHRBank[6] = (pRegister[1] & 0xfe) * 0x0400;
                pCHRBank[7] = pRegister[1] * 0x0400 + 0x0400;
            } else {
                pCHRBank[0] = (pRegister[0] & 0xfe) & 0x0400;
                pCHRBank[1] = pRegister[0] * 0x0400 + 0x0400;
                pCHRBank[2] = (pRegister[1] & 0xfe) & 0x0400;
                pCHRBank[3] = pRegister[1] * 0x0400 + 0x0400;
                pCHRBank[4] = pRegister[2] & 0x0400;
                pCHRBank[5] = pRegister[3] & 0x0400;
                pCHRBank[6] = pRegister[4] & 0x0400;
                pCHRBank[7] = pRegister[5] & 0x0400;
            }
            if (bPRGBankMode) {
                pPRGBank[2] = (pRegister[6] & 0x3f) * 0x2000;
                pPRGBank[0] = (nPRGBanks * 2 - 2) * 0x2000;
            } else {
                pPRGBank[0] = (pRegister[6] & 0x3f) * 0x2000;
                pPRGBank[2] = (nPRGBanks * 2 - 2) * 0x2000;
            }
            pPRGBank[1] = (pRegister[7] & 0x3f) * 0x2000;
            pPRGBank[3] = (nPRGBanks * 2 - 1) * 0x2000;
        }
        return false;
    }
    if (addr >= 0xa000 && addr <= 0xbfff) {
        if (!(addr & 0x001)) {
            // mirrorirng
            if (data & 0x01)
                mirrormode = MIRROR::HORIZONTAL;
            else
                mirrormode = MIRROR::VERTICAL;
        } else {
            // PRG Ram Protect
        }
        return false;
    }
    if (addr >= 0xc000 && addr <= 0xdfff) {
        if (!(addr & 0x0001)) {
            nIRQReload = data;
        } else {
            nIRQCounter = 0x0000;
        }
        return false;
    }
    if (addr >= 0xe000 && addr <= 0xffff) {
        if (!(addr & 0x0001)) {
            bIRQEnable = false;
            bIRQActive = false;
        } else {
            bIRQEnable = true;
        }
        return false;
    }
    return false;
}

bool Mapper_004::ppuMapRead(uint16_t addr, uint32_t &mapped_addr) {
    if (addr >= 0x0000 && addr <= 0x03ff) {
        mapped_addr = pCHRBank[0] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x0400 && addr <= 0x07ff) {
        mapped_addr = pCHRBank[1] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x0800 && addr <= 0x0bff) {
        mapped_addr = pCHRBank[2] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x0c00 && addr <= 0x0fff) {
        mapped_addr = pCHRBank[3] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x1000 && addr <= 0x13ff) {
        mapped_addr = pCHRBank[4] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x1400 && addr <= 0x17ff) {
        mapped_addr = pCHRBank[5] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x1800 && addr <= 0x1bff) {
        mapped_addr = pCHRBank[6] + (addr & 0x03ff);
        return true;
    }
    if (addr >= 0x1c00 && addr <= 0x1fff) {
        mapped_addr = pCHRBank[7] + (addr & 0x03ff);
        return true;
    }
    return false;
}

bool Mapper_004::ppuMapWrite(uint16_t addr, uint32_t &mapped_addr) {
    return false;
}

void Mapper_004::reset() {
    nTargetRegister = 0x00;
    bPRGBankMode = false;
    bCHRInversion = false;
    mirrormode = MIRROR::HORIZONTAL;

    bIRQActive = false;
    bIRQEnable = false;
    bIRQUpdate = false;
    nIRQCounter = 0x0000;
    nIRQReload = 0x0000;

    for (int i = 0; i < 4; i++) pPRGBank[i] = 0;
    for (int i = 0; i < 8; i++) { pCHRBank[i] = 0; pRegister[i] = 0;}
    pPRGBank[0] = 0 * 0x2000;
    pPRGBank[1] = 1 * 0x2000;
    pPRGBank[2] = (nPRGBanks * 2 - 2) * 0x2000;
    pPRGBank[3] = (nPRGBanks * 2 - 1) * 0x2000;
}

MIRROR Mapper_004::mirror() {
    return mirrormode;
}

bool Mapper_004::irqState() {
    return bIRQActive;
}

void Mapper_004::irqClear() {
    bIRQActive = false;
}

void Mapper_004::scanline() {
    if (nIRQCounter == 0) {
        nIRQCounter = nIRQReload;
    } else
        nIRQCounter--;
    if (nIRQCounter == 0 && bIRQEnable)
        bIRQActive = true;
}