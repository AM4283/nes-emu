#pragma once
#include <cstdint>

enum MIRROR
{
	HARDWARE,
	HORIZONTAL,
	VERTICAL,
	ONESCREEN_LO,
	ONESCREEN_HI,
};

class Mapper {
    public:
        Mapper(uint8_t prgBanks, uint8_t chrBanks);
        ~Mapper();

        // transform CPU bus addr into PRG rom offset
        virtual bool cpuMapRead(uint16_t addr, uint32_t &mapped_addr, uint8_t &data) = 0;
        virtual bool cpuMapWrite(uint16_t addr, uint32_t &mapped_addr, uint8_t data = 0) = 0;
        // transform PPU bus addr into chr rom offset
        virtual bool ppuMapRead(uint16_t addr, uint32_t &mapped_addr) = 0;
        virtual bool ppuMapWrite(uint16_t addr, uint32_t &mapped_addr) = 0;

        virtual void reset() = 0;
        // get mirror mode if mapper isi n control;
        virtual MIRROR mirror();
        // irq interface
        virtual bool irqState();
        virtual void irqClear();

        virtual void scanline();
    protected:
        // shared locally as mappers require this info
        uint8_t nPRGBanks = 0;
        uint8_t nCHRBanks = 0;
};