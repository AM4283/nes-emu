#pragma once
#include "cpu.h"
#include "ppu.h"
#include "apu.h"
#include "cartridge.h"

#include <array>

class Bus {
public:
    Bus();
    ~Bus();

    CPU cpu;
    // fake ram
    // std::array<uint8_t, 64*1024> ram;
    PPU ppu;
    // 2A03 apu
    APU apu;
    // cartridge or game pak
    std::shared_ptr<Cartridge> cart;
    // 2kb of ram
    uint8_t cpuRAM[2048];
    // Controllers
    uint8_t controller[2];

    // Sync with system audio
    void setSampleFrequency(uint32_t sample_rate);
    double dAudioSample = 0.0;

    // bus read / write
    // uint8_t read(uint16_t addr, bool bReadOnly = false);
    // void write(uint16_t addr, uint8_t data);

    uint8_t cpuRead(uint16_t addr, bool bReadOnly = false);
    void cpuWrite(uint16_t addr, uint8_t data);
private:
    double dAudioTime = 0.0;
    double dAudioGlobalTime = 0.0;
    double dAudioTimePerNESClock = 0.0;
    double dAudioTimePerSystemSample = 0.0f;
    // count of how many clocks have passed
    uint32_t nSystemClockCounter = 0;
    //internal cache of controller state
    uint8_t controller_state[2];
private:
    // simple form of DMA used to switfly transfer data from CPU bus mem
    // into OAM memory
    // Program prepares a page of mem w srite info req for next frame
    // then initializes DMA transfer. Suspends CPU momentarily
    // while PPU gets sent data at PPU clock speeds
    // dma_page and dma_addr form a 16 bit addr in CPU bus addr space
    uint8_t dma_page = 0x00;
    uint8_t dma_addr = 0x00;
    uint8_t dma_data = 0x00;
    // DMA transfers need to be timed accurately
    // 512 cycle sto R/W 256 bytes to OAM, a read followed by a write
    // However, CPU needs to be on even clock cycle so a "dummy cycle" of idle may be required
    bool dma_dummy = true;
    // flag to indicate DMA is happening
    bool dma_transfer = false;
public: // system interface
    //connect cartridge to internal buses
    void insertCartridge(const std::shared_ptr<Cartridge>& cartridge);
    // reset the system
    void reset();
    // clock the system one tick
    bool clock();
};