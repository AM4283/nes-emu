#include "bus.h"

Bus::Bus() {
    cpu.ConnectBus(this);
    // Clear ram contents
    // for (auto &i : ram) i = 0x00;
}
Bus::~Bus(){}

// void Bus::write(uint16_t addr, uint8_t data) {
//     if (addr >= 0x0000 && addr <= 0xFFFF) ram[addr] = data;
// }
// uint8_t Bus::read(uint16_t addr, bool bReadOnly) {
//     if (addr >= 0x0000 && addr<= 0xFFFF)
//         return ram[addr];

//     return 0x00;
// }

void Bus::setSampleFrequency(uint32_t sample_rate) {
    dAudioTimePerSystemSample = 1.0 / (double) sample_rate;
    dAudioTimePerNESClock = 1.0 / 5369138.0; // PPU clock frequency
}

void Bus::cpuWrite(uint16_t addr, uint8_t data) {
    if (cart->cpuWrite(addr, data))  {
        // cart sees all and has facility to stop bus transaction if it needs
        // allows cart to map any addr to some other data including other physical devices
    }
    else if (addr >= 0x0000 && addr <= 0x1FFF) {
        // Sys ram addr range, covers 8KB tho only 2KB available
        // 2KB is "mirrored" thru this addr range
        // use AND to mask bottom 11 bits is same as addr % 2048
        cpuRAM[addr & 0x07FF] = data;
    } else if (addr >= 0x2000 && addr <= 0x3FFF) {
        // ppu addr range
        // ppu has 8 primary registers repeated thru htis range
        // use AND to mask bottom 3 bits, equivalent to addr % 8
        ppu.cpuWrite(addr & 0x0007, data);
    } else if ((addr >= 0x4000 && addr <= 0x4013) || addr == 0x4015 || addr == 0x4017) { // APU addr range
        apu.cpuWrite(addr, data);
    } else if (addr == 0x4014) {
        // Write to this addr indicates DMA transfer
        dma_page = data;
        dma_addr = 0x00;
        dma_transfer = true;
    } else if (addr >= 0x4016 && addr <= 0x4017) {
        // Controller addr (2 addr)
        controller_state[addr & 0x0001] = controller[addr & 0x0001];
    }
}
uint8_t Bus::cpuRead(uint16_t addr, bool bReadOnly) {
    uint8_t data = 0x00;
    if (cart->cpuRead(addr, data)) {
        // cart addr range
    } else if (addr >= 0x0000 && addr <= 0x1FFF) {
        // sys addr range mirrored every 20248
        data = cpuRAM[addr & 0x07FF];
    } else if(addr >= 0x2000 && addr <= 0x3FFF) {
        // PPU addr range mirrored every 8
        data = ppu.cpuRead(addr & 0x0007, bReadOnly);
    } else if (addr == 0x4015) {
        // APU read status
        data = apu.cpuRead(addr);
    } else if (addr >= 0x4016 && addr <= 0x4017) {
        // read out controller status word
        data = (controller_state[addr & 0x0001] & 0x80) > 0;
        controller_state[addr & 0x0001] <<= 1;
    }
    return data;
}

void Bus::insertCartridge(const std::shared_ptr<Cartridge>& cartridge) {
    // connect cart to main bus and ppu bus
    this->cart = cartridge;
    ppu.connectCartridge(cartridge);
}

void Bus::reset() { 
    cpu.reset();
    ppu.reset();
    nSystemClockCounter = 0;
    dma_page = 0x00;
    dma_addr = 0x00;
    dma_data = 0x00;
    dma_dummy = true;
    dma_transfer = false;
}

bool Bus::clock() {
    // running freq is controlled by whatever calls this function
    // divide clocks as necessary and call peripheral devices clock() at correct time
    // fastest meaningful clock freq is == PPU clock, so PPU is clocked each time this is run
    ppu.clock();
    // clock the APU
    apu.clock();
    // CPU runs 3x slower than PPU so we call its clock() every 3 times this func is called
    // use global counter to keep track of calls
    if (nSystemClockCounter % 3 == 0) {
        // check if sys is performing a DMA trasnfer from CPU->OAM on PPU
        if (dma_transfer) {
            // Wait till next even CPU clock cycle
            if (dma_dummy) {
                if (nSystemClockCounter % 2 == 1) {
                    dma_dummy = false; // allow dma to start
                }
            } else {
                // DMA takes place
                if (nSystemClockCounter % 2 == 0) {
                    // on even cycle read from CPU bus
                    dma_data = cpuRead(dma_page << 8 | dma_addr);
                } else {
                    // on odd cycles write to PPU OAM
                    ppu.pOAM[dma_addr] = dma_data;
                    // increment lo bye of addr
                    dma_addr++;
                    // if this wraps around, 256 bytes have been written so end DMA transfer
                    if (dma_addr == 0x00) {
                        dma_transfer = false;
                        dma_dummy = true;
                    }
                }
            }
        } else {
            // No DMA happening, CPU runs as normal
            cpu.clock();
        }
    }
    // Sync with audio
    bool bAudioSampleReady = false;
    dAudioTime += dAudioTimePerNESClock;
    if (dAudioTime >= dAudioTimePerSystemSample) {
        dAudioTime -= dAudioTimePerSystemSample;
        dAudioSample = apu.getOutputSample();
        bAudioSampleReady = true;
    }
    // PPU is capable of emitting an interrupt ot indicate vblank period has been entered
    // if this is the case we need to sned htis irq to the CPU
    if (ppu.nmi) {
        ppu.nmi = false;
        cpu.nmi();
    }
    // check if cartridge is requesting irq
    if (cart->getMapper()->irqState()) {
        cart->getMapper()->irqClear();
        cpu.irq();
    }
    nSystemClockCounter++;
    return bAudioSampleReady;
}