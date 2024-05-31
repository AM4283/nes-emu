#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#ifdef LOGMODE
#include <stdio.h>
#endif
// NES 2A03
// NTSC chip consists of a 6502 (without decimal mode)
// Audio, joypad, and DMA functionality

// Registers

// uint16_t* SQ1_VOL = (uint16_t*) 0x4000; // Duty cycle and volume
// uint16_t* SQ1_SWEEP = (uint16_t*) 0x4001; // Sweep control register
// uint16_t* SQ1_LO = (uint16_t*) 0x4002; // Low byte period
// uint16_t* SQ1_HI = (uint16_t*) 0x4003; // high byte period/length counter value

// uint16_t* SQ2_VOL = (uint16_t*) 0x4004;
// uint16_t* SQ2_SWEEP = (uint16_t*) 0x4005;
// uint16_t* SQ2_LO = (uint16_t*) 0x4006;
// uint16_t* SQ2_HI = (uint16_t*) 0x4007;

// uint16_t* TRI_LINEAR = (uint16_t*) 0x4008; // linear counter
// uint16_t* TRI_SWEEP = (uint16_t*) 0x4009; // unused
// uint16_t* TRI_LO = (uint16_t*) 0x400A;
// uint16_t* TRI_HI = (uint16_t*) 0x400B;

// uint16_t* NOISE_VOL = (uint16_t*) 0x400C;
// uint16_t* NOISE_SWEEP = (uint16_t*) 0x400D; // unused
// uint16_t* NOISE_LO = (uint16_t*) 0x400E;
// uint16_t* NOISE_HI = (uint16_t*) 0x400F;


// uint16_t* DMC_FREQ = (uint16_t*) 0x4010; // irq flag, loop flag and freq
// uint16_t* DMC_RAW = (uint16_t*) 0x4011; // 7-bit DAC
// uint16_t* DMC_START = (uint16_t*) 0x4012; // start addr = $C000 + $40+$xx
// uint16_t* DMC_LEN = (uint16_t*) 0x4013; // sample len = $10*$xx + 1 bytes (128*$xx + 8 samples)

// uint16_t* OAMDMA = (uint16_t*) 0x4014; // Copy 256 bytes from $xx00-$xxFF into OAM via $OAMDATA ($2004)

// uint16_t* SND_CHN = (uint16_t*) 0x4015; // sound challens enable / sound channel and irq status

// uint16_t* JOY1 = (uint16_t*) 0x4016; // Joystick strobe / Joystick 1 data
// uint16_t* JOY2 = (uint16_t*) 0x4017; // Frame counter control / Joystick 2 data

// uint16_t* APU_TEST1 = (uint16_t*) 0x4018; // APU test functonality
// uint16_t* APU_TEST2 = (uint16_t*) 0x4019;
// uint16_t* APU_TEST3 = (uint16_t*) 0x401A;

// uint16_t* IRQ_TIMER1 = (uint16_t*) 0x401C; // Unfinshed irq timer functionality
// uint16_t* IRQ_TIMER2 = (uint16_t*) 0x401D;
// uint16_t* IRQ_TIMER3 = (uint16_t*) 0x401E;
// uint16_t* IRQ_TIMER4 = (uint16_t*) 0x401F;

class Bus;

class CPU {
public:
    CPU();
    ~CPU();
    // core registers
    uint8_t acc = 0x00; // accumulator register
    uint8_t x = 0x00; // x reg
    uint8_t y = 0x00; // y reg
    uint8_t stkp = 0x00; // stack pointer
    uint16_t pc = 0x0000; // program counter
    uint8_t status = 0x00; // status reg

    void reset(); // reset interrupt - force cpu to known state
    void irq(); // interrupt request - exec instruction
    void nmi(); // non-maskable irq - as above but cannot be disabled
    void clock(); // perform one clock cycles worth of update

    bool complete(); // indicates current instruction has completed by ret true
    // this is a utility to enable step-by-step exec manually clocking every cycle

    void ConnectBus(Bus *n) { 
        bus = n; 
    } // link CPU to comm bus

    std::map<uint16_t, std::string> disassemble(uint16_t nStart, uint16_t nStop);

    enum FLAGS {
        C = (1 << 0), // carry bit
        Z = (1 << 1), // zero bit
        I = (1 << 2), // disable interrupts
        D = (1 << 3), // dec mode
        B = (1 << 4), // break
        U = (1 << 5), // unused
        V = (1 << 6), // overflow
        N = (1 << 7), // negative
    };

private:
    // set/get status reg
    uint8_t getFlag(FLAGS f);
    void setFlag(FLAGS f, bool v);

    // assisstive varibles to facilitate emulation
    uint8_t fetched = 0x00; // represents the working input val to the ALU
    uint16_t temp = 0x0000; // a convenience var used everywhere
    uint16_t addr_abs = 0x0000; // all used mem addr end up in here
    uint16_t addr_rel = 0x00; // abs address following a branch
    uint8_t opcode = 0x00; // instrution byte
    uint8_t cycles = 0; // how many cyles instruction has remaining
    uint32_t clock_count = 0; // global accumulation of num of clocks

    // link to comm bus
    Bus *bus = nullptr;
    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t data);
    // read addr can come from mem addr or is available
    // as part of the instruction, fetch() decides
    // depending on addr mode of instructon byte
    uint8_t fetch();

    // opcode translation table, 256 diferent instructions
    // stored in numerical order, no decoding required
    // Each entry hods: Pneumonic: text rep of instruc (for disassembly)
    //  Opcode function: pointer to implementation of opcode
    //  Opcode addr mode: ufnc pointer to impl of addr mechanism used by instruction
    //  Cycle count: int representing base nu mof clock cycles
    //      required to perform instruction

    struct INSTRUCTION {
        std::string name;
        uint8_t (CPU::*operate)(void) = nullptr;
        uint8_t (CPU::*addrmode)(void) = nullptr;
        uint8_t cycles = 0;
    };
    std::vector<INSTRUCTION> lookup;

    // Addressing modes
    // 6502 has Variety of diff addr modes to acces data in memory
    // Some are indirect others are direct (pointers)
    // Each opcode contains info about which addr mode should be employed on read/write
    // Addr mode changes num of byte sthat make up the full instruction =>
    //      addressing has to be implemented b4 exec instruction
    //      Make sure pc is at correct location, instruc has the right addresses
    //      Nm of cloc cycles the instruc requires is calculated
    // These functions may ajust number of clock cycles required depening upon
    // where and how mem is accessed, return required adjustment

    uint8_t IMP(); // implicit
    uint8_t ZP0(); // zero page addr
    uint8_t ZPX(); // zero page index reg X
    uint8_t ZPY(); // zero page index reg Y
    uint8_t ABS(); // absolute addr
    uint8_t ABX(); // absolute X
    uint8_t ABY(); // absolute Y
    uint8_t IND(); // indiret addr
    uint8_t IZX(); // indirect index X
    uint8_t IZY(); // indirect index Y
    uint8_t REL(); // relative 
    uint8_t IMM(); // immediate mode

    // 56 "legitamate" opcodes provided by 6502
    // each opcode defined by 1 byte => 256 possible codes
    // Alphabetical order for ease of finding

    uint8_t ADC();
    uint8_t AND();
    uint8_t ASL();
    uint8_t BCC();
    uint8_t BCS();
    uint8_t BEQ();
    uint8_t BIT();
    uint8_t BMI();
    uint8_t BNE();
    uint8_t BPL();
    uint8_t BRK();
    uint8_t BVC();
    uint8_t BVS();
    uint8_t CLC();
    uint8_t CLD();
    uint8_t CLI();
    uint8_t CLV();
    uint8_t CMP();
    uint8_t CPX();
    uint8_t CPY();
    uint8_t DEC();
    uint8_t DEX();
    uint8_t DEY();
    uint8_t EOR();
    uint8_t INC();
    uint8_t INX();
    uint8_t INY();
    uint8_t JMP();
    uint8_t JSR();
    uint8_t LDA();
    uint8_t LDX();
    uint8_t LDY();
    uint8_t LSR();
    uint8_t NOP();
    uint8_t ORA();
    uint8_t PHA();
    uint8_t PHP();
    uint8_t PLA();
    uint8_t PLP();
    uint8_t ROL();
    uint8_t ROR();
    uint8_t RTI();
    uint8_t RTS();
    uint8_t SBC();
    uint8_t SEC();
    uint8_t SED();
    uint8_t SEI();
    uint8_t STA();
    uint8_t STX();
    uint8_t STY();
    uint8_t TAX();
    uint8_t TAY();
    uint8_t TSX();
    uint8_t TXA();
    uint8_t TXS();
    uint8_t TYA();
    // captures all unofficial opcodes, identical to a NOP
    uint8_t XXX();
#ifdef LOGMODE
private:
    FILE* logfile = nullptr;
#endif
};