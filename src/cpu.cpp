#include "cpu.h"
#include "bus.h"
//  https://swh.princeton.edu/~mae412/HANDOUTS/Datasheets/6502.pdf
// https://www.nesdev.org/obelisk-6502-guide/reference.html

CPU::CPU() {
    // assembles verbose addr translation table
    // 16*16 entries gives 256 instructions
    // bototm 4 bits of instruction choose the col, top 4 bits choose hte rows

    using a = CPU;
    lookup =
    {
		{ "BRK", &a::BRK, &a::IMM, 7 },{ "ORA", &a::ORA, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::ZP0, 3 },{ "ASL", &a::ASL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHP", &a::PHP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::IMM, 2 },{ "ASL", &a::ASL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABS, 4 },{ "ASL", &a::ASL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BPL", &a::BPL, &a::REL, 2 },{ "ORA", &a::ORA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ZPX, 4 },{ "ASL", &a::ASL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLC", &a::CLC, &a::IMP, 2 },{ "ORA", &a::ORA, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABX, 4 },{ "ASL", &a::ASL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "JSR", &a::JSR, &a::ABS, 6 },{ "AND", &a::AND, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "BIT", &a::BIT, &a::ZP0, 3 },{ "AND", &a::AND, &a::ZP0, 3 },{ "ROL", &a::ROL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLP", &a::PLP, &a::IMP, 4 },{ "AND", &a::AND, &a::IMM, 2 },{ "ROL", &a::ROL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "BIT", &a::BIT, &a::ABS, 4 },{ "AND", &a::AND, &a::ABS, 4 },{ "ROL", &a::ROL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BMI", &a::BMI, &a::REL, 2 },{ "AND", &a::AND, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ZPX, 4 },{ "ROL", &a::ROL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEC", &a::SEC, &a::IMP, 2 },{ "AND", &a::AND, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ABX, 4 },{ "ROL", &a::ROL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "RTI", &a::RTI, &a::IMP, 6 },{ "EOR", &a::EOR, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "EOR", &a::EOR, &a::ZP0, 3 },{ "LSR", &a::LSR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHA", &a::PHA, &a::IMP, 3 },{ "EOR", &a::EOR, &a::IMM, 2 },{ "LSR", &a::LSR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::ABS, 3 },{ "EOR", &a::EOR, &a::ABS, 4 },{ "LSR", &a::LSR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BVC", &a::BVC, &a::REL, 2 },{ "EOR", &a::EOR, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ZPX, 4 },{ "LSR", &a::LSR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLI", &a::CLI, &a::IMP, 2 },{ "EOR", &a::EOR, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ABX, 4 },{ "LSR", &a::LSR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "RTS", &a::RTS, &a::IMP, 6 },{ "ADC", &a::ADC, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ADC", &a::ADC, &a::ZP0, 3 },{ "ROR", &a::ROR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLA", &a::PLA, &a::IMP, 4 },{ "ADC", &a::ADC, &a::IMM, 2 },{ "ROR", &a::ROR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::IND, 5 },{ "ADC", &a::ADC, &a::ABS, 4 },{ "ROR", &a::ROR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BVS", &a::BVS, &a::REL, 2 },{ "ADC", &a::ADC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ZPX, 4 },{ "ROR", &a::ROR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEI", &a::SEI, &a::IMP, 2 },{ "ADC", &a::ADC, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ABX, 4 },{ "ROR", &a::ROR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "???", &a::NOP, &a::IMP, 2 },{ "STA", &a::STA, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZP0, 3 },{ "STA", &a::STA, &a::ZP0, 3 },{ "STX", &a::STX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "DEY", &a::DEY, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 2 },{ "TXA", &a::TXA, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "STY", &a::STY, &a::ABS, 4 },{ "STA", &a::STA, &a::ABS, 4 },{ "STX", &a::STX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
		{ "BCC", &a::BCC, &a::REL, 2 },{ "STA", &a::STA, &a::IZY, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZPX, 4 },{ "STA", &a::STA, &a::ZPX, 4 },{ "STX", &a::STX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "TYA", &a::TYA, &a::IMP, 2 },{ "STA", &a::STA, &a::ABY, 5 },{ "TXS", &a::TXS, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::NOP, &a::IMP, 5 },{ "STA", &a::STA, &a::ABX, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::XXX, &a::IMP, 5 },
		{ "LDY", &a::LDY, &a::IMM, 2 },{ "LDA", &a::LDA, &a::IZX, 6 },{ "LDX", &a::LDX, &a::IMM, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "LDY", &a::LDY, &a::ZP0, 3 },{ "LDA", &a::LDA, &a::ZP0, 3 },{ "LDX", &a::LDX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "TAY", &a::TAY, &a::IMP, 2 },{ "LDA", &a::LDA, &a::IMM, 2 },{ "TAX", &a::TAX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "LDY", &a::LDY, &a::ABS, 4 },{ "LDA", &a::LDA, &a::ABS, 4 },{ "LDX", &a::LDX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
		{ "BCS", &a::BCS, &a::REL, 2 },{ "LDA", &a::LDA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "LDY", &a::LDY, &a::ZPX, 4 },{ "LDA", &a::LDA, &a::ZPX, 4 },{ "LDX", &a::LDX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "CLV", &a::CLV, &a::IMP, 2 },{ "LDA", &a::LDA, &a::ABY, 4 },{ "TSX", &a::TSX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 4 },{ "LDY", &a::LDY, &a::ABX, 4 },{ "LDA", &a::LDA, &a::ABX, 4 },{ "LDX", &a::LDX, &a::ABY, 4 },{ "???", &a::XXX, &a::IMP, 4 },
		{ "CPY", &a::CPY, &a::IMM, 2 },{ "CMP", &a::CMP, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPY", &a::CPY, &a::ZP0, 3 },{ "CMP", &a::CMP, &a::ZP0, 3 },{ "DEC", &a::DEC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INY", &a::INY, &a::IMP, 2 },{ "CMP", &a::CMP, &a::IMM, 2 },{ "DEX", &a::DEX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "CPY", &a::CPY, &a::ABS, 4 },{ "CMP", &a::CMP, &a::ABS, 4 },{ "DEC", &a::DEC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BNE", &a::BNE, &a::REL, 2 },{ "CMP", &a::CMP, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ZPX, 4 },{ "DEC", &a::DEC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLD", &a::CLD, &a::IMP, 2 },{ "CMP", &a::CMP, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ABX, 4 },{ "DEC", &a::DEC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "CPX", &a::CPX, &a::IMM, 2 },{ "SBC", &a::SBC, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPX", &a::CPX, &a::ZP0, 3 },{ "SBC", &a::SBC, &a::ZP0, 3 },{ "INC", &a::INC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INX", &a::INX, &a::IMP, 2 },{ "SBC", &a::SBC, &a::IMM, 2 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::SBC, &a::IMP, 2 },{ "CPX", &a::CPX, &a::ABS, 4 },{ "SBC", &a::SBC, &a::ABS, 4 },{ "INC", &a::INC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BEQ", &a::BEQ, &a::REL, 2 },{ "SBC", &a::SBC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ZPX, 4 },{ "INC", &a::INC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SED", &a::SED, &a::IMP, 2 },{ "SBC", &a::SBC, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ABX, 4 },{ "INC", &a::INC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
	};

}
CPU::~CPU() {}

// Bus connectivity
// read 8 bit byte from bus, located at specified 16 bit addr
uint8_t CPU::read(uint16_t addr) {
    return bus->cpuRead(addr, false);
}
// write a byte to the bus at specified addr
void CPU::write(uint16_t addr, uint8_t data) {
    bus->cpuWrite(addr, data);
}

// forces CPU to a known state, hard wirerd inside CPU
// registers set to 0x00 status reg is cleared except for unused bit
// abs addr is read from locaiton 0xFFFC which contans a second addr
// PC is set to -- allows prorammer to jump to a known and programmable
// location in memory to start exeucting from. Prorammer sets the value
// at locaiton 0xFFFC at compile time
void CPU::reset() {
    // get addr to set pc to 
    addr_abs = 0xFFFC;
    uint16_t lo = read(addr_abs+0);
    uint16_t hi = read(addr_abs+1);
    // set pc
    pc = (hi << 8 ) | lo;

    // reset internal reg
    acc = 0;
    x = 0;
    y = 0;
    stkp = 0xFD;
    status = 0x00 | U;

    // clear internal helper vars
    addr_rel = 0x0000;
    addr_abs = 0x0000;
    fetched = 0x00;

    // reset takes time
    cycles = 8;
}

// interrupts only occur if disable interrupts flag is 0
// irq can happen anytime but u dont wnat them to be destructive
// to the currently running program
// the current instruction is allowed to finish (do the whole thing whne cycles = 0)
// then pc is stored to stack. When rutine that services hte interrupt has finishes
// the status reg nad pc cna be restored to how it was before
// this is done using the RTI instruction
// Once irq has happened, in a similar way to a rset, a programmable addr is read from
// hard coded location 0xFFFE which is set to the program counter

void CPU::irq() {
    if (getFlag(I) == 0) {
        // push PC to stack -- its 16 bits so it takes two pushes
        write(0x0100 + stkp, (pc>>8) & 0x00FF);
        stkp--;
        write(0x0100 + stkp, pc & 0x00FF);
        stkp--;

        // push status reg onto stack
        setFlag(B, 0);
        setFlag(U, 1);
        setFlag(I, 1);
        write(0x0100 + stkp, status);
        stkp--;

        // read new pc location from fixed addr
        addr_abs = 0xFFFE;
        uint16_t lo = read(addr_abs);
        uint16_t hi = read(addr_abs + 1);
        pc = (hi << 8) | lo;
        // irqs take time
        cycles = 7;
    }
}
// non-maskable interrupt cannot be ignored
// reads new PC addr from 0xFFFA
void CPU::nmi() {
    write(0x0100 + stkp, (pc>>8) & 0x00FF);
    stkp--;
    write(0x0100 + stkp, pc & 0x00FF);
    stkp--;

    // push status reg onto stack
    setFlag(B, 0);
    setFlag(U, 1);
    setFlag(I, 1);
    write(0x0100 + stkp, status);
    stkp--;

    // read new pc location from fixed addr
    addr_abs = 0xFFFA;
    uint16_t lo = read(addr_abs);
    uint16_t hi = read(addr_abs + 1);
    pc = (hi << 8) | lo;
    cycles = 8;
}
void CPU::clock() {
    // to remain compliant w connected evices, it is importnat that
    // emulation takes itme to exec instructions
    // impleent by simply counting down cycles req by the instruction
    // 0 = complete

    if (cycles == 0) {
        // read next instruction byte
        // 8 bit val is used to index translation table
        opcode = read(pc);
    #ifdef LOGMODE
        uint16_t log_pc = pc;
    #endif
        // set unused flag to 1
        setFlag(U, true);
        // inc pc since we read opcode byte
        pc++;
        cycles = lookup[opcode].cycles;
        // perform fetch of intermediate data using the required addr mode
        uint8_t additional_cycle1 = (this->*lookup[opcode].addrmode)();
        uint8_t additional_cycle2 = (this->*lookup[opcode].operate)();
        // addrmode and opcode may have latered number of cycles this instruc requires
        cycles += (additional_cycle1 & additional_cycle2);
        // set unusued status flag bit  to 1
        setFlag(U, true);
    #ifdef LOGMODE
        // logger dumps every cycle the entire processor state
        // can be used for debugging emulation, very slow
        if (logfile == nullptr) logfile = fopen("CPU.txt", "wt");
        if (logfile != nullptr) {
            fprintf(logfile, "%10d:%02d PC:%04X %s A:%02X X:%02X Y:%02X %s%s%s%s%s%s%s%s STKP:%02X\n",
                clock_count, 0, log_pc, "XXX", a, x, y,
                getFlag(N) ? "N" : ".", getFlag(V) ? "V" : ".", getFlag(U) ? "U" : ".", 
                getFlag(B) ? "B" : ".", getFlag(D) ? "D" : ".", getFlag(I) ? "I" : ".",
                getFlag(Z) ? "Z" : ".", getFlag(C) ? "C" : ".", stkp);
        }
    #endif
    }
    // inc global clock count, watch var for debugging
    clock_count++;
    // Dec num of cycles remaining for instruction
    cycles--;
}

// flagfunctions
uint8_t CPU::getFlag(FLAGS f) {
    return ((status & f) > 0) ? 1 : 0;
}
void CPU::setFlag(FLAGS f, bool v) {
    if (v)
        status |= f;
    else
        status &= ~f;
}

// addressing modes
// Addr between 0x0000 and 0xffff
// high byte is page, low byte is offset, 256 pages w 256 bytes
// several addressing modes can add additional clock cycle if they
// cross a page boundary. Combined w several instuctions that  enble this
// additional cock cycle. Each addr function returns a flag
// saying it has potentialso does each instruciton. If
// instruciton and addr function return 1, then an additional
// clock cyle is required


// Implied
// No additional data, simple like sets a status bit
// We target accumulator

uint8_t CPU::IMP() {
    fetched = acc;
    return 0;
}
// immediate
// expects next byte to be used as a value, so prep read addr to point to next byte
uint8_t CPU::IMM() {
    addr_abs = pc++;
    return 0;
}

// zero page
// to save program bytes, zero page addr allows you to absolutely addr
// a location in first 0xFF bytes of addr range
// only req one byte instead of 2

uint8_t CPU::ZP0(){
    addr_abs = read(pc);
    pc++;
    addr_abs &= 0x00FF;
    return 0;
}

// zero page with X offset
// contnets of x reg is added to upplied single byte addr
// iterate thru ranges in first page

uint8_t CPU::ZPX() {
    addr_abs = (read(pc) + x);
    pc++;
    addr_abs &= 0x00FF;
    return 0;
}

uint8_t CPU::ZPY() {
    addr_abs = (read(pc) + y);
    pc++;
    addr_abs &= 0x00FF;
    return 0;
}

// relative
// exclusive to branch instructions
// addr must reside within -128 to 127 of branch instruction
// cant directly branch to any addr in addr range
uint8_t CPU::REL() {
    addr_rel = read(pc);
    pc++;
    if(addr_rel & 0x80)
      addr_rel |= 0xFF00;
    return 0;
}

// absolute
// full 16 bit addr is loaded and used
uint8_t CPU::ABS() {
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;
    addr_abs = (hi << 8) | lo;
    return 0;
}

// absolue with x offst
// contnets of x reg added to supplied addr
// if resulting addr changes page an additional cycle is reqired

uint8_t CPU::ABX() {
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;
    addr_abs = (hi << 8) | lo;
    addr_abs += x;
    if ((addr_abs & 0xFF00) != (hi << 8)) {
        return 1;
    } else {
        return 0;
    }
}
uint8_t CPU::ABY() {
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;
    addr_abs = (hi << 8) | lo;
    addr_abs += y;
    if ((addr_abs & 0xFF00) != (hi << 8)) {
        return 1;
    } else {
        return 0;
    }
}

// indirect
// supplied 16 bit addr is read to get actual addr
// contains a bug in hardware => emlate bug
// if low byte of addr is 0xFF then to read high byte of actual addr
// we need to cross a page boundary
// on the chip, there is a bug that makes it so it just wraps 
// back around ot the same page yielding na invalid actual address
uint8_t CPU::IND() {
    uint16_t ptr_lo = read(pc);
    pc++;
    uint16_t ptr_hi = read(pc);
    pc++;
    uint16_t ptr = (ptr_hi << 8) | ptr_lo;
    if (ptr_lo == 0x00FF) // simulate page boundary hw bug
    {
        addr_abs = (read(ptr&0xFF00) << 8) | read(ptr);
    } else {
        addr_abs = (read(ptr+1) << 8) | read(ptr);
    }
    return 0;
}

// indirect x
// suopplied 8 bit addr is offset by x reg to index location
// in page 0x00
// actual 16 bit addr is read from this locaiton
uint8_t CPU::IZX() {
    uint16_t t = read(pc);
    pc++;
    uint16_t hi = read((uint16_t)(t + (uint16_t)x) & 0x00FF);
    uint16_t lo = read((uint16_t)(t + (uint16_t)x + 1) & 0x00FF);
    addr_abs = (hi << 8) | lo;
    return 0;
}
// indirect y
// supplie 8 bit addr indexes loc in page 0x00
// actual 16 bit addr is read and contents of y reg is added to offset it
// if offset causes a chang ein page then an additional clock cycle is req

uint8_t CPU::IZY() {
    uint16_t t = read(pc);
    pc++;
    uint16_t hi = read(t + 0x00FF);
    uint16_t lo = read((t + 1) & 0x00FF);
    addr_abs = (hi << 8) | lo;
    addr_abs += y; 
    if ((addr_abs & 0xFF00) != (hi << 8))
        return 1;
    else
        return 0;
}
// sources data used by instruction into numeric variable
// some instructions dont have to fetch data since source is implied
// (eg INX increments X reg)
// For all other modes, data resides at locaiton in addr_abs
// immediate mode exploits this since it has addr_abs = pc+1
// so it just fethces data from next byte 
// fetched is global var to CPU set by this function
uint8_t CPU::fetch() {
    if (!(lookup[opcode].addrmode == &CPU::IMP))
        fetched = read(addr_abs);
    return fetched;
}


// Instructions

// add and sub: neg numbers have MSB set pos do not (2s complement 8 bit signed)
// Overflow flag is set if reuslt has wrapped around
// Pos + Pos = Neg indicates overflow
// Neg + neg = pos indicates underflow
// Pos + neg = either cnanot overflow
// if pos + pos = pos, no overflow
// if neg + neg = neg, no underflow
// Recall: 8 bit signed is -128 to 127, unsigned is 0 to 255

// addition with carry in
uint8_t CPU::ADC() {
    // grab data to be added to acc
    fetch();
    // perform add in 16 bit to capture any carry bit which exists in 8 bit of 16-bit word
    temp = (uint16_t) acc + (uint16_t) fetched + (uint16_t)getFlag(C);
    setFlag(C, temp > 255); // set carry flag if high bit != 0
    setFlag(Z, (temp & 0x00FF) == 0); // set zero flag if result is 0
    // signed overflow flag
    setFlag(V, (~((uint16_t)acc ^ (uint16_t)fetched) & 
    ((uint16_t)acc ^ (uint16_t)temp)) & 0x0080);
    // negative flag set to MSB of result
    setFlag(N, temp & 0x80);
    // load result into acc (its 8 bit)
    acc = temp & 0x00FF;
    // potential to req additional clock cycle
    return 1;
}
// subtract with borrow in
// to make a signed pos number negative, invert the its and add 1
uint8_t CPU::SBC() {

    // grab data to be added to acc
    fetch();
    // invert bottom 8 bits with xor
    uint16_t value = ((uint16_t)fetched) ^ 0x00FF;
    // perform add in 16 bit to capture any carry bit which exists in 8 bit of 16-bit word
    temp = (uint16_t) acc + (uint16_t) value + (uint16_t)getFlag(C);
    setFlag(C, temp > 255); // set carry flag if high bit != 0
    setFlag(Z, (temp & 0x00FF) == 0); // set zero flag if result is 0
    // signed overflow flag
    setFlag(V, (temp ^ (uint16_t)acc) & (temp ^ value) & 0x0080);
    // negative flag set to MSB of result
    setFlag(N, temp & 0x0080);
    // load result into acc (its 8 bit)
    acc = temp & 0x00FF;
    // potential to req additional clock cycle
    return 1;
}

// bitwise AND
uint8_t CPU::AND() {
    fetch();
    acc = acc & fetched;
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
}
// arithmetic shift left
// A - C <- (A << 1) <- 0
uint8_t CPU::ASL() {
    fetch();
    temp = (uint16_t) fetched << 1;
    setFlag(C, (temp & 0xFF00) > 0);
    setFlag(Z, (temp & 0x00FF) == 0x00);
    setFlag(N, temp & 0x80);
    if (lookup[opcode].addrmode == &CPU::IMP)
        acc = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}
// branh if carry clear (if C==0) pc = address
uint8_t CPU::BCC() {
    if (getFlag(C) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// branch if carry set
uint8_t CPU::BCS() {
    if (getFlag(C) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// branch if equal (if Z == 1 pc = addr)
uint8_t CPU::BEQ() {
    if (getFlag(Z) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// set bits based on fetched data
uint8_t CPU::BIT() {
    fetch();
    temp = acc & fetched;
    setFlag(Z, (temp & 0x00FF) == 0x00);
    setFlag(N, fetched & (1 << 7));
    setFlag(V, fetched & (1 << 6));
    return 0;
}
// branch if negative
uint8_t CPU::BMI() {
    if (getFlag(N) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// branch if not equal

uint8_t CPU::BNE() {
    if (getFlag(Z) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// branch if positive
uint8_t CPU::BPL() {
    if (getFlag(N) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// break (interrupt in program)
uint8_t CPU::BRK() {
    pc++;
    setFlag(I, 1); // set disable interrupt flag
    write(0x0100 + stkp, (pc >> 8) & 0x00FF); // write high bits of pc to stack
    stkp--;
    write(0x0100 + stkp, pc & 0x00FF); // write low bits of pc to stack
    stkp--;

    setFlag(B, 1); // set break flag
    write(0x0100 + stkp, status); // write status
    stkp--;
    setFlag(B, 0); // unset break
    pc = (uint16_t) read(0xFFFE) | ((uint16_t)read(0xFFFF) << 8); // load interrupt vector addr from memory
    // jumps exec to interrupt service routine
    return 0;
}
// branch if overflow clear
uint8_t CPU::BVC() {
    if (getFlag(V) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// branch if overflow set
uint8_t CPU::BVS() {
    if (getFlag(V) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;
        if((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}
// clear carry flag
uint8_t CPU::CLC() {
    setFlag(C, false);
    return 0;
}
// clear decimal flag
uint8_t CPU::CLD() {
    setFlag(D, false);
    return 0;
}
// clear interrupt flag
uint8_t CPU::CLI() {
    setFlag(I, false);
    return 0;
}
// clear overflow flag
uint8_t CPU::CLV() {
    setFlag(V, false);
    return 0;
}
// compare accumulator to fetched data
uint8_t CPU::CMP() {
    fetch();
    temp = (uint16_t) acc - (uint16_t) fetched;
    setFlag(C, acc >= fetched);
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    return 1;
}
// compare x reg to fetched data
uint8_t CPU::CPX() {
    fetch();
    temp = (uint16_t) x - (uint16_t) fetched;
    setFlag(C, x >= fetched);
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    return 0;
}
// compare y reg to fetched data
uint8_t CPU::CPY() {
    fetch();
    temp = (uint16_t) y - (uint16_t) fetched;
    setFlag(C, y >= fetched);
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    return 0;
}
// dec value at mem location
uint8_t CPU::DEC() {
    fetch();
    temp = fetched - 1;
    write(addr_abs, temp & 0x00FF);
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    return 0;
}
// dec value at x reg
uint8_t CPU::DEX() {
    x--;
    setFlag(Z, x == 0x00);
    setFlag(N, x & 0x80);
    return 0;
}
// dec value at y reg
uint8_t CPU::DEY() {
    y--;
    setFlag(Z, y == 0x00);
    setFlag(N, y & 0x80);
    return 0;
}
// bitwise logic xor
uint8_t CPU::EOR() {
    fetch();
    acc = acc ^ fetched;
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
    return 1;
}
// inc value at mem location
uint8_t CPU::INC() {
    fetch();
    temp = fetched + 1;
    write(addr_abs, temp & 0x00FF);
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    return 0;
}
// inc value at x reg
uint8_t CPU::INX() {
    x++;
    setFlag(Z, x == 0x00);
    setFlag(N, x & 0x80);
    return 0;
}
// inc value at y reg
uint8_t CPU::INY() {
    y++;
    setFlag(Z, y == 0x00);
    setFlag(N, y & 0x80);
    return 0;
}
// jump to location
uint8_t CPU::JMP() {
    pc = addr_abs;
    return 0;
}
// jump to subroutine new location saving ret addr
// push pc to stack, pc = address
uint8_t CPU::JSR() {
    pc--;
    write(0x0100 + stkp, (pc >> 8) & 0x00FF);
    stkp--;
    write(0x0100 + stkp, pc & 0x00FF);
    stkp--;
    pc = addr_abs;
    return 0;
}
// load the accumulator
uint8_t CPU::LDA() {
    fetch();
    acc = fetched;
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
    return 1;
}
// load the x reg
uint8_t CPU::LDX() {
    fetch();
    x = fetched;
    setFlag(Z, x == 0x00);
    setFlag(N, x & 0x80);
    return 1;
}
// load the y reg
uint8_t CPU::LDY() {
    fetch();
    y = fetched;
    setFlag(Z, y == 0x00);
    setFlag(N, y & 0x80);
    return 1;
}
// load status reg
uint8_t CPU::LSR() {
    fetch();
    setFlag(C, fetched & 0x0001);
    temp = fetched >> 1;
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    if (lookup[opcode].addrmode == &CPU::IMP)
        acc = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}
// NOP no operation
uint8_t CPU::NOP() {
    // Not all NOPs are equal, added a few ere
    // Check nesdev wiki unofficial opcodes
    // need to add more based on game compatibility
    // need to cover all illegal opcodes oto
    switch (opcode) {
        case 0x1C:
        case 0x3C:
        case 0x5C:
        case 0x7C:
        case 0xDC:
        case 0xFC:
            return 1;
            break;
    }
    return 0;
}
// bitwise logic or w/ accumulator
uint8_t CPU::ORA(){
    fetch();
    acc = acc | fetched;
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
    return 1;
}
// push acc to stack
uint8_t CPU::PHA() {
    write(0x0100 + stkp, acc);
    stkp--;
    return 0;
}
// push status register to stack
// break flag is set to 1 before push, unused flag always set
uint8_t CPU::PHP() {
    write(0x0100 + stkp, status | B | U);
    setFlag(B, 0);
    setFlag(U, 0);
    stkp--;
    return 0;
}
// pop accumulator off stack
uint8_t CPU::PLA() {
    stkp++;
    acc = read(0x0100 + stkp);
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
}
// pop status reg off stack
uint8_t CPU::PLP() {
    stkp++;
    status = read(0x0100 + stkp);
    setFlag(U, 1);
    return 0;
}
// rotate acc left 1 bit, put old carry bit into bit 0 and old bit 7 into carry
uint8_t CPU::ROL() {
    fetch();
    temp = (uint16_t)(fetched << 1) | getFlag(C);
    setFlag(C, temp & 0xFF00);
    setFlag(Z, (temp & 0x00FF) == 0x0000);
    setFlag(N, temp & 0x0080);
    if (lookup[opcode].addrmode == &CPU::IMP)
        acc = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}
// rotate acc right 1 bit, put old carry bit into bit 7, old bit 7 into carry
uint8_t CPU::ROR() {
    fetch();
    temp = (uint16_t)(getFlag(C) << 7) | (fetched >> 1);
    setFlag(C, fetched & 0x01);
    setFlag(Z, (temp & 0x00FF) == 0x00);
    setFlag(N, temp & 0x0080);
    if (lookup[opcode].addrmode == &CPU::IMP)
        acc = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}
// return from interrupt
// pulls processor flags from stack followed by prog counter
uint8_t CPU::RTI() {
    stkp++;
    status = read(0x0100 + stkp);
    status &= ~B;
    status &= ~U;
    stkp++;
    pc = (uint16_t)read(0x0100 + stkp);
    stkp++;
    pc |= (uint16_t)read(0x0100 + stkp) << 8;
    return 0;
}
// return from subroutine
// pulls pc counter (-1) from the stack
uint8_t CPU::RTS() {
    stkp++;
    pc = (uint16_t)read(0x0100 + stkp);
    stkp++;
    pc |= (uint16_t)read(0x0100 + stkp) << 8;
    pc++;
    return 0;
}
// set carry flag
uint8_t CPU::SEC() {
    setFlag(C, true);
    return 0;
}
// set decimal flag
uint8_t CPU::SED() {
    setFlag(D, true);
    return 0;
}
// set interrupt flag
uint8_t CPU::SEI() {
    setFlag(I, true);
    return 0;
}
// store accumulator at address
uint8_t CPU::STA(){
    write(addr_abs, acc);
    return 0;
}
// store x reg at address
uint8_t CPU::STX(){
    write(addr_abs, x);
    return 0;
}
// store y reg at address
uint8_t CPU::STY(){
    write(addr_abs, y);
    return 0;
}
// transfer acc to x reg
uint8_t CPU::TAX() {
    x = acc;
    setFlag(Z, x == 0x00);
    setFlag(N, x & 0x80);
    return 0;
}
// transfer acc to y reg
uint8_t CPU::TAY() {
    y = acc;
    setFlag(Z, y == 0x00);
    setFlag(N, y & 0x80);
    return 0;
}
// transfer stack pointer to x reg
uint8_t CPU::TSX() {
    x = stkp;
    setFlag(Z, x == 0x00);
    setFlag(N, x & 0x80);
    return 0;
}
// transfer x reg to acc
uint8_t CPU::TXA() {
    acc = x;
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
    return 0;
}
// transfer x reg to stack pointer
uint8_t CPU::TXS() {
    stkp = x;
    return 0;
}
// transfer y reg to acc
uint8_t CPU::TYA() {
    acc = y;
    setFlag(Z, acc == 0x00);
    setFlag(N, acc & 0x80);
    return 0;
}
// handle illegal opcodes
uint8_t CPU::XXX() {
    return 0;
}
// HELPER FUNCTIONS
// checks if instruction is complete
bool CPU::complete() {
    return cycles == 0;
}
// dissaembly function
// not required for emulation, but is convenient
// turns binary instruction code to human readable form
// taking advantage of CPU internal operations
std::map<uint16_t, std::string> CPU::disassemble(uint16_t nStart, uint16_t nStop) {
    uint32_t addr = nStart;
    uint8_t value = 0x00, lo = 0x00, hi = 0x00;
    std::map<uint16_t, std::string> mapLines;
    uint16_t line_addr = 0;

    // convenient utility to convert vars into hex strings
    // lambda function
    auto hex = [](uint32_t n, uint8_t d) {
        std::string s(d, '0');
        for (int i = d - 1; i>= 0; i--, n >>= 4)
            s[i] = "0123456789ABCDEF"[n & 0xF];
        return s;
    };
    // specified addr => instruction byte => lookup table =>
    // how many additional bytes we need to read and what the addr mode is
    // Assemble this info to hr syntax depdening upon addr mode

    // as instruciton is decoded, a string is assembled w/ readable output
    while(addr <= (uint32_t)nStop) {
        line_addr = addr;
        // prefix line w/ instruction addr
        std::string sInst = "$" + hex(addr, 4) + ": ";
        // read instruction and get its readable name
        uint8_t opcode = bus->cpuRead(addr, true);
        addr++;
        sInst += lookup[opcode].name + " ";
        // get operands from desired location
        // form instruction based on addr mode
        // mimics actual fetch routine of 6502
        if (lookup[opcode].addrmode == &CPU::IMP) {
            sInst += " {IMP}";
        } else if (lookup[opcode].addrmode == &CPU::IMM) {
            value = bus->cpuRead(addr, true); addr++;
            sInst += "#$" + hex(value, 2) + " {IMM}";
        } else if (lookup[opcode].addrmode == &CPU::ZP0) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + " {ZP0}";
        } else if (lookup[opcode].addrmode == &CPU::ZPX) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + ", X {ZPX}";
        } else if (lookup[opcode].addrmode == &CPU::ZPY) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + ", Y {ZPY}";
        } else if (lookup[opcode].addrmode == &CPU::IZX) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + ", X {IZX}";
        } else if (lookup[opcode].addrmode == &CPU::IZY) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + ", Y {IZX}";
        } else if (lookup[opcode].addrmode == &CPU::ABS) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = bus->cpuRead(addr, true); addr++;
            sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + " {ABS}";
        } else if (lookup[opcode].addrmode == &CPU::ABX) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = bus->cpuRead(addr, true); addr++;
            sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", X {ABX}";
        } else if (lookup[opcode].addrmode == &CPU::ABY) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = bus->cpuRead(addr, true); addr++;
            sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", Y {ABY}";
        } else if (lookup[opcode].addrmode == &CPU::IND) {
            lo = bus->cpuRead(addr, true); addr++;
            hi = bus->cpuRead(addr, true); addr++;
            sInst += "($" + hex((uint16_t)(hi << 8) | lo, 4) + ") {IND}";
        } else if (lookup[opcode].addrmode == &CPU::REL) {
            value = bus->cpuRead(addr, true); addr++;
            sInst += "$" + hex(value, 2) + "[$" + hex(addr + value, 4) + "] {REL}";
        }
        // add formed string to a std::map using instruction addr as a key
        mapLines[line_addr] = sInst;
    }
    return mapLines;
}