#include "cartridge.h"

Cartridge::Cartridge(const std::string& sFileName) {
    // iNes Format Header
    struct sHeader {
        char name[4];
        uint8_t prg_rom_chunks;
        uint8_t chr_rom_chunks;
        uint8_t mapper1;
        uint8_t mapper2;
        uint8_t prg_ram_size;
        uint8_t tv_system1;
        uint8_t tv_system2;
        char unused[5];
    } header;
    bImageValid = false;
    std::ifstream ifs;
    // open rom file
    ifs.open(sFileName, std::ifstream::binary);
    if (ifs.is_open()) {
        // read header n bytes
        ifs.read((char*)&header, sizeof(sHeader));
        // if a "trainer" exists read pst it
        if (header.mapper1 & 0x04) {
            ifs.seekg(512, std::ios_base::cur);
        }
        // det mapper id
        nMapperID = ((header.mapper2 >> 4) << 4) | (header.mapper1 >> 4);
        hw_mirror = (header.mapper1 & 0x01) ? VERTICAL : HORIZONTAL;
        // discover file format
        uint8_t nFileType = 1;
        if (nFileType == 0) {}
        if (nFileType == 1) {
                nPRGBanks = header.prg_rom_chunks;
                vPRGMemory.resize(nPRGBanks*16384); // banks are 16kb
                ifs.read((char*)vPRGMemory.data(), vPRGMemory.size()); // read in rom data into memory

                nCHRbanks = header.chr_rom_chunks;
                vCHRMemory.resize(nCHRbanks * 8192); // PPU banks are 8kb
                ifs.read((char*)vCHRMemory.data(), vCHRMemory.size()); // read in PPU data
        }
        if (nFileType == 2) {
            nPRGBanks = ((header.prg_ram_size & 0x07) << 8) | header.prg_rom_chunks;
            vPRGMemory.resize(nPRGBanks * 16384);
            ifs.read((char*)vPRGMemory.data(), vPRGMemory.size());
            nCHRbanks = ((header.prg_ram_size & 0x38) << 8) | header.chr_rom_chunks;
            vCHRMemory.resize(nCHRbanks * 8192);
            ifs.read((char*)vCHRMemory.data(), vCHRMemory.size());
        }
        // load appropirate mapper
        switch (nMapperID) {
            case 0:
                pMapper = std::make_shared<Mapper_000>(nPRGBanks, nCHRbanks);
                break;
            case 1:
                pMapper = std::make_shared<Mapper_001>(nPRGBanks, nCHRbanks);
                break;
            case 2:
                pMapper = std::make_shared<Mapper_002>(nPRGBanks, nCHRbanks);
                break;
            case 3:
                pMapper = std::make_shared<Mapper_003>(nPRGBanks, nCHRbanks);
                break;
            case 4:
                pMapper = std::make_shared<Mapper_004>(nPRGBanks, nCHRbanks);
                break;
            case 66:
                pMapper = std::make_shared<Mapper_066>(nPRGBanks, nCHRbanks);
                break;
        }
        bImageValid = true;
        ifs.close();
    }
}
Cartridge::~Cartridge() {}

bool Cartridge::ImageValid() {
    return bImageValid;
}

bool Cartridge::cpuRead(uint16_t addr, uint8_t &data) {
    uint32_t mapped_addr = 0;
    if (pMapper->cpuMapRead(addr, mapped_addr, data)) {
        if (mapped_addr == 0xFFFFFFFF) {
            // mapper has set the data value, ex cartridge based ram
            return true;
        }
        data = vPRGMemory[mapped_addr]; // read from mapped prg memory
        return true;
    } else {
        return false;
    }
}

bool Cartridge::cpuWrite(uint16_t addr, uint8_t data) {
    uint32_t mapped_addr = 0;
    if (pMapper->cpuMapWrite(addr, mapped_addr)) {
        if (mapped_addr == 0xFFFFFFFF) return true;
        vPRGMemory[mapped_addr] = data; // write data to mapped addr in virt mem
        return true;
    } else {
        return false;
    }
}

bool Cartridge::ppuRead(uint16_t addr, uint8_t &data) {
    uint32_t mapped_addr = 0;
    if (pMapper->ppuMapRead(addr, mapped_addr)) {
        data = vCHRMemory[mapped_addr]; // read from mapped ppu mem
        return true;
    } else {
        return false;
    }
}

bool Cartridge::ppuWrite(uint16_t addr, uint8_t data) {
    uint32_t mapped_addr = 0;
    if (pMapper->ppuMapRead(addr, mapped_addr)) {
        vCHRMemory[mapped_addr] = data; // write to mapped ppu mem
        return true;
    } else {
        return false;
    }
}

void Cartridge::reset() {
    // Does not reset ROM contents, resets the mapper
    if (pMapper != nullptr) {
        pMapper->reset();
    }
}
MIRROR Cartridge::Mirror() {
    MIRROR m = pMapper->mirror();
    if (m == MIRROR::HARDWARE) {
        // mirror cfg was defined in hardware via soldering
        return hw_mirror;
    } else {
        // mirror cfg can be set dynamically via mapper
        return m;
    }
}
std::shared_ptr<Mapper> Cartridge::getMapper() {
    return pMapper;
}