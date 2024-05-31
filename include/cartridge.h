#pragma once
#include <cstdint>
#include <string>
#include <fstream>
#include <vector>
#include <memory>

#include "mapper_000.h"
#include "mapper_001.h"
#include "mapper_002.h"
#include "mapper_003.h"
#include "mapper_004.h"
#include "mapper_066.h"

class Cartridge {
    public:
        Cartridge(const std::string& sFileName);
        ~Cartridge();

        bool ImageValid();
        // enum MIRROR {
        //     HORIZONTAL,
        //     VERTICAL,
        //     ONSCREEN_LO,
        //     ONSCREEN_HI,
        // } mirror = HORIZONTAL;
    private:
        bool bImageValid = false;
        MIRROR hw_mirror = HORIZONTAL;
        
        uint8_t nMapperID = 0; // mapper num
        uint8_t nPRGBanks = 0; // program bank num
        uint8_t nCHRbanks = 0; // character bank?

        std::vector<uint8_t> vPRGMemory; // virt prog memory (CPU)
        std::vector<uint8_t> vCHRMemory; // virt character memory (PPU)

        std::shared_ptr<Mapper> pMapper;
    public:
        bool cpuRead(uint16_t addr, uint8_t &data);
        bool cpuWrite(uint16_t addr, uint8_t data);

        bool ppuRead(uint16_t addr, uint8_t &data);
        bool ppuWrite(uint16_t addr, uint8_t data);

        // Permits system rest of mapper to know state
        void reset();
        // get mirror onfig
        MIRROR Mirror();
        std::shared_ptr<Mapper> getMapper();
};