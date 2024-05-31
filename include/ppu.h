#pragma once
#include <cstdint>
#include <memory>
#include "cartridge.h"
#include "oldPixelGameEngine.h"

class PPU {

public:
    PPU();
    ~PPU();
private:
    uint8_t tblName[2][1024];
    uint8_t tblPattern[2][4096];
    uint8_t tblPalette[32];
private:
    olc::Pixel palScreen[0x40];
    olc::Sprite* sprScreen;
    olc::Sprite* sprNameTable[2];
    olc::Sprite* sprPatternTable[2];
public:
    // debug utils
    olc::Sprite& getScreen();
    olc::Sprite& getNameTable(uint8_t i);
    olc::Sprite& getPatternTable(uint8_t i, uint8_t palette);

    olc::Pixel& getColorFromPaletteRam(uint8_t palette, uint8_t pixel);

    bool frame_complete=  false;
private:
    union {
        struct {
            uint8_t unused : 5;
            uint8_t sprite_overflow: 1;
            uint8_t sprite_zero_hit : 1;
            uint8_t vertical_blank : 1;
        };
        uint8_t reg;
    } status;

    union {
        struct {
            uint8_t grayscale : 1;
            uint8_t render_background_left : 1;
            uint8_t render_sprites_left : 1;
            uint8_t render_background : 1;
            uint8_t render_sprites : 1;
            uint8_t enhance_red : 1;
            uint8_t enhance_green : 1;
            uint8_t enhance_blue : 1;
        };
        uint8_t reg;
    } mask;
    union PPUCTRL {
        struct {
            uint8_t nametable_x : 1;
            uint8_t nametable_y : 1;
            uint8_t increment_mode : 1;
            uint8_t pattern_sprite : 1;
            uint8_t pattern_background : 1;
            uint8_t sprite_size : 1;
            uint8_t slave_mode : 1;
            uint8_t enable_nmi : 1;
        };
        uint8_t reg;
    } control;
    
    union PPU_register {
        struct {
            uint16_t coarse_x : 5;
            uint16_t coarse_y : 5;
            uint16_t nametable_x : 1;
            uint16_t nametable_y : 1;
            uint16_t fine_y : 3;
            uint16_t unused : 1;
        };
        uint16_t reg = 0x0000;
    };
    PPU_register vram_addr; // Active "pointer" addr into nametable to extract bg tile info
    PPU_register tram_addr; // Temp store of info to be transferred into pointer at various times

    // pixel offset horizontally
    uint8_t fine_x = 0x00;
    // internal communications
    uint8_t address_latch = 0x00;
    uint8_t ppu_data_buffer = 0x00;
    // Pixel "dot" position information
    int16_t scanline = 0;
    int16_t cycle = 0;

    // Background rendering
    uint8_t bg_next_tile_id = 0x00;
    uint8_t bg_next_tile_attrib = 0x00;
    uint8_t bg_next_tile_lsb = 0x00;
    uint8_t bg_next_tile_msb = 0x00;
    uint16_t bg_shifter_pattern_lo = 0x0000;
    uint16_t bg_shifter_pattern_hi = 0x0000;
    uint16_t bg_shifter_attrib_lo = 0x0000;
    uint16_t bg_shifter_attrib_hi = 0x0000;

    // Foreground "Sprite" rendering
    // OAM is additional mmeory internal to PPU not connected ot any bus
    // stores locaitons of 64off 8x8 (or 8x16) tiles to be rawn next frame
    struct sObjectAttributeEntry {
        uint8_t y; // y pos of sprite
        uint8_t id; // id of tile from pattern mem
        uint8_t attribute; // flags define how sprite should be rendered
        uint8_t x; // x pos of sprite
    } OAM[64];
    // a reg to store the addr when the CPU manually communicates with OAM
    // via PPU reg. Not commonly used bc it is slow and a 256-byte DMA transfer
    // is usually used
    uint8_t oam_addr = 0x00;
    sObjectAttributeEntry spriteScanline[8];
    uint8_t sprite_count;
    uint8_t sprite_shifter_pattern_lo[8];
    uint8_t sprite_shifter_pattern_hi[8];
    // sprite zero collision flags
    bool bSpriteZeroHitPossible = false;
    bool bSpriteZeroBeingRendered = false;
    // DMA mechanism needs access to OAM to write one byte at a time
public:
    uint8_t* pOAM = (uint8_t*)OAM;
    // communications w bus
    uint8_t cpuRead(uint16_t addr, bool rdonly = false);
    void cpuWrite(uint16_t addr, uint8_t data);
    // comm w ppu bus
    uint8_t ppuRead(uint16_t addr, bool rdonly = false);
    void ppuWrite(uint16_t addr, uint8_t data);
private:
    // cartridge
    std::shared_ptr<Cartridge> cart;
public:
    // Interface
    void connectCartridge(const std::shared_ptr<Cartridge>& cartridge);
    void clock();
    void reset();
    bool nmi = false;
};