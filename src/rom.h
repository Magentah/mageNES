#pragma once

#include <vector>
#include <string>
#include <memory>

#define PRG_SIZE_MULTIPLE 16384
#define CHR_SIZE_MULTIPLE 8192

enum Flag6 {
    MIRRORING = 0x01,
    BATTERY_RAM = 0x02,
    TRAINER_EXISTS = 0x04,
    IGNORE_MIRRORING = 0x08,
};

enum Flag7 {
    VS_UNISYSTEM = 0x01,
    PLAYCHOICE_10 = 0x02,
    NES_2_FORMAT = 0x04,
};

struct INESHeader {
    unsigned char nes[4];
    uint8_t prg16kbSize;
    uint8_t chr8kbSize;
    uint8_t flags6;
    uint8_t flags7;
    uint8_t flags8; //prg ram size
    uint8_t flags9;
    uint8_t flags10;
    unsigned char zeros[5];
};

class ROM {
private:
    INESHeader m_header;
    std::vector<uint8_t> m_trainer;
    std::vector<uint8_t> m_prgData;   // Program Data
    std::vector<uint8_t> m_chrData;   // Character Data
    std::vector<uint8_t> m_instRom;
    std::vector<uint8_t> m_prom;
    std::vector<uint8_t> m_title;

    bool m_isLoaded = false;
    bool m_isMirroring;
    uint8_t m_mapperNumber;
    uint16_t m_prgOffset;
public:
    bool load(const std::string romFilePath, uint16_t prgOffset);
    void printHeader();
    const uint8_t& read(uint16_t address);
};