#include "rom.h"

#include <fstream>
#include <bitset>
#include <iostream>

bool ROM::load(const std::string romFilePath, uint16_t prgOffset) 
{
    std::ifstream file(romFilePath, std::ios::binary);
    if (file.is_open()) 
    {
        file.read(reinterpret_cast<char*>(&m_header.nes), sizeof(m_header.nes));
        file.read(reinterpret_cast<char*>(&m_header.prg16kbSize), sizeof(m_header.prg16kbSize));
        file.read(reinterpret_cast<char*>(&m_header.chr8kbSize), sizeof(m_header.chr8kbSize));
        file.read(reinterpret_cast<char*>(&m_header.flags6), sizeof(m_header.flags6));
        file.read(reinterpret_cast<char*>(&m_header.flags7), sizeof(m_header.flags7));
        file.read(reinterpret_cast<char*>(&m_header.flags8), sizeof(m_header.flags8));
        file.read(reinterpret_cast<char*>(&m_header.flags9), sizeof(m_header.flags9));
        file.read(reinterpret_cast<char*>(&m_header.flags10), sizeof(m_header.flags10));
        file.read(reinterpret_cast<char*>(&m_header.zeros), sizeof(m_header.zeros));

        m_trainer.reserve(512);
        auto prgSize = m_header.prg16kbSize * PRG_SIZE_MULTIPLE;
        auto chrSize = m_header.chr8kbSize * CHR_SIZE_MULTIPLE;
        m_prgData.resize(prgSize);
        m_chrData.resize(chrSize);

        m_isMirroring = (m_header.flags6 & Flag6::MIRRORING);
        if (m_header.flags6 & Flag6::TRAINER_EXISTS) {
            file.read((char*) m_trainer.data(), 512);
        }

        file.read((char*) m_prgData.data(), prgSize);
        if (m_header.chr8kbSize > 0) 
        {
            file.read((char*) m_chrData.data(), chrSize);
        } else {
            m_chrData = std::vector<uint8_t>(CHR_SIZE_MULTIPLE, 0);
        }
        m_prgOffset = prgOffset;
        m_isLoaded = true;
        return true;
    }
    return false;
}

void ROM::printHeader() 
{
    if (m_isLoaded) 
    {
        std::cout << "ROM Header :: " << std::endl;
        std::cout << "Signature: " << m_header.nes << std::endl;
        std::cout << "PRG ROM Size: " << m_prgData.size() << std::endl;
        std::cout << "CHR ROM Size: " << m_chrData.size() << std::endl;
        std::cout << "Is Mirroring: " << m_isMirroring << std::endl;
        
        std::bitset<8> flag6bits(m_header.flags6);
        std::bitset<8> flag7bits(m_header.flags6);

        std::cout << "Flag 6 bits: " << flag6bits << std::endl;
        std::cout << "Flag 7 bits: " << flag7bits << std::endl;
    }
}

const uint8_t& ROM::read(uint16_t address)
{
    address = (address - m_prgOffset) % m_prgData.size();
    return m_prgData[address];
}