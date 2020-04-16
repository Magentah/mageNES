#include "rom.h"

#include <fstream>
#include <bitset>
#include <iostream>

bool ROM::load(const std::string romFilePath, uint16_t prgOffset) 
{
    std::ifstream file(romFilePath, std::ios::binary);
    if (file.is_open()) 
    {
        file.read(reinterpret_cast<char*>(&this->header.nes), sizeof(this->header.nes));
        file.read(reinterpret_cast<char*>(&this->header.prg16kbSize), sizeof(this->header.prg16kbSize));
        file.read(reinterpret_cast<char*>(&this->header.chr8kbSize), sizeof(this->header.chr8kbSize));
        file.read(reinterpret_cast<char*>(&this->header.flags6), sizeof(this->header.flags6));
        file.read(reinterpret_cast<char*>(&this->header.flags7), sizeof(this->header.flags7));
        file.read(reinterpret_cast<char*>(&this->header.flags8), sizeof(this->header.flags8));
        file.read(reinterpret_cast<char*>(&this->header.flags9), sizeof(this->header.flags9));
        file.read(reinterpret_cast<char*>(&this->header.flags10), sizeof(this->header.flags10));
        file.read(reinterpret_cast<char*>(&this->header.zeros), sizeof(this->header.zeros));

        this->trainer.reserve(512);
        auto prgSize = this->header.prg16kbSize * this->PRG_SIZE_MULTIPLE;
        auto chrSize = this->header.chr8kbSize * this->CHR_SIZE_MULTIPLE;
        this->prgData.resize(prgSize);
        this->chrData.resize(chrSize);

        this->isMirroring = (this->header.flags6 & Flag6::MIRRORING);
        if (this->header.flags6 & Flag6::TRAINER_EXISTS) {
            file.read((char*) this->trainer.data(), 512);
        }

        file.read((char*) this->prgData.data(), prgSize);
        if (this->header.chr8kbSize > 0) 
        {
            file.read((char*) this->chrData.data(), chrSize);
        } else {
            this->chrData = std::vector<uint8_t>(this->CHR_SIZE_MULTIPLE, 0);
        }
        this->prgOffset = prgOffset;
        this->isLoaded = true;
        return true;
    }
    return false;
}

void ROM::printHeader() 
{
    if (this->isLoaded) 
    {
        std::cout << "ROM Header :: " << std::endl;
        std::cout << "Signature: " << this->header.nes << std::endl;
        std::cout << "PRG ROM Size: " << this->prgData.size() << std::endl;
        std::cout << "CHR ROM Size: " << this->chrData.size() << std::endl;
        std::cout << "Is Mirroring: " << this->isMirroring << std::endl;
        
        std::bitset<8> flag6bits(this->header.flags6);
        std::bitset<8> flag7bits(this->header.flags6);

        std::cout << "Flag 6 bits: " << flag6bits << std::endl;
        std::cout << "Flag 7 bits: " << flag7bits << std::endl;
    }
}

std::shared_ptr<uint8_t> ROM::read(uint16_t address)
{
    address = (address - this->prgOffset) % this->prgData.size();
    return std::make_shared<uint8_t>(std::move(this->prgData[address]));
}