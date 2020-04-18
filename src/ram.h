#pragma once

#include <cstdint>
#include <memory>

#define RAM_SIZE 2048

class RAM
{
private: 
    uint8_t m_ram[RAM_SIZE] = {0};


public:
    const uint8_t& read(uint16_t address);
    void write(uint16_t address, uint8_t data);
};