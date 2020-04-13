#pragma once

#include <cstdint>

#define RAM_SIZE 2048

class RAM
{
private: 
    uint8_t ram[RAM_SIZE] = {0};

public:
    uint8_t* read(uint16_t address);
    void write(uint16_t address, uint8_t data);
};