#include "ram.h"

const uint8_t& RAM::read(uint16_t address)
{
    address %= RAM_SIZE;
    return ram[address];
}

void RAM::write(uint16_t address, uint8_t data)
{
    address %= RAM_SIZE;
    ram[address] = data;
}