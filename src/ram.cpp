#include "ram.h"

std::shared_ptr<uint8_t> RAM::read(uint16_t address)
{
    address %= RAM_SIZE;
    return std::make_shared<uint8_t>(ram[address]);
}

void RAM::write(uint16_t address, uint8_t data)
{
    address %= RAM_SIZE;
    ram[address] = data;
}