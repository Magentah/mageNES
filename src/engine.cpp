#include "engine.h"

#include <cassert>

Engine::Engine()
{
    this->cpu = std::make_shared<CPU6502>(CPU6502(*this));
}

void Engine::load(std::string romFilePath, int prgOffset)
{
    this->cpu->startup();
    this->isInit = this->rom.load(romFilePath, prgOffset);
}

void Engine::run()
{
    if (this->isInit)
    {
        this->isRunning = true;
        while (this->isRunning)
        {
            this->cpu->step();
        }
    }
}

void Engine::pushStack(uint8_t address, uint8_t data)
{
    this->ram.write(address + 0x0100, data);
}

const uint8_t& Engine::popStack(uint8_t address)
{
    return this->ram.read(address + 0x0100);   
}

const uint8_t& Engine::read(uint16_t address)
{
    if (address < 0x2000)
    {
        return this->ram.read(address);
    }
    else if (address >= 0x2000 && address < 0x4020)
    {
        assert(false);
    }
    else
    {
        return this->rom.read(address);
    }    
}

const uint16_t Engine::read16(uint16_t address)
{
    auto lsb = this->read(address);
    auto msb = this->read(address + 1);
    return (lsb | (msb << 8));
}

void Engine::write(uint16_t address, uint8_t data)
{
    if (address < 0x2000)
    {
        this->ram.write(address, data);
    }
    else
    {
        assert(false);
    }
}