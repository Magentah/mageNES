#include "engine.h"

#include <cassert>

Engine::Engine(bool enableCpuPrint)
{
    m_cpu = std::make_shared<CPU6502>(CPU6502(*this));
    m_cpu->startup(enableCpuPrint);
}

void Engine::load(std::string romFilePath, int prgOffset)
{    
    m_isInit = m_rom.load(romFilePath, prgOffset);
}

void Engine::run()
{
    if (m_isInit) {
        m_cpu->step();
        if (m_cpu->getCycles() >= 27000)
            m_endRunning = true;
    }
}

void Engine::pushStack(uint8_t address, uint8_t data)
{
    m_ram.write(address + 0x0100, data);
}

const uint8_t& Engine::popStack(uint8_t address)
{
    return m_ram.read(address + 0x0100);   
}

const uint8_t& Engine::read(uint16_t address)
{
    if (address < 0x2000)
    {
        return m_ram.read(address);
    }
    else if (address >= 0x2000 && address < 0x4020)
    {
        assert(false);
        // Read from somewhere that's not ram or rom.
    }
    else
    {
        return m_rom.read(address);
    }    
}

const uint16_t Engine::read16(uint16_t address)
{
    auto lsb = read(address);
    auto msb = read(address + 1);
    return (lsb | (msb << 8));
}

void Engine::write(uint16_t address, uint8_t data)
{
    if (address < 0x2000)
    {
        m_ram.write(address, data);
    }
    else
    {
        // Write to somewhere that's not ram.
    }
}