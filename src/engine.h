#pragma once

#include <memory>

#include "cpu.h"
#include "ram.h"
#include "rom.h"

class CPU6502;

class Engine
{
public:
    Engine(bool enableCpuPrint);
    ~Engine() {};

    void load(std::string romFilePath, int pgrOffset);

    const uint8_t& read(uint16_t address);
    const uint16_t read16(uint16_t address);
    void pushStack(uint8_t address, uint8_t data);
    const uint8_t& popStack(uint8_t address);
    void write(uint16_t address, uint8_t data);
    void run();
    bool endRunning() const { return m_endRunning; };

private:
    std::shared_ptr<CPU6502> m_cpu;
    RAM m_ram;
    ROM m_rom;
    uint8_t m_stackPointer;
    uint64_t m_totalCycleCount;

    bool m_isRunning;
    bool m_isInit;
    bool m_endRunning = false;
};