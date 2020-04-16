#pragma once

#include <memory>

#include "cpu.h"
#include "ram.h"
#include "rom.h"

class CPU6502;

class Engine
{
private:
    std::shared_ptr<CPU6502> cpu;
    RAM ram;
    ROM rom;

    bool isRunning;
    bool isInit;
public:
    Engine();
    ~Engine() {};

    void load(std::string romFilePath, int pgrOffset);

    std::shared_ptr<uint8_t> read(uint16_t address);
    std::shared_ptr<uint16_t> read16(uint16_t address);
    void write(uint16_t address, uint8_t data);
    void run();
};