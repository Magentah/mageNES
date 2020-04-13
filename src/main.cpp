#include <iostream>
#include "rom.h"
#include "cpu.h"

int main(int argc, char** argv) {
    ROM rom;
    rom.load("tests/nestest.nes", 0xC000);
    rom.printHeader();

    CPU6502 cpu;
    cpu.startup();
    cpu.load(rom);
    while(true)
    {
        cpu.step();
    }

    cpu.shutdown();
    std::cin.get();
    return 0;
}