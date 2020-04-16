#include <iostream>
#include "rom.h"
#include "cpu.h"
#include "engine.h"

int main(int argc, char** argv) {
    Engine engine;
    engine.load("tests/nestest.nes", 0xC000);
    engine.run();
    std::cin.get();
    return 0;
}