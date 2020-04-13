#include "cpu.h"

#include <iostream>
#include <iomanip>

void CPU6502::startup()
{
    this->registers.x = 0;
    this->registers.y = 0;
    this->registers.accumulator = 0;
    this->registers.programCounter = PROGRAM_COUNTER_STARTUP_VAL;
    this->registers.stackPointer = STACK_POINTER_STARTUP_VAL;
}

void CPU6502::reset()
{
    startup();
}

uint8_t CPU6502::getInstruction()
{
    uint8_t instruction = *this->rom->read(this->registers.programCounter);
    return instruction;
}

void CPU6502::notImplemented(uint8_t instruction)
{
    std::cout << "Not implemented opcode: " << std::hex << static_cast<int>(instruction) << std::endl;
}

void CPU6502::unknownInstruction(uint8_t instruction) 
{
    std::cout << "Unknown opcode: " << std::hex << static_cast<int>(instruction) << std::endl;
    this->registers.programCounter++;
}

void CPU6502::printStatus(uint8_t instruction)
{
    std::cout << "opcode: " << std::hex << std::setw(4) << std::setfill('0') << std::uppercase << static_cast<int>(instruction) << std::endl;
    std::cout << "a: " << std::setw(4) << static_cast<int>(this->registers.accumulator) << std::endl;
    std::cout << "x: " << std::setw(4) << static_cast<int>(this->registers.x) << std::endl;
    std::cout << "y: " << std::setw(4) << static_cast<int>(this->registers.y) << std::endl;
    std::cout << "pc: " << std::setw(4) << static_cast<int>(this->registers.programCounter) << std::endl;
    std::cout << "sp: " << std::setw(4) << static_cast<int>(this->registers.stackPointer) << std::endl;
    std::cout << "st: " << std::setw(4) << static_cast<int>(this->registers.statusRegister) << std::endl;
    std::cout << "cycle: " << std::dec << this->cycle << std::endl;
}

void CPU6502::executeInstruction(uint8_t instruction)
{
    this->printStatus(instruction);
    switch (instruction)
    {
        default: this->unknownInstruction(instruction); break;

        // ADC - Add with carry
        case 0x61: adc(AddressingMode::INDIRECT_X, 6); break;
        case 0x65: adc(AddressingMode::ZERO_PAGE, 3); break;
        case 0x69: adc(AddressingMode::IMMEDIATE, 2); break;
        case 0x6D: adc(AddressingMode::ABSOLUTE, 4); break;
        case 0x71: adc(AddressingMode::INDIRECT_Y, 5); break;
        case 0x75: adc(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x79: adc(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x7D: adc(AddressingMode::ABSOLUTE_X, 4); break;
        
        // AND
        case 0x21: AND(AddressingMode::INDIRECT_X,6); break;
        case 0x25: AND(AddressingMode::ZERO_PAGE, 3); break;
        case 0x29: AND(AddressingMode::IMMEDIATE, 2); break;
        case 0x2D: AND(AddressingMode::ABSOLUTE, 4); break;
        case 0x31: AND(AddressingMode::INDIRECT_Y, 5); break;
        case 0x35: AND(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x39: AND(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x3D: AND(AddressingMode::ABSOLUTE_X, 4); break;

        // ASL - Arithmetic Shift Left
        case 0x06: this->notImplemented(instruction); break;
        case 0x0A: this->notImplemented(instruction); break;
        case 0x0E: this->notImplemented(instruction); break;
        case 0x16: this->notImplemented(instruction); break;
        case 0x1E: this->notImplemented(instruction); break;

        // Branching
        case 0x90: this->notImplemented(instruction); break;    // BCC - Branch if carry clear
        case 0xB0: bcs(AddressingMode::RELATIVE, 3); break;    // BCS - Branch if carry set
        case 0xF0: this->notImplemented(instruction); break;    // BEQ - Branch if equal
        case 0x30: this->notImplemented(instruction); break;    // BMI - Branch if minus
        case 0xD0: this->notImplemented(instruction); break;    // BNE - Branch if not equal
        case 0x10: this->notImplemented(instruction); break;    // BPL - Branch if positive
        case 0x50: this->notImplemented(instruction); break;    // BVC - Branch if overflow clear
        case 0x70: this->notImplemented(instruction); break;    // BVS - Branch if overflow set

        // Bit test
        case 0x24: this->notImplemented(instruction); break;
        case 0x2C: this->notImplemented(instruction); break;

        // Force Interrupt
        case 0x00: this->notImplemented(instruction); break;

        // Clear
        case 0x18: this->notImplemented(instruction); break;    // Clear carry flag
        case 0xD8: this->notImplemented(instruction); break;    // Clear decimal flag
        case 0x58: this->notImplemented(instruction); break;    // Clear interrupt disable
        case 0xB8: this->notImplemented(instruction); break;    // Clear overflow flag

        // CMP - Compare
        case 0xC1: this->notImplemented(instruction); break;
        case 0xC5: this->notImplemented(instruction); break;
        case 0xC9: this->notImplemented(instruction); break;
        case 0xCD: this->notImplemented(instruction); break;
        case 0xD1: this->notImplemented(instruction); break;
        case 0xD5: this->notImplemented(instruction); break;
        case 0xD9: this->notImplemented(instruction); break;
        case 0xDD: this->notImplemented(instruction); break;

        // CPX - Compare X Register
        case 0xE0: this->notImplemented(instruction); break;
        case 0xE4: this->notImplemented(instruction); break;
        case 0xEC: this->notImplemented(instruction); break;

        // CPY - Compare Y Register
        case 0xC0: this->notImplemented(instruction); break;
        case 0xC4: this->notImplemented(instruction); break;
        case 0xCC: this->notImplemented(instruction); break;

        // DEC - Decrement Memory
        case 0xC6: this->notImplemented(instruction); break;
        case 0xCE: this->notImplemented(instruction); break;
        case 0xD6: this->notImplemented(instruction); break;
        case 0xDE: this->notImplemented(instruction); break;

        // DEX - Decrement X Register
        case 0xCA: this->notImplemented(instruction); break;

        // DEY - Decrement Y Register
        case 0x88: this->notImplemented(instruction); break;

        // EOR - Exclusive OR
        case 0x41: this->notImplemented(instruction); break;
        case 0x45: this->notImplemented(instruction); break;
        case 0x49: this->notImplemented(instruction); break;
        case 0x4D: this->notImplemented(instruction); break;
        case 0x51: this->notImplemented(instruction); break;
        case 0x55: this->notImplemented(instruction); break;
        case 0x59: this->notImplemented(instruction); break;
        case 0x5D: this->notImplemented(instruction); break;

        // INC - Increment Memory
        case 0xE6: this->notImplemented(instruction); break;
        case 0xEE: this->notImplemented(instruction); break;
        case 0xF6: this->notImplemented(instruction); break;
        case 0xFE: this->notImplemented(instruction); break;

        // INX - Incremement X Register
        case 0xE8: this->notImplemented(instruction); break;

        // INY - Increment Y Register
        case 0xC8: this->notImplemented(instruction); break;

        // JMP 
        case 0x4C: jmp(AddressingMode::ABSOLUTE, 3); break;
        case 0x6C: jmp(AddressingMode::INDIRECT, 5); break;

        // JSR - Jump to subroutine
        case 0x20: jsr(AddressingMode::ABSOLUTE, 6); break;

        // LDA - Load Accumulator
        case 0xA1: this->notImplemented(instruction); break;
        case 0xA5: this->notImplemented(instruction); break;
        case 0xA9: this->notImplemented(instruction); break;
        case 0xAD: this->notImplemented(instruction); break;
        case 0xB1: this->notImplemented(instruction); break;
        case 0xB5: this->notImplemented(instruction); break;
        case 0xB9: this->notImplemented(instruction); break;
        case 0xBD: this->notImplemented(instruction); break;

        // LDX - Load X Register
        case 0xA2: ldx(AddressingMode::IMMEDIATE, 2); break;
        case 0xA6: ldx(AddressingMode::ZERO_PAGE, 3); break;
        case 0xAE: ldx(AddressingMode::ZERO_PAGE_Y, 4); break;
        case 0xB6: ldx(AddressingMode::ABSOLUTE, 4); break;
        case 0xBE: ldx(AddressingMode::ABSOLUTE_Y, 4); break;

        // LDY - Load Y Register
        case 0xA0: this->notImplemented(instruction); break;
        case 0xA4: this->notImplemented(instruction); break;
        case 0xAC: this->notImplemented(instruction); break;
        case 0xB4: this->notImplemented(instruction); break;
        case 0xBC: this->notImplemented(instruction); break;

        // LSR - Logical Shift Right
        case 0x46: this->notImplemented(instruction); break;
        case 0x4A: this->notImplemented(instruction); break;
        case 0x4E: this->notImplemented(instruction); break;
        case 0x56: this->notImplemented(instruction); break;
        case 0x5E: this->notImplemented(instruction); break;

        // NOP
        case 0xEA: nop(AddressingMode::IMMEDIATE, 2); break;

        // ORA - Logical Inclusive OR
        case 0x01: this->notImplemented(instruction); break;
        case 0x05: this->notImplemented(instruction); break;
        case 0x09: this->notImplemented(instruction); break;
        case 0x0D: this->notImplemented(instruction); break;
        case 0x11: this->notImplemented(instruction); break;
        case 0x15: this->notImplemented(instruction); break;
        case 0x19: this->notImplemented(instruction); break;
        case 0x1D: this->notImplemented(instruction); break;

        // PHA - Push Accumulator
        case 0x48: this->notImplemented(instruction); break;

        // PHP - Push Processor Status
        case 0x08: this->notImplemented(instruction); break;

        // PLA - Pull Accumulator
        case 0x68: this->notImplemented(instruction); break;

        // PLP - Pull Processor Status
        case 0x28: this->notImplemented(instruction); break;

        // ROL - Rotate Left
        case 0x26: this->notImplemented(instruction); break;
        case 0x2A: this->notImplemented(instruction); break;
        case 0x2E: this->notImplemented(instruction); break;
        case 0x36: this->notImplemented(instruction); break;
        case 0x3E: this->notImplemented(instruction); break;

        // ROR - Rotate Right
        case 0x66: this->notImplemented(instruction); break;
        case 0x6A: this->notImplemented(instruction); break;
        case 0x6E: this->notImplemented(instruction); break;
        case 0x76: this->notImplemented(instruction); break;
        case 0x7E: this->notImplemented(instruction); break;

        // RTI - Return from interrupt
        case 0x40: this->notImplemented(instruction); break;

        // RTS - Return from subroutine
        case 0x60: this->notImplemented(instruction); break;

        // SBC - Subtract with carry
        case 0xE1: this->notImplemented(instruction); break;
        case 0xE5: this->notImplemented(instruction); break;
        case 0xE9: this->notImplemented(instruction); break;
        case 0xED: this->notImplemented(instruction); break;
        case 0xF1: this->notImplemented(instruction); break;
        case 0xF5: this->notImplemented(instruction); break;
        case 0xF9: this->notImplemented(instruction); break;
        case 0xFD: this->notImplemented(instruction); break;

        // SEC - Set Carry Flag
        case 0x38: sec(2); break;

        // SED - Set Decimal Flag
        case 0xF8: this->notImplemented(instruction); break;

        // SEI - Set Interrupt Disable
        case 0x78: this->notImplemented(instruction); break;

        // STA - Store Accumulator
        case 0x81: this->notImplemented(instruction); break;
        case 0x85: this->notImplemented(instruction); break;
        case 0x89: this->notImplemented(instruction); break;
        case 0x8D: this->notImplemented(instruction); break;
        case 0x91: this->notImplemented(instruction); break;
        case 0x95: this->notImplemented(instruction); break;
        case 0x99: this->notImplemented(instruction); break;
        case 0x9D: this->notImplemented(instruction); break;

        //　STX - Store X Register
        case 0x86: stx(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8E: stx(AddressingMode::ABSOLUTE, 4); break;
        case 0x96: stx(AddressingMode::ZERO_PAGE_Y, 4); break;

        // STY - Store Y Register
        case 0x84: this->notImplemented(instruction); break;
        case 0x8C: this->notImplemented(instruction); break;
        case 0x94: this->notImplemented(instruction); break;

        // TAX - Transfer Accumulator to X
        case 0xAA: this->notImplemented(instruction); break;

        // TAY - Transfer Accumulator to Y
        case 0xA8: this->notImplemented(instruction); break;

        // TSX - Transfer Stack Pointer to X
        case 0xBA: this->notImplemented(instruction); break;

        // TXA - Transfer X to Accumulator
        case 0x8A: this->notImplemented(instruction); break;

        // TXS - Transfer X to Stack Pointer
        case 0x9A: this->notImplemented(instruction); break;

        // TYA - Transfer Y to Accumulator
        case 0x98: this->notImplemented(instruction); break;
    }
}

void CPU6502::step()
{
    uint8_t instruction = this->getInstruction();
    this->executeInstruction(instruction);
    this->registers.programCounter++;
}

void CPU6502::load(ROM& rom) 
{
    this->rom = &rom;
}

void CPU6502::shutdown()
{
    this->rom = nullptr;
}

void CPU6502::setStatusFlag(StatusFlag flag, bool enabled)
{
    if (enabled)
        this->registers.statusRegister |= flag;
    else 
        this->registers.statusRegister &= ~flag;
}

// Page is 256 bytes. If pages change, increase cycle.
void CPU6502::tickIfNewPage(uint16_t programCounter, uint16_t newProgramCounter)
{
    uint16_t newPcMSB = newProgramCounter >> 8;
    uint16_t oldPcMSB = programCounter >> 8;
    
    if (newPcMSB != oldPcMSB) {
        tick();
    }
}

void CPU6502::tick(int times)
{
    for (int i = 0; i < times; i++) 
    {
        this->cycle++;
        // 3x ppu ticks
    }
}

// Addressing Modes
uint16_t CPU6502::immediate()
{
    return *this->rom->read(++this->registers.programCounter);
}

uint16_t CPU6502::relative()
{
    uint8_t offset = this->immediate();
    return this->registers.programCounter + offset;
}

uint16_t CPU6502::zeroPage()
{
    uint8_t address = *this->rom->read(++this->registers.programCounter);
    return address % 256;
}

uint16_t CPU6502::zeroPageX()
{
    uint8_t address = *this->rom->read(++this->registers.programCounter);
    return (address + this->registers.x) % 256;
}

uint16_t CPU6502::zeroPageY()
{
    uint8_t address = *this->rom->read(++this->registers.programCounter);
    return (address + this->registers.y) % 256;
}

uint16_t CPU6502::absolute()
{
    uint8_t lsb = *this->rom->read(this->registers.programCounter + 1);
    uint8_t msb = *this->rom->read(this->registers.programCounter + 2);
    uint16_t address = (msb << 8) + lsb;
    return address;
}

uint16_t CPU6502::absoluteX(bool extraTick)
{
    uint16_t address = this->absolute();
    if (extraTick)
        this->tickIfNewPage(address, address + this->registers.x);
    return address + this->registers.x;
}

uint16_t CPU6502::absoluteY(bool extraTick)
{
    uint16_t address = this->absolute();
    if (extraTick)
        this->tickIfNewPage(address, address + this->registers.y);
    return address + this->registers.x;
}

uint16_t CPU6502::indirect()
{
    uint16_t address = this->absolute();
    uint8_t lsb = *this->rom->read(address);
    uint16_t msbAddress = (address & 0xFF) == 0xFF ? address & 0xFF00 : address + 1;
    uint8_t msb = *this->rom->read(msbAddress);
    uint16_t returnAddress = (msb << 8) + lsb;
    return returnAddress;
}

uint16_t CPU6502::indirectX()
{
    uint16_t operand = this->absoluteX(false);
    uint8_t lsb = *this->rom->read(operand);
    uint8_t msb = *this->rom->read((operand + 1) % 256);
    uint16_t address = (msb << 8) + lsb;
    return address;
}

uint16_t CPU6502::indirectY(bool extraTick)
{
    uint16_t operand = this->immediate();
    uint8_t lsb = *this->rom->read(operand);
    uint8_t msb = *this->rom->read((operand + 1) % 256);
    uint16_t address = (msb << 8) + lsb;
    if (extraTick)
        this->tickIfNewPage(address, address + this->registers.y);
    return address + this->registers.y;
}

uint16_t CPU6502::addressing(AddressingMode mode, bool extraTick)
{
    switch(mode)
    {
        case AddressingMode::IMMEDIATE: return this->immediate();
        case AddressingMode::RELATIVE: return this->relative();
        case AddressingMode::ZERO_PAGE: return this->zeroPage();
        case AddressingMode::ZERO_PAGE_X: return this->zeroPageX();
        case AddressingMode::ZERO_PAGE_Y: return this->zeroPageY();
        case AddressingMode::ABSOLUTE: return this->absolute();
        case AddressingMode::ABSOLUTE_X: return this->absoluteX(extraTick);
        case AddressingMode::ABSOLUTE_Y: return this->absoluteY(extraTick);
        case AddressingMode::INDIRECT: return this->indirect();
        case AddressingMode::INDIRECT_X: return this->indirectX();
        case AddressingMode::INDIRECT_Y: return this->indirectY(extraTick);
        default: std::cout << "Invalid Addressing Mode" << std::endl; return 0;
    }
}

// Instructions

void CPU6502::adc(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = addressing(mode, true);
    bool carry = this->registers.statusRegister & StatusFlag::CARRY;
    uint16_t sum = data + this->registers.accumulator + carry;
    uint8_t overflow = (this->registers.accumulator ^ sum) & (data ^ sum) & 0x80;
    this->setStatusFlag(StatusFlag::CARRY, sum > 0xFF);
    this->registers.accumulator = sum;
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::OVERFLOW, overflow);
}

void CPU6502::AND(AddressingMode mode, int ticks)
{
    tick(ticks);
    auto data = addressing(mode);
    this->registers.accumulator &= data;
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
}

void CPU6502::bcs(AddressingMode mode, int ticks)
{
    tick(ticks);
    bool carry = this->registers.statusRegister & StatusFlag::CARRY;
    if (carry) 
    {
        uint16_t newProgramCounter = addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else 
    {
        this->registers.programCounter++;
    }
}

void CPU6502::jmp(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = addressing(mode);
    this->registers.programCounter = address - 1;

    switch(mode)
    {
        case AddressingMode::ABSOLUTE: break;
        case AddressingMode::INDIRECT: break;
    }
}

void CPU6502::jsr(AddressingMode mode, int ticks)
{
    tick(ticks);
    auto data = addressing(mode);
    uint8_t lsb = this->registers.programCounter & 0xFF;
    uint8_t msb = this->registers.programCounter >> 8;
    this->ram.write(this->registers.stackPointer--, msb);
    this->ram.write(this->registers.stackPointer--, lsb);
    this->registers.programCounter = data - 1;
}

void CPU6502::ldx(AddressingMode mode, int ticks)
{
    tick(ticks);
    auto data = addressing(mode, true);
    this->registers.x = data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.x & 0x80);    
}

void CPU6502::nop(AddressingMode mode, int ticks)
{
    tick(ticks);
}

void CPU6502::sec(int ticks)
{
    tick(ticks);
    this->setStatusFlag(StatusFlag::CARRY, true);
}

void CPU6502::stx(AddressingMode mode, int ticks)
{
    tick(ticks);
    auto data = addressing(mode);
    this->ram.write(data, this->registers.x);
}

