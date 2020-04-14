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
    this->registers.statusRegister = STATUS_REGISTER_STARTUP_VAL;
}

void CPU6502::reset()
{
    this->startup();
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
    /*std::cout << "opcode: " << std::hex << std::setw(4) << std::setfill('0') << std::uppercase << static_cast<int>(instruction) << std::endl;
    std::cout << "a: " << std::setw(4) << static_cast<int>(this->registers.accumulator) << std::endl;
    std::cout << "x: " << std::setw(4) << static_cast<int>(this->registers.x) << std::endl;
    std::cout << "y: " << std::setw(4) << static_cast<int>(this->registers.y) << std::endl;
    std::cout << "pc: " << std::setw(4) << static_cast<int>(this->registers.programCounter) << std::endl;
    std::cout << "sp: " << std::setw(4) << static_cast<int>(this->registers.stackPointer) << std::endl;
    std::cout << "st: " << std::setw(4) << static_cast<int>(this->registers.statusRegister) << std::endl;
    std::cout << "cycle: " << std::dec << this->cycle << std::endl;*/
    std::cout << std::hex << std::setw(4) << std::setfill('0') << std::uppercase
              << static_cast<int>(this->registers.programCounter)
              << "\t" << std::setw(2) << static_cast<int>(instruction)
              << "\tA:" << std::setw(2) << static_cast<int>(this->registers.accumulator)
              << " X:" << static_cast<int>(this->registers.x)
              << " Y:" << static_cast<int>(this->registers.y)
              << " P:" << static_cast<int>(this->registers.statusRegister)
              << " SP:" << static_cast<int>(this->registers.stackPointer)
              << " PPU:" <<  std::dec << static_cast<int>(this->ppu)
              << " CYC:" << this->cycle << std::endl;
}

void CPU6502::executeInstruction(uint8_t instruction)
{
    this->printStatus(instruction);
    switch (instruction)
    {
        default: this->unknownInstruction(instruction); break;

        // ADC - Add with carry
        case 0x61: this->adc(AddressingMode::INDIRECT_X, 6); break;
        case 0x65: this->adc(AddressingMode::ZERO_PAGE, 3); break;
        case 0x69: this->adc(AddressingMode::IMMEDIATE, 2); break;
        case 0x6D: this->adc(AddressingMode::ABSOLUTE, 4); break;
        case 0x71: this->adc(AddressingMode::INDIRECT_Y, 5); break;
        case 0x75: this->adc(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x79: this->adc(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x7D: this->adc(AddressingMode::ABSOLUTE_X, 4); break;
        
        // AND
        case 0x21: this->AND(AddressingMode::INDIRECT_X,6); break;
        case 0x25: this->AND(AddressingMode::ZERO_PAGE, 3); break;
        case 0x29: this->AND(AddressingMode::IMMEDIATE, 2); break;
        case 0x2D: this->AND(AddressingMode::ABSOLUTE, 4); break;
        case 0x31: this->AND(AddressingMode::INDIRECT_Y, 5); break;
        case 0x35: this->AND(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x39: this->AND(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x3D: this->AND(AddressingMode::ABSOLUTE_X, 4); break;

        // ASL - Arithmetic Shift Left
        case 0x06: this->asl(AddressingMode::ZERO_PAGE, 5); break;
        case 0x0A: this->asl(AddressingMode::ACCUMULATOR, 2); break;
        case 0x0E: this->asl(AddressingMode::ABSOLUTE, 6); break;
        case 0x16: this->asl(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x1E: this->asl(AddressingMode::ABSOLUTE_X, 7); break;

        // Branching
        case 0x90: this->bcc(AddressingMode::RELATIVE, 2); break;    // BCC - Branch if carry clear
        case 0xB0: this->bcs(AddressingMode::RELATIVE, 2); break;    // BCS - Branch if carry set
        case 0xF0: this->beq(AddressingMode::RELATIVE, 2); break;    // BEQ - Branch if equal
        case 0x30: this->bmi(AddressingMode::RELATIVE, 2); break;    // BMI - Branch if minus
        case 0xD0: this->bne(AddressingMode::RELATIVE, 2); break;    // BNE - Branch if not equal
        case 0x10: this->bpl(AddressingMode::RELATIVE, 2); break;    // BPL - Branch if positive
        case 0x50: this->bvc(AddressingMode::RELATIVE, 2); break;    // BVC - Branch if overflow clear
        case 0x70: this->bvs(AddressingMode::RELATIVE, 2); break;    // BVS - Branch if overflow set

        // Bit test
        case 0x24: this->bit(AddressingMode::ZERO_PAGE, 3); break;
        case 0x2C: this->bit(AddressingMode::ABSOLUTE, 4); break;

        // Force Interrupt
        case 0x00: this->brk(AddressingMode::IMPLIED, 7); break;

        // Clear
        case 0x18: this->clc(AddressingMode::IMMEDIATE, 2); break;    // Clear carry flag
        case 0xD8: this->cld(AddressingMode::IMMEDIATE, 2); break;    // Clear decimal flag
        case 0x58: this->cli(AddressingMode::IMMEDIATE, 2); break;    // Clear interrupt disable
        case 0xB8: this->clv(AddressingMode::IMMEDIATE, 2); break;    // Clear overflow flag

        // CMP - Compare
        case 0xC1: this->cmp(AddressingMode::INDIRECT_X, 2); break;
        case 0xC5: this->cmp(AddressingMode::ZERO_PAGE, 3); break;
        case 0xC9: this->cmp(AddressingMode::IMMEDIATE, 2); break;
        case 0xCD: this->cmp(AddressingMode::ABSOLUTE, 4); break;
        case 0xD1: this->cmp(AddressingMode::INDIRECT_Y, 5); break;
        case 0xD5: this->cmp(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xD9: this->cmp(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0xDD: this->cmp(AddressingMode::ABSOLUTE_X, 4); break;

        // CPX - Compare X Register
        case 0xE0: this->cpx(AddressingMode::IMMEDIATE, 2); break;
        case 0xE4: this->cpx(AddressingMode::ZERO_PAGE, 3); break;
        case 0xEC: this->cpx(AddressingMode::ABSOLUTE, 4); break;

        // CPY - Compare Y Register
        case 0xC0: this->cpy(AddressingMode::IMMEDIATE, 2); break;
        case 0xC4: this->cpy(AddressingMode::ZERO_PAGE, 3); break;
        case 0xCC: this->cpy(AddressingMode::ABSOLUTE, 4); break;

        // DEC - Decrement Memory
        case 0xC6: this->dec(AddressingMode::ZERO_PAGE, 5); break;
        case 0xCE: this->dec(AddressingMode::ABSOLUTE, 6); break;
        case 0xD6: this->dec(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0xDE: this->dec(AddressingMode::ABSOLUTE_X, 7); break;

        // DEX - Decrement X Register
        case 0xCA: this->dex(AddressingMode::IMMEDIATE, 2); break;

        // DEY - Decrement Y Register
        case 0x88: this->dey(AddressingMode::IMMEDIATE, 2); break;

        // EOR - Exclusive OR
        case 0x41: this->eor(AddressingMode::INDIRECT_X, 2); break;
        case 0x45: this->eor(AddressingMode::ZERO_PAGE, 3); break;
        case 0x49: this->eor(AddressingMode::IMMEDIATE, 2); break;
        case 0x4D: this->eor(AddressingMode::ABSOLUTE, 4); break;
        case 0x51: this->eor(AddressingMode::INDIRECT_Y, 5); break;
        case 0x55: this->eor(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x59: this->eor(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x5D: this->eor(AddressingMode::ABSOLUTE_X, 4); break;

        // INC - Increment Memory
        case 0xE6: this->inc(AddressingMode::ZERO_PAGE, 5); break;
        case 0xEE: this->inc(AddressingMode::ABSOLUTE, 6); break;
        case 0xF6: this->inc(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0xFE: this->inc(AddressingMode::ABSOLUTE_X, 7); break;

        // INX - Incremement X Register
        case 0xE8: this->inx(AddressingMode::IMMEDIATE, 2); break;

        // INY - Increment Y Register
        case 0xC8: this->iny(AddressingMode::IMMEDIATE, 2); break;

        // JMP 
        case 0x4C: this->jmp(AddressingMode::ABSOLUTE, 3); break;
        case 0x6C: this->jmp(AddressingMode::INDIRECT, 5); break;

        // JSR - Jump to subroutine
        case 0x20: this->jsr(AddressingMode::ABSOLUTE, 6); break;

        // LDA - Load Accumulator
        case 0xA1: this->lda(AddressingMode::INDIRECT_X, 6); break;
        case 0xA5: this->lda(AddressingMode::ZERO_PAGE, 3); break;
        case 0xA9: this->lda(AddressingMode::IMMEDIATE, 2); break;
        case 0xAD: this->lda(AddressingMode::ABSOLUTE, 4); break;
        case 0xB1: this->lda(AddressingMode::INDIRECT_Y, 5); break;
        case 0xB5: this->lda(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xB9: this->lda(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0xBD: this->lda(AddressingMode::ABSOLUTE_X, 4); break;

        // LDX - Load X Register
        case 0xA2: this->ldx(AddressingMode::IMMEDIATE, 2); break;
        case 0xA6: this->ldx(AddressingMode::ZERO_PAGE, 3); break;
        case 0xAE: this->ldx(AddressingMode::ABSOLUTE, 4); break;
        case 0xB6: this->ldx(AddressingMode::ZERO_PAGE_Y, 4); break;
        case 0xBE: this->ldx(AddressingMode::ABSOLUTE_Y, 4); break;

        // LDY - Load Y Register
        case 0xA0: this->ldy(AddressingMode::IMMEDIATE, 2); break;
        case 0xA4: this->ldy(AddressingMode::ZERO_PAGE, 3); break;
        case 0xAC: this->ldy(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xB4: this->ldy(AddressingMode::ABSOLUTE, 4); break;
        case 0xBC: this->ldy(AddressingMode::ABSOLUTE_X, 4); break;

        // LSR - Logical Shift Right
        case 0x46: this->lsr(AddressingMode::ZERO_PAGE, 5); break;
        case 0x4A: this->lsr(AddressingMode::ACCUMULATOR, 2); break;
        case 0x4E: this->lsr(AddressingMode::ABSOLUTE, 6); break;
        case 0x56: this->lsr(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x5E: this->lsr(AddressingMode::ABSOLUTE_X, 7); break;

        // NOP
        case 0xEA: this->nop(AddressingMode::IMMEDIATE, 2); break;

        // ORA - Logical Inclusive OR
        case 0x01: this->ora(AddressingMode::INDIRECT_X, 6); break;
        case 0x05: this->ora(AddressingMode::ZERO_PAGE, 3); break;
        case 0x09: this->ora(AddressingMode::IMMEDIATE, 2); break;
        case 0x0D: this->ora(AddressingMode::ABSOLUTE, 4); break;
        case 0x11: this->ora(AddressingMode::INDIRECT_Y, 5); break;
        case 0x15: this->ora(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x19: this->ora(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x1D: this->ora(AddressingMode::ABSOLUTE_X, 4); break;

        // PHA - Push Accumulator
        case 0x48: this->pha(AddressingMode::IMMEDIATE, 3); break;

        // PHP - Push Processor Status
        case 0x08: this->php(AddressingMode::IMMEDIATE, 3); break;

        // PLA - Pull Accumulator
        case 0x68: this->pla(AddressingMode::IMMEDIATE, 4); break;

        // PLP - Pull Processor Status
        case 0x28: this->plp(AddressingMode::IMMEDIATE, 4); break;

        // ROL - Rotate Left
        case 0x26: this->rol(AddressingMode::ZERO_PAGE, 5); break;
        case 0x2A: this->rol(AddressingMode::ACCUMULATOR, 2); break;
        case 0x2E: this->rol(AddressingMode::ABSOLUTE, 6); break;
        case 0x36: this->rol(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x3E: this->rol(AddressingMode::ABSOLUTE_X, 7); break;

        // ROR - Rotate Right
        case 0x66: this->ror(AddressingMode::ZERO_PAGE, 5); break;
        case 0x6A: this->ror(AddressingMode::ACCUMULATOR, 2); break;
        case 0x6E: this->ror(AddressingMode::ABSOLUTE, 6); break;
        case 0x76: this->ror(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x7E: this->ror(AddressingMode::ABSOLUTE_X, 7); break;

        // RTI - Return from interrupt
        case 0x40: this->rti(AddressingMode::IMPLIED, 6); break;

        // RTS - Return from subroutine
        case 0x60: this->rts(AddressingMode::IMMEDIATE, 6); break;

        // SBC - Subtract with carry
        case 0xE1: this->sbc(AddressingMode::INDIRECT_X, 6); break;
        case 0xE5: this->sbc(AddressingMode::ZERO_PAGE, 3); break;
        case 0xE9: this->sbc(AddressingMode::IMMEDIATE, 2); break;
        case 0xED: this->sbc(AddressingMode::ABSOLUTE, 4); break;
        case 0xF1: this->sbc(AddressingMode::INDIRECT_Y, 5); break;
        case 0xF5: this->sbc(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xF9: this->sbc(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0xFD: this->sbc(AddressingMode::ABSOLUTE_X, 4); break;

        // SEC - Set Carry Flag
        case 0x38: this->sec(2); break;

        // SED - Set Decimal Flag
        case 0xF8: this->sed(2); break;

        // SEI - Set Interrupt Disable
        case 0x78: this->sei(2); break;

        // STA - Store Accumulator
        case 0x81: this->sta(AddressingMode::INDIRECT_X, 6); break;
        case 0x85: this->sta(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8D: this->sta(AddressingMode::ABSOLUTE, 4); break;
        case 0x91: this->sta(AddressingMode::INDIRECT_Y, 6); break;
        case 0x95: this->sta(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x99: this->sta(AddressingMode::ABSOLUTE_Y, 5); break;
        case 0x9D: this->sta(AddressingMode::ABSOLUTE_X, 5); break;

        //　STX - Store X Register
        case 0x86: this->stx(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8E: this->stx(AddressingMode::ABSOLUTE, 4); break;
        case 0x96: this->stx(AddressingMode::ZERO_PAGE_Y, 4); break;

        // STY - Store Y Register
        case 0x84: this->sty(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8C: this->sty(AddressingMode::ABSOLUTE, 4); break;
        case 0x94: this->sty(AddressingMode::ZERO_PAGE_Y, 4); break;

        // TAX - Transfer Accumulator to X
        case 0xAA: this->tax(AddressingMode::IMMEDIATE, 2); break;

        // TAY - Transfer Accumulator to Y
        case 0xA8: this->tay(AddressingMode::IMMEDIATE, 2); break;

        // TSX - Transfer Stack Pointer to X
        case 0xBA: this->tsx(AddressingMode::IMMEDIATE, 2); break;

        // TXA - Transfer X to Accumulator
        case 0x8A: this->txa(AddressingMode::IMMEDIATE, 2); break;

        // TXS - Transfer X to Stack Pointer
        case 0x9A: this->txs(AddressingMode::IMMEDIATE, 2); break;

        // TYA - Transfer Y to Accumulator
        case 0x98: this->tya(AddressingMode::IMMEDIATE, 2); break;
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
        this->tick();
    }
}

void CPU6502::tick(int times)
{
    for (int i = 0; i < times; i++) 
    {
        this->cycle++;
        this->ppu++;
        this->ppu++;
        this->ppu++;
        this->ppu %= 341;
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
    uint8_t lsb = *this->rom->read(++this->registers.programCounter);
    uint8_t msb = *this->rom->read(++this->registers.programCounter);
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
    this->tick(ticks);
    uint8_t data = this->addressing(mode, true);
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
    this->tick(ticks);
    auto data = this->addressing(mode, true);
    this->registers.accumulator &= data;
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
}

void CPU6502::asl(AddressingMode mode, int ticks)
{
    bool carry;
    bool zero;
    bool negative;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        carry = (this->registers.accumulator >> 7) & 1;
        this->registers.accumulator <<= 1;
        zero = this->registers.accumulator == 0;
        negative = this->registers.accumulator & 0x80;
        
    }
    else
    {
        uint16_t data = this->addressing(mode);
        carry = (data >> 7) & 1;
        data <<= 1;
        zero = data == 0;
        negative = data & 0x80;        
    }
    
    this->setStatusFlag(StatusFlag::CARRY, carry);
    this->setStatusFlag(StatusFlag::ZERO, zero);
    this->setStatusFlag(StatusFlag::NEGATIVE, negative);
}

void CPU6502::bcc(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (!(this->registers.statusRegister & StatusFlag::CARRY))
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else 
    {
        this->registers.programCounter++;
    }
}

void CPU6502::bcs(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (this->registers.statusRegister & StatusFlag::CARRY)
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else 
    {
        this->registers.programCounter++;
    }
}

void CPU6502::beq(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (this->registers.statusRegister & StatusFlag::ZERO)
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else 
    {
        this->registers.programCounter++;
    }
}

void CPU6502::bmi(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (this->registers.statusRegister & StatusFlag::NEGATIVE)
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else
    {
        this->registers.programCounter++;
    }    
}

void CPU6502::bne(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (!(this->registers.statusRegister & StatusFlag::ZERO))
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else
    {
        this->registers.programCounter++;
    }    
}

void CPU6502::bit(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t address = this->addressing(mode);
    uint16_t data = *this->ram.read(address);
    uint16_t result = this->registers.accumulator & data;
    this->setStatusFlag(StatusFlag::ZERO, result == 0);
    this->setStatusFlag(StatusFlag::OVERFLOW, (data >> 6) & 1);
    this->setStatusFlag(StatusFlag::NEGATIVE, (data >> 7) & 1);
}

void CPU6502::brk(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint8_t lsb = this->registers.programCounter & 0xFF;
    uint8_t msb = this->registers.programCounter >> 8;
    this->ram.write(this->registers.stackPointer--, msb);
    this->ram.write(this->registers.stackPointer--, lsb);
    // BRK sets Bit 4 when pushing status register
    // Bit 5 should always be set.
    this->ram.write(this->registers.stackPointer--, (this->registers.statusRegister | StatusFlag::BIT4 | StatusFlag::BIT5));

    uint8_t* irclsb = this->rom->read(0xFFFE);
    uint8_t* ircmsb = this->rom->read(0xFFFF);
    this->registers.programCounter = ((*ircmsb >> 8) + *irclsb) - 1;
}

void CPU6502::bpl(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (!(this->registers.statusRegister & StatusFlag::NEGATIVE))
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else
    {
        this->registers.programCounter++;
    }
    
}

void CPU6502::bvc(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    if (!(this->registers.statusRegister & StatusFlag::OVERFLOW))
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else
    {
        this->registers.programCounter++;
    }
}

void CPU6502::bvs(AddressingMode mode, int ticks)
{
    this->tick(ticks);    
    if (this->registers.statusRegister & StatusFlag::OVERFLOW)
    {
        this->tick();
        uint16_t newProgramCounter = this->addressing(mode);
        this->tickIfNewPage(this->registers.programCounter + 1, newProgramCounter + 1);
        this->registers.programCounter = newProgramCounter;
    }
    else 
    {
        this->registers.programCounter++;
    }
}

void CPU6502::clc(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::CARRY, false);
}

void CPU6502::cld(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::DECIMAL, false);
}

void CPU6502::cli(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::INTERRUPT_DISABLE, false);
}

void CPU6502::clv(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::OVERFLOW, false);
}

void CPU6502::cmp(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode, true);
    this->setStatusFlag(StatusFlag::CARRY, this->registers.accumulator >= data);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == data);
    this->setStatusFlag(StatusFlag::NEGATIVE, (this->registers.accumulator - data) & 0x80);   
}

void CPU6502::cpx(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode);
    this->setStatusFlag(StatusFlag::CARRY, this->registers.x >= data);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == data);
    this->setStatusFlag(StatusFlag::NEGATIVE, (this->registers.x - data) & 0x80); 
}

void CPU6502::cpy(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode);
    this->setStatusFlag(StatusFlag::CARRY, this->registers.y >= data);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.y == data);
    this->setStatusFlag(StatusFlag::NEGATIVE, (this->registers.y - data) & 0x80); 
}

void CPU6502::dec(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t address = this->addressing(mode, true);
    uint8_t* data = this->ram.read(address);
    *data--;
    this->setStatusFlag(StatusFlag::ZERO, *data == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, *data & 0x80);
}

void CPU6502::dex(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.x--;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.x & 0x80);
}

void CPU6502::dey(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.y--;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.y == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.y & 0x80);
}

void CPU6502::eor(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode, true);
    this->registers.accumulator ^= data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
}

void CPU6502::inx(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.x++;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.x & 0x80);
}

void CPU6502::inc(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t address = this->addressing(mode);
    uint8_t* data = this->ram.read(address);
    *data++;
    this->setStatusFlag(StatusFlag::ZERO, *data == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, *data & 0x80);
}

void CPU6502::iny(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.y++;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.y == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.y & 0x80);
}

void CPU6502::jmp(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t address = this->addressing(mode);
    this->registers.programCounter = address - 1;

    switch(mode)
    {
        case AddressingMode::ABSOLUTE: break;
        case AddressingMode::INDIRECT: break;
    }
}

void CPU6502::jsr(AddressingMode mode, int ticks)
{
    // If JSR doesn't decrease and RTS doesn't increase, it still works if you explicitly use
    // them together. However, sometimes games set an address manually and use RTS, expecting
    // it to increase. Need to decrease/increase in JSR and RTS to make sure it works as expected.
    this->tick(ticks);
    auto data = this->addressing(mode);
    uint16_t pc = this->registers.programCounter - 1;
    uint8_t lsb = pc & 0xFF;
    uint8_t msb = pc >> 8;
    this->ram.write(this->registers.stackPointer--, msb);
    this->ram.write(this->registers.stackPointer--, lsb);
    this->registers.programCounter = data - 1;
}

void CPU6502::lda(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    auto data = this->addressing(mode, true);
    this->registers.accumulator = data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
}

void CPU6502::ldx(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    auto data = this->addressing(mode, true);
    this->registers.x = data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.x & 0x80);    
}

void CPU6502::ldy(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode, true);
    this->registers.y = data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.y == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.y & 0x80);
}

void CPU6502::lsr(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    bool carry;
    bool zero;
    bool negative;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        carry = this->registers.accumulator & 1;
        this->registers.accumulator >>= 1;
        zero = this->registers.accumulator == 0;
        negative = this->registers.accumulator & 0x80;
    }
    else
    {
        uint16_t data = this->addressing(mode);
        carry = data & 1;
        data >>= 1;
        zero = data == 0;
        negative = data & 0x80;
    }
    
    this->setStatusFlag(StatusFlag::CARRY, carry);
    this->setStatusFlag(StatusFlag::ZERO, zero);
    this->setStatusFlag(StatusFlag::NEGATIVE, negative);

}

void CPU6502::nop(AddressingMode mode, int ticks)
{
    this->tick(ticks);
}

void CPU6502::ora(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode, true);
    this->registers.accumulator |= data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE,this->registers.accumulator & 0x80);
}

void CPU6502::pha(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->ram.write(this->registers.stackPointer--, this->registers.accumulator);
}

void CPU6502::php(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    // PHP sets Bit 4 when pushing the status register.
    // Bit 5 should always be set.
    this->ram.write(this->registers.stackPointer--, (this->registers.statusRegister | StatusFlag::BIT4 | StatusFlag::BIT5));
}

void CPU6502::pla(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint8_t data = *this->ram.read(++this->registers.stackPointer);
    this->registers.accumulator = data;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
}

void CPU6502::plp(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint8_t newStatus = *this->ram.read(++this->registers.stackPointer);
    // PLP ignores the BIT4 register when reading
    // Set BIT5 in case of pulling a non-status register stack value that doesn't have it set.
    newStatus &= ~StatusFlag::BIT4;
    newStatus |= StatusFlag::BIT5;
    this->registers.statusRegister = newStatus;
}

void CPU6502::rol(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    bool carry = this->registers.statusRegister & StatusFlag::CARRY;
    bool newCarry;
    bool zero;
    bool negative;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        newCarry = (this->registers.accumulator >> 7) & 1;
        this->registers.accumulator <<= 1;
        this->registers.accumulator |= carry;
        zero = this->registers.accumulator == 0;
        negative = this->registers.accumulator & 0x80;
    }
    else
    {
        uint16_t data = this->addressing(mode);
        newCarry = (data >> 7) & 1;
        data <<= 1;
        data |= carry;
        zero = data == 0;
        negative = data & 0x80;
    }
       
    this->setStatusFlag(StatusFlag::CARRY, newCarry);
    this->setStatusFlag(StatusFlag::ZERO, zero);
    this->setStatusFlag(StatusFlag::NEGATIVE, negative);
}

void CPU6502::ror(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    bool carry;
    bool zero;
    bool negative;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        carry = this->registers.accumulator & StatusFlag::CARRY;
        this->registers.accumulator >>= 1;
        this->registers.accumulator |= (carry << 7);
        zero = this->registers.accumulator == 0;
        negative = this->registers.accumulator & 0x80;
    }
    else
    {
        uint16_t data = this->addressing(mode);
        carry = data & StatusFlag::CARRY;
        data >>= 1;
        data |= (carry << 7);
        zero = data == 0;
        negative = data & 0x80;
    }
    this->setStatusFlag(StatusFlag::CARRY, carry);
    this->setStatusFlag(StatusFlag::ZERO, zero);
    this->setStatusFlag(StatusFlag::NEGATIVE, negative);
}

void CPU6502::rti(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint8_t status = *this->ram.read(++this->registers.stackPointer);
    uint8_t lsb = *this->ram.read(++this->registers.stackPointer);
    uint8_t msb = *this->ram.read(++this->registers.stackPointer);

    this->registers.statusRegister = status;
    this->registers.programCounter = ((msb << 8) + lsb) + 2;
}

void CPU6502::rts(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint8_t lsb = *this->ram.read(++this->registers.stackPointer);
    uint8_t msb = *this->ram.read(++this->registers.stackPointer);
    uint16_t newProgramCounter = ((msb << 8) + lsb) + 1;
    this->registers.programCounter = newProgramCounter;
}

void CPU6502::sbc(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = (this->addressing(mode, true) ^ 0xFF);
    bool carry = this->registers.statusRegister & StatusFlag::CARRY;
    uint16_t sum = data + this->registers.accumulator + carry;
    uint8_t overflow = (this->registers.accumulator ^ sum) & (data ^ sum) & 0x80;
    this->setStatusFlag(StatusFlag::CARRY, sum > 0xFF);
    this->registers.accumulator = sum;
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::OVERFLOW, overflow);
}

void CPU6502::sec(int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::CARRY, true);
}

void CPU6502::sed(int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::DECIMAL, true);
}

void CPU6502::sei(int ticks)
{
    this->tick(ticks);
    this->setStatusFlag(StatusFlag::INTERRUPT_DISABLE, true);
}

void CPU6502::sta(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    auto address = this->addressing(mode);
    this->ram.write(address, this->registers.accumulator);
}

void CPU6502::stx(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    auto data = this->addressing(mode);
    this->ram.write(data, this->registers.x);
}

void CPU6502::sty(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    uint16_t data = this->addressing(mode);
    this->ram.write(data, this->registers.y);
}

void CPU6502::tax(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.x = this->registers.accumulator;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.x & 0x80);
}

void CPU6502::tay(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.y = this->registers.accumulator;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.y == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.y & 0x80);
}

void CPU6502::tsx(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.x = this->registers.stackPointer;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.x == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.x & 0x80);
}

void CPU6502::txa(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.accumulator = this->registers.x;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
}

void CPU6502::txs(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.stackPointer = this->registers.x;
}

void CPU6502::tya(AddressingMode mode, int ticks)
{
    this->tick(ticks);
    this->registers.accumulator = this->registers.y;
    this->setStatusFlag(StatusFlag::ZERO, this->registers.accumulator == 0);
    this->setStatusFlag(StatusFlag::NEGATIVE, this->registers.accumulator & 0x80);
}
