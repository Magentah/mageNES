#include "cpu.h"

#include <iostream>
#include <iomanip>

CPU6502::CPU6502(Engine& engine)
    :m_engine(engine)
    {
    }

CPU6502::~CPU6502() {}

void CPU6502::startup(bool enablePrint)
{
    m_registers.x = 0;
    m_registers.y = 0;
    m_registers.accumulator = 0;
    m_registers.programCounter = PROGRAM_COUNTER_STARTUP_VAL;
    m_registers.stackPointer = STACK_POINTER_STARTUP_VAL;
    m_registers.statusRegister = STATUS_REGISTER_STARTUP_VAL;

    m_enablePrint = enablePrint;
}

void CPU6502::reset()
{
    startup(m_enablePrint);
}

uint8_t CPU6502::getInstruction()
{
    uint8_t instruction = m_engine.read(m_registers.programCounter);
    return instruction;
}

void CPU6502::unofficialNop(uint8_t instruction)
{
    // Unofficial NOPs use addressing modes. Need to handle this to get the correct
    // bytes, so program counter stays correct.
    switch(instruction)
    {
    // Zero Page
    case 0x04: case 0x44: case 0x64: 
        zeroPage();
        tick(3);
        break;
        // Zero Page X
    case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
        zeroPageX();
        tick(4);
        break;
    // Absolute        
    case 0x0C:
        absolute();
        tick(4);
        break;
    // Absolute X
    // 0x9C is SAY / SHY. Unofficial opcode but does something.
    case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0x9C: case 0xDC: case 0xFC:
        absoluteX(true);
        tick(4);
        break;
    // Absolute Y
    case 0x9E:
        absoluteY(false);
        tick(6);
        break;
    // Implied
    case 0x82: case 0x89: case 0xC2: case 0xE2: case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDa: case 0xFA:
        tick(2);
        break;
    // Immediate
    case 0x80:
        this->immediate();
        tick(2);
        break;
    // Stop / halt instructions.. Use unknown for now.
    case 0x02: case 0x12: case 0x22: case 0x32: case 0x42: case 0x52: case 0x62: case 0x72: case 0x92: case 0xB2: 
    case 0xD2: case 0xF2:
    default:
        unknownInstruction(instruction);
        tick();
        break;
    }
}

void CPU6502::unknownInstruction(uint8_t instruction) 
{
    std::cout << "Unknown opcode: " << std::hex << static_cast<int>(instruction) << std::endl;
    m_registers.programCounter++;
}

void CPU6502::printStatus(uint8_t instruction)
{
    if (!m_enablePrint)
        return;
    /*std::cout << "opcode: " << std::hex << std::setw(4) << std::setfill('0') << std::uppercase << static_cast<int>(instruction) << std::endl;
    std::cout << "a: " << std::setw(4) << static_cast<int>(m_registers.accumulator) << std::endl;
    std::cout << "x: " << std::setw(4) << static_cast<int>(m_registers.x) << std::endl;
    std::cout << "y: " << std::setw(4) << static_cast<int>(m_registers.y) << std::endl;
    std::cout << "pc: " << std::setw(4) << static_cast<int>(m_registers.programCounter) << std::endl;
    std::cout << "sp: " << std::setw(4) << static_cast<int>(m_registers.stackPointer) << std::endl;
    std::cout << "st: " << std::setw(4) << static_cast<int>(m_registers.statusRegister) << std::endl;
    std::cout << "cycle: " << std::dec << cycle << std::endl;*/
    std::cout << std::hex << std::setw(4) << std::setfill('0') << std::uppercase
              << static_cast<int>(m_registers.programCounter - 1)
              << "\t" << std::setw(2) << static_cast<int>(instruction)
              << "\tA:" << std::setw(2) << static_cast<int>(m_registers.accumulator)
              << " X:" << static_cast<int>(m_registers.x)
              << " Y:" << static_cast<int>(m_registers.y)
              << " P:" << static_cast<int>(m_registers.statusRegister)
              << " SP:" << static_cast<int>(m_registers.stackPointer)
              << " PPU:" <<  std::dec << static_cast<int>(m_ppu)
              << " CYC:" << m_cycle << std::endl;
}

void CPU6502::executeInstruction(uint8_t instruction)
{
    printStatus(instruction);
    switch (instruction)
    {
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
        case 0x06: asl(AddressingMode::ZERO_PAGE, 5); break;
        case 0x0A: asl(AddressingMode::ACCUMULATOR, 2); break;
        case 0x0E: asl(AddressingMode::ABSOLUTE, 6); break;
        case 0x16: asl(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x1E: asl(AddressingMode::ABSOLUTE_X, 7); break;

        // Branching
        case 0x90: bcc(AddressingMode::RELATIVE, 2); break;    // BCC - Branch if carry clear
        case 0xB0: bcs(AddressingMode::RELATIVE, 2); break;    // BCS - Branch if carry set
        case 0xF0: beq(AddressingMode::RELATIVE, 2); break;    // BEQ - Branch if equal
        case 0x30: bmi(AddressingMode::RELATIVE, 2); break;    // BMI - Branch if minus
        case 0xD0: bne(AddressingMode::RELATIVE, 2); break;    // BNE - Branch if not equal
        case 0x10: bpl(AddressingMode::RELATIVE, 2); break;    // BPL - Branch if positive
        case 0x50: bvc(AddressingMode::RELATIVE, 2); break;    // BVC - Branch if overflow clear
        case 0x70: bvs(AddressingMode::RELATIVE, 2); break;    // BVS - Branch if overflow set

        // Bit test
        case 0x24: bit(AddressingMode::ZERO_PAGE, 3); break;
        case 0x2C: bit(AddressingMode::ABSOLUTE, 4); break;

        // Force Interrupt
        case 0x00: brk(AddressingMode::IMPLIED, 7); break;

        // Clear
        case 0x18: clc(AddressingMode::IMMEDIATE, 2); break;    // Clear carry flag
        case 0xD8: cld(AddressingMode::IMMEDIATE, 2); break;    // Clear decimal flag
        case 0x58: cli(AddressingMode::IMMEDIATE, 2); break;    // Clear interrupt disable
        case 0xB8: clv(AddressingMode::IMMEDIATE, 2); break;    // Clear overflow flag

        // CMP - Compare
        case 0xC1: cmp(AddressingMode::INDIRECT_X, 6); break;
        case 0xC5: cmp(AddressingMode::ZERO_PAGE, 3); break;
        case 0xC9: cmp(AddressingMode::IMMEDIATE, 2); break;
        case 0xCD: cmp(AddressingMode::ABSOLUTE, 4); break;
        case 0xD1: cmp(AddressingMode::INDIRECT_Y, 5); break;
        case 0xD5: cmp(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xD9: cmp(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0xDD: cmp(AddressingMode::ABSOLUTE_X, 4); break;

        // CPX - Compare X Register
        case 0xE0: cpx(AddressingMode::IMMEDIATE, 2); break;
        case 0xE4: cpx(AddressingMode::ZERO_PAGE, 3); break;
        case 0xEC: cpx(AddressingMode::ABSOLUTE, 4); break;

        // CPY - Compare Y Register
        case 0xC0: cpy(AddressingMode::IMMEDIATE, 2); break;
        case 0xC4: cpy(AddressingMode::ZERO_PAGE, 3); break;
        case 0xCC: cpy(AddressingMode::ABSOLUTE, 4); break;

        // DEC - Decrement Memory
        case 0xC6: dec(AddressingMode::ZERO_PAGE, 5); break;
        case 0xCE: dec(AddressingMode::ABSOLUTE, 6); break;
        case 0xD6: dec(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0xDE: dec(AddressingMode::ABSOLUTE_X, 7); break;

        // DEX - Decrement X Register
        case 0xCA: dex(AddressingMode::IMMEDIATE, 2); break;

        // DEY - Decrement Y Register
        case 0x88: dey(AddressingMode::IMMEDIATE, 2); break;

        // EOR - Exclusive OR
        case 0x41: eor(AddressingMode::INDIRECT_X, 6); break;
        case 0x45: eor(AddressingMode::ZERO_PAGE, 3); break;
        case 0x49: eor(AddressingMode::IMMEDIATE, 2); break;
        case 0x4D: eor(AddressingMode::ABSOLUTE, 4); break;
        case 0x51: eor(AddressingMode::INDIRECT_Y, 5); break;
        case 0x55: eor(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x59: eor(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x5D: eor(AddressingMode::ABSOLUTE_X, 4); break;

        // INC - Increment Memory
        case 0xE6: inc(AddressingMode::ZERO_PAGE, 5); break;
        case 0xEE: inc(AddressingMode::ABSOLUTE, 6); break;
        case 0xF6: inc(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0xFE: inc(AddressingMode::ABSOLUTE_X, 7); break;

        // INX - Incremement X Register
        case 0xE8: inx(AddressingMode::IMMEDIATE, 2); break;

        // INY - Increment Y Register
        case 0xC8: iny(AddressingMode::IMMEDIATE, 2); break;

        // JMP 
        case 0x4C: jmp(AddressingMode::ABSOLUTE, 3); break;
        case 0x6C: jmp(AddressingMode::INDIRECT, 5); break;

        // JSR - Jump to subroutine
        case 0x20: jsr(AddressingMode::ABSOLUTE, 6); break;

        // LDA - Load Accumulator
        case 0xA1: lda(AddressingMode::INDIRECT_X, 6); break;
        case 0xA5: lda(AddressingMode::ZERO_PAGE, 3); break;
        case 0xA9: lda(AddressingMode::IMMEDIATE, 2); break;
        case 0xAD: lda(AddressingMode::ABSOLUTE, 4); break;
        case 0xB1: lda(AddressingMode::INDIRECT_Y, 5); break;
        case 0xB5: lda(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xB9: lda(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0xBD: lda(AddressingMode::ABSOLUTE_X, 4); break;

        // LDX - Load X Register
        case 0xA2: ldx(AddressingMode::IMMEDIATE, 2); break;
        case 0xA6: ldx(AddressingMode::ZERO_PAGE, 3); break;
        case 0xAE: ldx(AddressingMode::ABSOLUTE, 4); break;
        case 0xB6: ldx(AddressingMode::ZERO_PAGE_Y, 4); break;
        case 0xBE: ldx(AddressingMode::ABSOLUTE_Y, 4); break;

        // LDY - Load Y Register
        case 0xA0: ldy(AddressingMode::IMMEDIATE, 2); break;
        case 0xA4: ldy(AddressingMode::ZERO_PAGE, 3); break;
        case 0xAC: ldy(AddressingMode::ABSOLUTE, 4); break;
        case 0xB4: ldy(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xBC: ldy(AddressingMode::ABSOLUTE_X, 4); break;

        // LSR - Logical Shift Right
        case 0x46: lsr(AddressingMode::ZERO_PAGE, 5); break;
        case 0x4A: lsr(AddressingMode::ACCUMULATOR, 2); break;
        case 0x4E: lsr(AddressingMode::ABSOLUTE, 6); break;
        case 0x56: lsr(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x5E: lsr(AddressingMode::ABSOLUTE_X, 7); break;

        // NOP
        case 0xEA: nop(AddressingMode::IMMEDIATE, 2); break;

        // ORA - Logical Inclusive OR
        case 0x01: ora(AddressingMode::INDIRECT_X, 6); break;
        case 0x05: ora(AddressingMode::ZERO_PAGE, 3); break;
        case 0x09: ora(AddressingMode::IMMEDIATE, 2); break;
        case 0x0D: ora(AddressingMode::ABSOLUTE, 4); break;
        case 0x11: ora(AddressingMode::INDIRECT_Y, 5); break;
        case 0x15: ora(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x19: ora(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0x1D: ora(AddressingMode::ABSOLUTE_X, 4); break;

        // PHA - Push Accumulator
        case 0x48: pha(AddressingMode::IMMEDIATE, 3); break;

        // PHP - Push Processor Status
        case 0x08: php(AddressingMode::IMMEDIATE, 3); break;

        // PLA - Pull Accumulator
        case 0x68: pla(AddressingMode::IMMEDIATE, 4); break;

        // PLP - Pull Processor Status
        case 0x28: plp(AddressingMode::IMMEDIATE, 4); break;

        // ROL - Rotate Left
        case 0x26: rol(AddressingMode::ZERO_PAGE, 5); break;
        case 0x2A: rol(AddressingMode::ACCUMULATOR, 2); break;
        case 0x2E: rol(AddressingMode::ABSOLUTE, 6); break;
        case 0x36: rol(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x3E: rol(AddressingMode::ABSOLUTE_X, 7); break;

        // ROR - Rotate Right
        case 0x66: ror(AddressingMode::ZERO_PAGE, 5); break;
        case 0x6A: ror(AddressingMode::ACCUMULATOR, 2); break;
        case 0x6E: ror(AddressingMode::ABSOLUTE, 6); break;
        case 0x76: ror(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x7E: ror(AddressingMode::ABSOLUTE_X, 7); break;

        // RTI - Return from interrupt
        case 0x40: rti(AddressingMode::IMPLIED, 6); break;

        // RTS - Return from subroutine
        case 0x60: rts(AddressingMode::IMMEDIATE, 6); break;

        // SBC - Subtract with carry
        case 0xE1: sbc(AddressingMode::INDIRECT_X, 6); break;
        case 0xE5: sbc(AddressingMode::ZERO_PAGE, 3); break;
        case 0xE9: sbc(AddressingMode::IMMEDIATE, 2); break;
        case 0xEB: sbc(AddressingMode::IMMEDIATE, 2); break;    // Duplicate opcode
        case 0xED: sbc(AddressingMode::ABSOLUTE, 4); break;
        case 0xF1: sbc(AddressingMode::INDIRECT_Y, 5); break;
        case 0xF5: sbc(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0xF9: sbc(AddressingMode::ABSOLUTE_Y, 4); break;
        case 0xFD: sbc(AddressingMode::ABSOLUTE_X, 4); break;

        // SEC - Set Carry Flag
        case 0x38: sec(2); break;

        // SED - Set Decimal Flag
        case 0xF8: sed(2); break;

        // SEI - Set Interrupt Disable
        case 0x78: sei(2); break;

        // STA - Store Accumulator
        case 0x81: sta(AddressingMode::INDIRECT_X, 6); break;
        case 0x85: sta(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8D: sta(AddressingMode::ABSOLUTE, 4); break;
        case 0x91: sta(AddressingMode::INDIRECT_Y, 6); break;
        case 0x95: sta(AddressingMode::ZERO_PAGE_X, 4); break;
        case 0x99: sta(AddressingMode::ABSOLUTE_Y, 5); break;
        case 0x9D: sta(AddressingMode::ABSOLUTE_X, 5); break;

        //　STX - Store X Register
        case 0x86: stx(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8E: stx(AddressingMode::ABSOLUTE, 4); break;
        case 0x96: stx(AddressingMode::ZERO_PAGE_Y, 4); break;

        // STY - Store Y Register
        case 0x84: sty(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8C: sty(AddressingMode::ABSOLUTE, 4); break;
        case 0x94: sty(AddressingMode::ZERO_PAGE_X, 4); break;

        // TAX - Transfer Accumulator to X
        case 0xAA: tax(AddressingMode::IMMEDIATE, 2); break;

        // TAY - Transfer Accumulator to Y
        case 0xA8: tay(AddressingMode::IMMEDIATE, 2); break;

        // TSX - Transfer Stack Pointer to X
        case 0xBA: tsx(AddressingMode::IMMEDIATE, 2); break;

        // TXA - Transfer X to Accumulator
        case 0x8A: txa(AddressingMode::IMMEDIATE, 2); break;

        // TXS - Transfer X to Stack Pointer
        case 0x9A: txs(AddressingMode::IMMEDIATE, 2); break;

        // TYA - Transfer Y to Accumulator
        case 0x98: tya(AddressingMode::IMMEDIATE, 2); break;

        // Unofficial Opcodes
        // ALR
        case 0x4B: alr(AddressingMode::IMMEDIATE, 2); break;

        // ANC
        case 0x0B: anc(AddressingMode::IMMEDIATE, 2); break;

        // ARR
        case 0x6B: arr(AddressingMode::IMMEDIATE, 2); break;

        // AXS
        case 0xCB: axs(AddressingMode::IMMEDIATE, 2); break;

        // LAX
        case 0xA3: lax(AddressingMode::INDIRECT_X, 6); break;
        case 0xA7: lax(AddressingMode::ZERO_PAGE, 3); break;
        case 0xAF: lax(AddressingMode::ABSOLUTE, 4); break;
        case 0xB3: lax(AddressingMode::INDIRECT_Y, 5); break;
        case 0xB7: lax(AddressingMode::ZERO_PAGE_Y, 4); break;
        case 0xBF: lax(AddressingMode::ABSOLUTE_Y, 4); break;

        // SAX
        case 0x83: sax(AddressingMode::INDIRECT_X, 6); break;
        case 0x87: sax(AddressingMode::ZERO_PAGE, 3); break;
        case 0x8F: sax(AddressingMode::ABSOLUTE, 4); break;
        case 0x97: sax(AddressingMode::ZERO_PAGE_Y, 4); break;

        // DCP -- DEC then CMP
        case 0xC3: dcp(AddressingMode::INDIRECT_X, 8); break;
        case 0xC7: dcp(AddressingMode::ZERO_PAGE, 5); break;
        case 0xCF: dcp(AddressingMode::ABSOLUTE, 6); break;
        case 0xD3: dcp(AddressingMode::INDIRECT_Y, 8); break;
        case 0xD7: dcp(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0xDB: dcp(AddressingMode::ABSOLUTE_Y, 7); break;
        case 0xDF: dcp(AddressingMode::ABSOLUTE_X, 7); break;

        // ISC -- INC then SBC
        case 0xE3: isc(AddressingMode::INDIRECT_X, 8); break; 
        case 0xE7: isc(AddressingMode::ZERO_PAGE, 5); break;
        case 0xEF: isc(AddressingMode::ABSOLUTE, 6); break;
        case 0xF3: isc(AddressingMode::INDIRECT_Y, 8); break;
        case 0xF7: isc(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0xFB: isc(AddressingMode::ABSOLUTE_Y, 7); break;
        case 0xFF: isc(AddressingMode::ABSOLUTE_X, 7); break;

        // RLA -- ROL then AND
        case 0x23: rla(AddressingMode::INDIRECT_X, 8); break; 
        case 0x27: rla(AddressingMode::ZERO_PAGE, 5); break;
        case 0x2F: rla(AddressingMode::ABSOLUTE, 6); break;
        case 0x33: rla(AddressingMode::INDIRECT_Y, 8); break;
        case 0x37: rla(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x3B: rla(AddressingMode::ABSOLUTE_Y, 7); break;
        case 0x3F: rla(AddressingMode::ABSOLUTE_X, 7); break;

        // RRA -- ROR then ADC
        case 0x63: rra(AddressingMode::INDIRECT_X, 8); break; 
        case 0x67: rra(AddressingMode::ZERO_PAGE, 5); break;
        case 0x6F: rra(AddressingMode::ABSOLUTE, 6); break;
        case 0x73: rra(AddressingMode::INDIRECT_Y, 8); break;
        case 0x77: rra(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x7B: rra(AddressingMode::ABSOLUTE_Y, 7); break;
        case 0x7F: rra(AddressingMode::ABSOLUTE_X, 7); break;

        // SLO -- ASL then ORA
        case 0x03: slo(AddressingMode::INDIRECT_X, 8); break; 
        case 0x07: slo(AddressingMode::ZERO_PAGE, 5); break;
        case 0x0F: slo(AddressingMode::ABSOLUTE, 6); break;
        case 0x13: slo(AddressingMode::INDIRECT_Y, 8); break;
        case 0x17: slo(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x1B: slo(AddressingMode::ABSOLUTE_Y, 7); break;
        case 0x1F: slo(AddressingMode::ABSOLUTE_X, 7); break;

        // SRE -- LSR then EOR
        case 0x43: sre(AddressingMode::INDIRECT_X, 8); break; 
        case 0x47: sre(AddressingMode::ZERO_PAGE, 5); break;
        case 0x4F: sre(AddressingMode::ABSOLUTE, 6); break;
        case 0x53: sre(AddressingMode::INDIRECT_Y, 8); break;
        case 0x57: sre(AddressingMode::ZERO_PAGE_X, 6); break;
        case 0x5B: sre(AddressingMode::ABSOLUTE_Y, 7); break;
        case 0x5F: sre(AddressingMode::ABSOLUTE_X, 7); break;

        default: unofficialNop(instruction); break;
    }
}

void CPU6502::step()
{
    uint8_t instruction = getInstruction();
    m_registers.programCounter++;
    executeInstruction(instruction);
}

void CPU6502::setStatusFlag(StatusFlag flag, bool enabled)
{
    if (enabled)
        m_registers.statusRegister |= flag;
    else 
        m_registers.statusRegister &= ~flag;
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
        m_cycle++;
        m_ppu++;
        m_ppu++;
        m_ppu++;
        m_ppu %= 341;
        // 3x ppu ticks
    }
}

// Addressing Modes
uint16_t CPU6502::immediate()
{
    return m_registers.programCounter++;
}

uint16_t CPU6502::relative()
{
    // needs to be signed byte as the relative offset can be negative (-128 to +127)
    int8_t offset = m_engine.read(m_registers.programCounter++);
    return m_registers.programCounter + offset;
}

uint16_t CPU6502::zeroPage()
{
    uint8_t address = m_engine.read(m_registers.programCounter++);
    return address % 256;
}

uint16_t CPU6502::zeroPageX()
{
    uint8_t address = m_engine.read(m_registers.programCounter++);
    return (address + m_registers.x) % 256;
}

uint16_t CPU6502::zeroPageY()
{
    uint8_t address = m_engine.read(m_registers.programCounter++);
    return (address + m_registers.y) % 256;
}

uint16_t CPU6502::absolute()
{
    uint8_t lsb = m_engine.read(m_registers.programCounter++);
    uint8_t msb = m_engine.read(m_registers.programCounter++);
    uint16_t address = (msb << 8) + lsb;
    return address;
}

uint16_t CPU6502::absoluteX(bool extraTick)
{
    uint16_t address = absolute();
    if (extraTick)
        tickIfNewPage(address, address + m_registers.x);
    return address + m_registers.x;
}

uint16_t CPU6502::absoluteY(bool extraTick)
{
    uint16_t address = absolute();
    if (extraTick)
        tickIfNewPage(address, address + m_registers.y);
    return address + m_registers.y;
}

uint16_t CPU6502::indirect()
{
    uint16_t address = absolute();
    uint8_t lsb = m_engine.read(address);
    uint16_t msbAddress = (address & 0xFF) == 0xFF ? address & 0xFF00 : address + 1;
    uint8_t msb = m_engine.read(msbAddress);
    uint16_t returnAddress = (msb << 8) + lsb;
    return returnAddress;
}

uint16_t CPU6502::indirectX()
{
    uint16_t operand = (m_engine.read(m_registers.programCounter++) + m_registers.x) % 256;
    uint8_t lsb = m_engine.read(operand);
    uint8_t msb = m_engine.read((operand + 1) % 256);
    uint16_t address = (msb << 8) + lsb;
    return address;
}

uint16_t CPU6502::indirectY(bool extraTick)
{
    uint16_t operand = m_engine.read(m_registers.programCounter++);
    uint8_t lsb = m_engine.read(operand);
    uint8_t msb = m_engine.read((operand + 1) % 256);
    uint16_t address = (msb << 8) + lsb;
    if (extraTick)
        tickIfNewPage(address, address + m_registers.y);
    return address + m_registers.y;
}

uint16_t CPU6502::getAddress(AddressingMode mode, bool extraTick)
{
    switch(mode)
    {
        case AddressingMode::IMMEDIATE: return immediate();
        case AddressingMode::RELATIVE: return relative();
        case AddressingMode::ZERO_PAGE: return zeroPage();
        case AddressingMode::ZERO_PAGE_X: return zeroPageX();
        case AddressingMode::ZERO_PAGE_Y: return zeroPageY();
        case AddressingMode::ABSOLUTE: return absolute();
        case AddressingMode::ABSOLUTE_X: return absoluteX(extraTick);
        case AddressingMode::ABSOLUTE_Y: return absoluteY(extraTick);
        case AddressingMode::INDIRECT: return indirect();
        case AddressingMode::INDIRECT_X: return indirectX();
        case AddressingMode::INDIRECT_Y: return indirectY(extraTick);
        default: std::cout << "Invalid Addressing Mode" << std::endl; return 0;
    }
}

const uint8_t& CPU6502::getDataWithMode(AddressingMode mode, bool extraTick)
{
    uint16_t address = getAddress(mode, extraTick);
    return m_engine.read(address);
}

// Instructions

void CPU6502::adc(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    bool carry = m_registers.statusRegister & StatusFlag::CARRY;
    uint16_t sum = data + m_registers.accumulator + carry;
    uint8_t overflow = (m_registers.accumulator ^ sum) & (data ^ sum) & 0x80;
    setStatusFlag(StatusFlag::CARRY, sum > 0xFF);
    m_registers.accumulator = (uint8_t)sum;
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::OFLOW, overflow);
}

void CPU6502::AND(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.accumulator &= data;
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
}

void CPU6502::asl(AddressingMode mode, int ticks)
{
    tick(ticks);
    bool carry;
    bool zero;
    bool negative;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        carry = (m_registers.accumulator >> 7) & 1;
        m_registers.accumulator <<= 1;
        zero = m_registers.accumulator == 0;
        negative = m_registers.accumulator & 0x80;
        
    }
    else
    {
        uint16_t address = getAddress(mode);
        uint8_t data = m_engine.read(address);
        carry = (data >> 7) & 1;
        data <<= 1;
        zero = data == 0;
        negative = data & 0x80;
        m_engine.write(address, data);
    }
    
    setStatusFlag(StatusFlag::CARRY, carry);
    setStatusFlag(StatusFlag::ZERO, zero);
    setStatusFlag(StatusFlag::NEGATIVE, negative);
}

void CPU6502::bcc(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (!(m_registers.statusRegister & StatusFlag::CARRY))
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else 
    {
        m_registers.programCounter++;
    }
}

void CPU6502::bcs(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (m_registers.statusRegister & StatusFlag::CARRY)
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else 
    {
        m_registers.programCounter++;
    }
}

void CPU6502::beq(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (m_registers.statusRegister & StatusFlag::ZERO)
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else 
    {
        m_registers.programCounter++;
    }
}

void CPU6502::bmi(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (m_registers.statusRegister & StatusFlag::NEGATIVE)
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else
    {
        m_registers.programCounter++;
    }    
}

void CPU6502::bne(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (!(m_registers.statusRegister & StatusFlag::ZERO))
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else
    {
        m_registers.programCounter++;
    }    
}

void CPU6502::bit(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode);
    uint8_t result = m_registers.accumulator & data;
    setStatusFlag(StatusFlag::ZERO, result == 0);
    setStatusFlag(StatusFlag::OFLOW, (data >> 6) & 1);
    setStatusFlag(StatusFlag::NEGATIVE, (data >> 7) & 1);
}

void CPU6502::brk(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t lsb = m_registers.programCounter & 0xFF;
    uint8_t msb = m_registers.programCounter >> 8;
    m_engine.pushStack(m_registers.stackPointer--, msb);
    m_engine.pushStack(m_registers.stackPointer--, lsb);
    // BRK sets Bit 4 when pushing status register
    // Bit 5 should always be set.
    m_engine.pushStack(m_registers.stackPointer--, (m_registers.statusRegister | StatusFlag::BIT4 | StatusFlag::BIT5));

    uint8_t irclsb = m_engine.read(0xFFFE);
    uint8_t ircmsb = m_engine.read(0xFFFF);
    m_registers.programCounter = (((uint16_t)ircmsb >> 8) + irclsb) - 1;
}

void CPU6502::bpl(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (!(m_registers.statusRegister & StatusFlag::NEGATIVE))
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else
    {
        m_registers.programCounter++;
    }
    
}

void CPU6502::bvc(AddressingMode mode, int ticks)
{
    tick(ticks);
    if (!(m_registers.statusRegister & StatusFlag::OFLOW))
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else
    {
        m_registers.programCounter++;
    }
}

void CPU6502::bvs(AddressingMode mode, int ticks)
{
    tick(ticks);    
    if (m_registers.statusRegister & StatusFlag::OFLOW)
    {
        tick();
        uint16_t newProgramCounter = getAddress(mode);
        tickIfNewPage(m_registers.programCounter, newProgramCounter);
        m_registers.programCounter = newProgramCounter;
    }
    else 
    {
        m_registers.programCounter++;
    }
}

void CPU6502::clc(AddressingMode mode, int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::CARRY, false);
}

void CPU6502::cld(AddressingMode mode, int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::DECIMAL, false);
}

void CPU6502::cli(AddressingMode mode, int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::INTERRUPT_DISABLE, false);
}

void CPU6502::clv(AddressingMode mode, int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::OFLOW, false);
}

void CPU6502::cmp(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    setStatusFlag(StatusFlag::CARRY, m_registers.accumulator >= data);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == data);
    setStatusFlag(StatusFlag::NEGATIVE, (m_registers.accumulator - data) & 0x80);   
}

void CPU6502::cpx(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode);
    setStatusFlag(StatusFlag::CARRY, m_registers.x >= data);
    setStatusFlag(StatusFlag::ZERO, m_registers.x == data);
    setStatusFlag(StatusFlag::NEGATIVE, (m_registers.x - data) & 0x80); 
}

void CPU6502::cpy(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode);
    setStatusFlag(StatusFlag::CARRY, m_registers.y >= data);
    setStatusFlag(StatusFlag::ZERO, m_registers.y == data);
    setStatusFlag(StatusFlag::NEGATIVE, (m_registers.y - data) & 0x80); 
}

void CPU6502::dec(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode, true);
    uint8_t data = m_engine.read(address);
    data--;
    setStatusFlag(StatusFlag::ZERO, data == 0);
    setStatusFlag(StatusFlag::NEGATIVE, data & 0x80);
    m_engine.write(address, data);
}

void CPU6502::dex(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.x--;
    setStatusFlag(StatusFlag::ZERO, m_registers.x == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.x & 0x80);
}

void CPU6502::dey(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.y--;
    setStatusFlag(StatusFlag::ZERO, m_registers.y == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.y & 0x80);
}

void CPU6502::eor(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.accumulator ^= data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

void CPU6502::inx(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.x++;
    setStatusFlag(StatusFlag::ZERO, m_registers.x == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.x & 0x80);
}

void CPU6502::inc(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode);
    uint8_t data = m_engine.read(address);
    data++;
    setStatusFlag(StatusFlag::ZERO, data == 0);
    setStatusFlag(StatusFlag::NEGATIVE, data & 0x80);
    m_engine.write(address, data);
}

void CPU6502::iny(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.y++;
    setStatusFlag(StatusFlag::ZERO, m_registers.y == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.y & 0x80);
}

void CPU6502::jmp(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode);
    m_registers.programCounter = address;
}

void CPU6502::jsr(AddressingMode mode, int ticks)
{
    // If JSR doesn't decrease and RTS doesn't increase, it still works if you explicitly use
    // them together. However, sometimes games set an address manually and use RTS, expecting
    // it to increase. Need to decrease/increase in JSR and RTS to make sure it works as expected.
    tick(ticks);
    uint16_t address = getAddress(mode);
    uint16_t pc = m_registers.programCounter - 1;
    uint8_t lsb = pc & 0xFF;
    uint8_t msb = pc >> 8;
    m_engine.pushStack(m_registers.stackPointer--, msb);
    m_engine.pushStack(m_registers.stackPointer--, lsb);
    m_registers.programCounter = address;
}

void CPU6502::lda(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.accumulator = data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

void CPU6502::ldx(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.x = data;
    setStatusFlag(StatusFlag::ZERO, m_registers.x == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.x & 0x80);    
}

void CPU6502::ldy(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.y = data;
    setStatusFlag(StatusFlag::ZERO, m_registers.y == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.y & 0x80);
}

void CPU6502::lsr(AddressingMode mode, int ticks)
{
    tick(ticks);
    bool carry;
    bool zero;
    bool negative;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        carry = m_registers.accumulator & 1;
        m_registers.accumulator >>= 1;
        zero = m_registers.accumulator == 0;
        negative = m_registers.accumulator & 0x80;
    }
    else
    {
        uint16_t address = getAddress(mode, true);
        uint8_t data = m_engine.read(address);
        carry = data & 1;
        data >>= 1;
        zero = data == 0;
        negative = data & 0x80;
        m_engine.write(address, data);
    }
    
    setStatusFlag(StatusFlag::CARRY, carry);
    setStatusFlag(StatusFlag::ZERO, zero);
    setStatusFlag(StatusFlag::NEGATIVE, negative);

}

void CPU6502::nop(AddressingMode mode, int ticks)
{
    tick(ticks);
}

void CPU6502::ora(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.accumulator |= data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE,m_registers.accumulator & 0x80);
}

void CPU6502::pha(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_engine.pushStack(m_registers.stackPointer--, m_registers.accumulator);
}

void CPU6502::php(AddressingMode mode, int ticks)
{
    tick(ticks);
    // PHP sets Bit 4 when pushing the status register.
    // Bit 5 should always be set.
    m_engine.pushStack(m_registers.stackPointer--, (m_registers.statusRegister | StatusFlag::BIT4 | StatusFlag::BIT5));
}

void CPU6502::pla(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = m_engine.popStack(++m_registers.stackPointer);
    m_registers.accumulator = data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

void CPU6502::plp(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t newStatus = m_engine.popStack(++m_registers.stackPointer);
    // PLP ignores the BIT4 register when reading
    // Set BIT5 in case of pulling a non-status register stack value that doesn't have it set.
    newStatus &= ~StatusFlag::BIT4;
    newStatus |= StatusFlag::BIT5;
    m_registers.statusRegister = newStatus;
}

void CPU6502::rol(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t currentCarry = m_registers.statusRegister & StatusFlag::CARRY;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        setStatusFlag(StatusFlag::CARRY, (m_registers.accumulator >> 7) & 1);
        m_registers.accumulator <<= 1;
        m_registers.accumulator |= currentCarry;
        setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
        setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    }
    else
    {
        uint16_t address = getAddress(mode, true);
        uint8_t data = m_engine.read(address);
        setStatusFlag(StatusFlag::CARRY, (data >> 7) & 1);
        data <<= 1;
        data |= currentCarry;
        setStatusFlag(StatusFlag::ZERO, data == 0);
        setStatusFlag(StatusFlag::NEGATIVE, data & 0x80);
        m_engine.write(address, data);
    }
}

void CPU6502::ror(AddressingMode mode, int ticks)
{
    tick(ticks);
    bool currentCarry = m_registers.statusRegister & StatusFlag::CARRY;
    if (mode == AddressingMode::ACCUMULATOR)
    {
        setStatusFlag(StatusFlag::CARRY, m_registers.accumulator & 1);
        m_registers.accumulator >>= 1;
        m_registers.accumulator |= (currentCarry << 7);
        setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
        setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    }
    else
    {
        uint16_t address = getAddress(mode, true);
        uint8_t data = m_engine.read(address);
        setStatusFlag(StatusFlag::CARRY, data & 1);
        data >>= 1;
        data |= (currentCarry << 7);
        setStatusFlag(StatusFlag::ZERO, data == 0);
        setStatusFlag(StatusFlag::NEGATIVE, data & 0x80);
        m_engine.write(address, data);
    }
}

void CPU6502::rti(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t status = m_engine.popStack(++m_registers.stackPointer);
    uint8_t lsb = m_engine.popStack(++m_registers.stackPointer);
    uint8_t msb = m_engine.popStack(++m_registers.stackPointer);

    status |= StatusFlag::BIT5;
    m_registers.statusRegister = status;
    m_registers.programCounter = ((msb << 8) + lsb);
}

void CPU6502::rts(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t lsb = m_engine.popStack(++m_registers.stackPointer);
    uint8_t msb = m_engine.popStack(++m_registers.stackPointer);
    uint16_t newProgramCounter = ((msb << 8) + lsb) + 1;
    m_registers.programCounter = newProgramCounter;
}

void CPU6502::sbc(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = (getDataWithMode(mode, true) ^ 0xFF);
    bool carry = m_registers.statusRegister & StatusFlag::CARRY;
    uint16_t sum = data + m_registers.accumulator + carry;
    uint8_t overflow = (m_registers.accumulator ^ sum) & (data ^ sum) & 0x80;
    setStatusFlag(StatusFlag::CARRY, sum > 0xFF);
    m_registers.accumulator = (uint8_t)sum;
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::OFLOW, overflow);
}

void CPU6502::sec(int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::CARRY, true);
}

void CPU6502::sed(int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::DECIMAL, true);
}

void CPU6502::sei(int ticks)
{
    tick(ticks);
    setStatusFlag(StatusFlag::INTERRUPT_DISABLE, true);
}

void CPU6502::sta(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode, false);
    m_engine.write(address, m_registers.accumulator);
}

void CPU6502::stx(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode, true);
    m_engine.write(address, m_registers.x);
}

void CPU6502::sty(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode, true);
    m_engine.write(address, m_registers.y);
}

void CPU6502::tax(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.x = m_registers.accumulator;
    setStatusFlag(StatusFlag::ZERO, m_registers.x == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.x & 0x80);
}

void CPU6502::tay(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.y = m_registers.accumulator;
    setStatusFlag(StatusFlag::ZERO, m_registers.y == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.y & 0x80);
}

void CPU6502::tsx(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.x = m_registers.stackPointer;
    setStatusFlag(StatusFlag::ZERO, m_registers.x == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.x & 0x80);
}

void CPU6502::txa(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.accumulator = m_registers.x;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

void CPU6502::txs(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.stackPointer = m_registers.x;
}

void CPU6502::tya(AddressingMode mode, int ticks)
{
    tick(ticks);
    m_registers.accumulator = m_registers.y;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

// Unofficial Opcodes
// AND #i, then LSR A.
void CPU6502::alr(AddressingMode mode, int ticks)
{
    tick(ticks);
    AND(mode, 0);
    lsr(AddressingMode::ACCUMULATOR, 0);
}

// AND #i, then copy N to C.
void CPU6502::anc(AddressingMode mode, int ticks)
{
    tick(ticks);
    AND(mode, 0);
    setStatusFlag(StatusFlag::CARRY, m_registers.statusRegister & StatusFlag::NEGATIVE);
}

// AND #i, then ROR A, except setting C to bit 6, and V to bit 6 XOR bit 5. N and Z are set as normal.
void CPU6502::arr(AddressingMode mode, int ticks)
{
    tick(ticks);
    AND(mode, 0);
    bool currentCarry = m_registers.statusRegister & StatusFlag::CARRY;
    setStatusFlag(StatusFlag::CARRY, (m_registers.accumulator >> 6) & 1);
    m_registers.accumulator >>= 1;
    m_registers.accumulator |= (currentCarry << 7);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::OFLOW, ((m_registers.accumulator >> 6) & 1) ^ ((m_registers.accumulator >> 5) & 1));
}

// A = A & X - value. Set NZC as normal.
void CPU6502::axs(AddressingMode mode, int ticks)
{
    uint8_t data = getDataWithMode(mode);
    m_registers.accumulator = (m_registers.accumulator & m_registers.x) - data;
    setStatusFlag(StatusFlag::CARRY, m_registers.accumulator > 0xFF);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

// LDA value, TAX.
void CPU6502::lax(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t data = getDataWithMode(mode, true);
    m_registers.accumulator = data;
    m_registers.x = data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}

// Store A AND X, doesn't affect flags.
void CPU6502::sax(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode);
    uint8_t data = m_registers.accumulator & m_registers.x;
    m_engine.write(address, data);
}

// DEC Value then CMP value
void CPU6502::dcp(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode);
    uint8_t data = m_engine.read(address);
    data--;
    m_engine.write(address, data);
    setStatusFlag(StatusFlag::CARRY, m_registers.accumulator >= data);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == data);
    setStatusFlag(StatusFlag::NEGATIVE, (m_registers.accumulator - data) & 0x80);   
}

// INC then SBC
void CPU6502::isc(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode);
    uint8_t data = m_engine.read(address);

    // INC
    data++;
    m_engine.write(address, data);

    // SBC
    data = (data ^ 0xFF);
    bool carry = m_registers.statusRegister & StatusFlag::CARRY;
    uint16_t sum = data + m_registers.accumulator + carry;
    uint8_t overflow = (m_registers.accumulator ^ sum) & (data ^ sum) & 0x80;    
    m_registers.accumulator = (uint8_t)sum;
    setStatusFlag(StatusFlag::CARRY, sum > 0xFF);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::OFLOW, overflow); 
}

// ROL value, then AND value.
void CPU6502::rla(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint8_t currentCarry = m_registers.statusRegister & StatusFlag::CARRY;
    uint16_t address = getAddress(mode);
    uint8_t data = m_engine.read(address);

    // ROL
    setStatusFlag(StatusFlag::CARRY, (data >> 7) & 1);
    data <<= 1;
    data |= currentCarry;
    m_engine.write(address, data);

    // AND
    m_registers.accumulator &= data;
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
}

// ROR value, then ADC value
void CPU6502::rra(AddressingMode mode, int ticks)
{
    tick(ticks);
    bool currentCarry = m_registers.statusRegister & StatusFlag::CARRY;
    uint16_t address = getAddress(mode);
    uint8_t data = m_engine.read(address);

    // ROR
    setStatusFlag(StatusFlag::CARRY, (data & 1) > 0);
    data >>= 1;
    data |= (currentCarry << 7);
    m_engine.write(address, data);

    // ADC
    bool carry = m_registers.statusRegister & StatusFlag::CARRY;
    uint16_t sum = data + m_registers.accumulator + carry;
    m_registers.accumulator += data + (m_registers.statusRegister & StatusFlag::CARRY);
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
    setStatusFlag(StatusFlag::CARRY, sum > 0xFF);
    setStatusFlag(StatusFlag::OFLOW, (m_registers.accumulator ^ sum) & (data ^ sum) & 0x80);
}

// ASL value, then ORA value.
void CPU6502::slo(AddressingMode mode, int ticks)
{
    tick(ticks);

    uint16_t address = getAddress(mode);
    uint8_t data = m_engine.read(address);

    // ASL
    setStatusFlag(StatusFlag::CARRY, (data >> 7) & 1);
    data <<= 1;
    m_engine.write(address, data);

    // ORA
    m_registers.accumulator |= data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE,m_registers.accumulator & 0x80);
}

// LSR value, then EOR value
void CPU6502::sre(AddressingMode mode, int ticks)
{
    tick(ticks);
    uint16_t address = getAddress(mode, false);
    uint8_t data = m_engine.read(address);

    // LSR
    setStatusFlag(StatusFlag::CARRY, data & 1);
    data >>= 1;
    m_engine.write(address, data);

    // EOR
    m_registers.accumulator ^= data;
    setStatusFlag(StatusFlag::ZERO, m_registers.accumulator == 0);
    setStatusFlag(StatusFlag::NEGATIVE, m_registers.accumulator & 0x80);
}