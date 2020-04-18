#pragma once

class Engine;

#include <memory>
#include "engine.h"

#include "rom.h"
#include "ram.h"

#define PROGRAM_COUNTER_STARTUP_VAL 0xC000
#define STACK_POINTER_STARTUP_VAL 0xFD
#define STATUS_REGISTER_STARTUP_VAL 0x24

enum StatusFlag
{
    CARRY = 1 << 0,
    ZERO = 1 << 1,
    INTERRUPT_DISABLE = 1 << 2,
    DECIMAL = 1 << 3,
    BIT4 = 1 << 4,
    BIT5 = 1 << 5,
    OVERFLOW = 1 << 6,
    NEGATIVE = 1 << 7
};

enum AddressingMode
{
    IMMEDIATE,
    RELATIVE,
    ZERO_PAGE,
    ZERO_PAGE_X,
    ZERO_PAGE_Y,
    ABSOLUTE,
    ABSOLUTE_X,
    ABSOLUTE_Y,
    INDIRECT,
    INDIRECT_X,
    INDIRECT_Y,
    ACCUMULATOR,
    IMPLIED
};

struct Registers
{
    uint8_t accumulator;
    uint8_t x;
    uint8_t y;
    uint16_t programCounter;
    uint8_t stackPointer;
    uint8_t statusRegister;
};

class CPU6502 
{
private:
    Registers m_registers;
    int m_cycle = 7;
    int m_ppu = 0;
    Engine& m_engine;

    void unofficialNop(uint8_t instruction);
    void unknownInstruction(uint8_t instruction);
    void tickIfNewPage(uint16_t programCounter, uint16_t newProgramCounter);
    void tick(int times = 1);
    void printStatus(uint8_t instruction);
    uint8_t getInstruction();
    void executeInstruction(uint8_t address);

    // addressing modes
    uint16_t immediate();
    uint16_t relative();
    uint16_t zeroPage();
    uint16_t zeroPageX();
    uint16_t zeroPageY();
    uint16_t absolute();
    uint16_t absoluteX(bool extraTick);
    uint16_t absoluteY(bool extraTick);
    uint16_t indirect();
    uint16_t indirectX();
    uint16_t indirectY(bool extraTick);

    uint16_t getAddress(AddressingMode mode, bool extraTick = false);

    // instructions
    void adc(AddressingMode mode, int ticks);
    void AND(AddressingMode mode, int ticks);
    void asl(AddressingMode mode, int ticks);
    void bcc(AddressingMode mode, int ticks);
    void bcs(AddressingMode mode, int ticks);
    void beq(AddressingMode mode, int ticks);
    void bit(AddressingMode mode, int ticks);
    void bmi(AddressingMode mode, int ticks);
    void bne(AddressingMode mode, int ticks);
    void bpl(AddressingMode mode, int ticks);
    void brk(AddressingMode mode, int ticks);
    void bvc(AddressingMode mode, int ticks);
    void bvs(AddressingMode mode, int ticks);
    void clc(AddressingMode mode, int ticks);
    void cld(AddressingMode mode, int ticks);
    void cli(AddressingMode mode, int ticks);
    void clv(AddressingMode mode, int ticks);
    void cmp(AddressingMode mode, int ticks);
    void cpx(AddressingMode mode, int ticks);
    void cpy(AddressingMode mode, int ticks);
    void dec(AddressingMode mode, int ticks);
    void dex(AddressingMode mode, int ticks);
    void dey(AddressingMode mode, int ticks);
    void eor(AddressingMode mode, int ticks);
    void inc(AddressingMode mode, int ticks);
    void inx(AddressingMode mode, int ticks);
    void iny(AddressingMode mode, int ticks);
    void jmp(AddressingMode mode, int ticks);
    void jsr(AddressingMode mode, int ticks);
    void lda(AddressingMode mode, int ticks);
    void ldx(AddressingMode mode, int ticks);
    void ldy(AddressingMode mode, int ticks);
    void lsr(AddressingMode mode, int ticks);
    void nop(AddressingMode mode, int ticks);
    void ora(AddressingMode mode, int ticks);
    void pha(AddressingMode mode, int ticks);
    void phr(AddressingMode mode, int ticks);
    void php(AddressingMode mode, int ticks);
    void pla(AddressingMode mode, int ticks);
    void plp(AddressingMode mode, int ticks);
    void rol(AddressingMode mode, int ticks);
    void ror(AddressingMode mode, int ticks);
    void rti(AddressingMode mode, int ticks);
    void rts(AddressingMode mode, int ticks);
    void sbc(AddressingMode mode, int ticks);
    void sec(int ticks);
    void sed(int ticks);
    void sei(int ticks);
    void sta(AddressingMode mode, int ticks);
    void stx(AddressingMode mode, int ticks);
    void sty(AddressingMode mode, int ticks);
    void tax(AddressingMode mode, int ticks);
    void tay(AddressingMode mode, int ticks);
    void tsx(AddressingMode mode, int ticks);
    void txa(AddressingMode mode, int ticks);
    void txs(AddressingMode mode, int ticks);
    void tya(AddressingMode mode, int ticks);

public:
    void startup();
    void reset();
    void step();
    CPU6502(Engine& engine);
    ~CPU6502();
    
    void setStatusFlag(StatusFlag flag, bool enabled);
};