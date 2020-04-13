#pragma once

#include "rom.h"
#include "ram.h"

const uint16_t PROGRAM_COUNTER_STARTUP_VAL = 0xC000;
const uint8_t STACK_POINTER_STARTUP_VAL = 0xFD;

enum StatusFlag
{
    CARRY = 0x01,
    ZERO = 0x02,
    INTERRUPT_DISABLE = 0x04,
    DECIMAL = 0x08,
    OVERFLOW = 0x0F,
    NEGATIVE = 0x10
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
    INDIRECT_Y
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
    Registers registers;
    int cycle = 7;
    ROM* rom;
    RAM ram;

    void notImplemented(uint8_t instruction);
    void unknownInstruction(uint8_t instruction);
    void tickIfNewPage(uint16_t programCounter, uint16_t newProgramCounter);
    void tick(int times = 1);
    void printStatus(uint8_t instruction);

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

    uint16_t addressing(AddressingMode mode, bool extraTick = false);

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
    void sed(AddressingMode mode, int ticks);
    void sei(AddressingMode mode, int ticks);
    void sta(AddressingMode mode, int ticks);
    void stx(AddressingMode mode, int ticks);
    void sty(AddressingMode mode, int ticks);
    void tax(AddressingMode mode, int ticks);
    void tay(AddressingMode mode, int ticks);
    void tsx(AddressingMode mode, int ticks);
    void tsa(AddressingMode mode, int ticks);
    void txs(AddressingMode mode, int ticks);
    void tya(AddressingMode mode, int ticks);

public:
    void startup();
    void reset();
    void load(ROM& rom);
    uint8_t getInstruction();
    void executeInstruction(uint8_t address);
    void step();
    void shutdown();

    
    void setStatusFlag(StatusFlag flag, bool enabled);
};