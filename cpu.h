#ifndef CPU_H_
#define CPU_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define NOP   0x00 
#define COPY  0x01 
#define COPYI 0x02 
#define IN    0x03 
#define INP   0x04 
#define OUT   0x05 
#define OUTI  0x06 
#define OUTP  0x07
#define CLR   0x08 
#define AND   0x09 
#define ANDI  0x0A 
#define OR    0x0B 
#define ORI   0x0C 
#define XOR   0x0D 
#define XORI  0x0E 
#define ADD   0x0F 

#define ADDI  0x10 
#define SUB   0x11 
#define SUBI  0x12 
#define MUL   0x13 
#define MULI  0x14 
#define DIV   0x15 
#define DIVI  0x16 
#define INC   0x17 
#define DEC   0x18 
#define POP   0x19 
#define PUSH  0x1A 
#define CALL  0x1B 
#define RET   0x1C 
#define RETI  0x1D 
#define CP    0x1E 
#define CPI   0x1F 

#define JMP   0x20 
#define JE    0x21 
#define JNE   0x22 
#define JGE   0x23 
#define JGT   0x24 
#define JLE   0x25 
#define JLT   0x26 
#define LSL   0x27 
#define LSR   0x28 
#define SEI   0x29 
#define CLI   0x2A 

#define I 5 /* Interrupt flag */
#define S 4 /* Signed flag */
#define N 3 /* Negative flag */
#define Z 2 /* Zero flag */
#define V 1 /* Overflow flag */
#define C 0 /* Carry flag */

#define DDRA  0x00 
#define PORTA 0x01 
#define PINA  0X02 
#define ICR   0x03 
#define IFR   0x04 
#define PCMSK 0x05

#define PCIEA  0x00 // Port A Pin Change Interrupt Enable bit.
#define PCIFA  0x00 // Port A Pin Change Interrupt Flag bit.

#define RESET_vect 0x00 
#define PCINTA_vect 0x02 

// Register.
#define R0   0x00
#define R1   0x01
#define R2   0x02
#define R3   0x03
#define R4   0x04
#define R5   0x05
#define R6   0x06
#define R7   0x07
#define R8   0x08
#define R9   0x09
#define R10  0x0A
#define R11  0x0B
#define R12  0x0C
#define R13  0x0D
#define R14  0x0E
#define R15  0x0F

#define R16  0x10
#define R17  0x11
#define R18  0x12
#define R19  0x13
#define R20  0x14
#define R21  0x15
#define R22  0x16
#define R23  0x17
#define R24  0x18
#define R25  0x19
#define R26  0x1A
#define R27  0x1B
#define R28  0x1C
#define R29  0x1D
#define R30  0x1E
#define R31  0x1F

#define XLOW R26
#define XHIGH R27
#define YLOW R28
#define YHIGH R29
#define ZLOW R30
#define ZHIGH R31
#define XREG XLOW
#define YREG YLOW
#define ZREG ZLOW

//Ports
#define PORTA0   0
#define PORTA1   1
#define PORTA2   2
#define PORTA3   3
#define PORTA4   4
#define PORTA5   5
#define PORTA6   6
#define PORTA7   7
#define PORTA8   8
#define PORTA9   9
#define PORTA10  10
#define PORTA11  11
#define PORTA12  12
#define PORTA13  13
#define PORTA14  14
#define PORTA15  15

#define PORTA16  16
#define PORTA17  17
#define PORTA18  18
#define PORTA19  19
#define PORTA20  20
#define PORTA21  21
#define PORTA22  22
#define PORTA23  23
#define PORTA24  24
#define PORTA25  25
#define PORTA26  26
#define PORTA27  27
#define PORTA28  28
#define PORTA29  29
#define PORTA30  30
#define PORTA31  31


#define CPU_REGISTER_ADDRESS_WIDTH 32 /* 32 CPU registers in control unit. */
#define CPU_REGISTER_DATA_WIDTH    8  /* 8 bit data width per CPU register. */
#define IO_REGISTER_DATA_WIDTH     8  /* 8 bit data width per I/O location. */

enum cpu_state
{
	CPU_STATE_FETCH,  /* Fetches next instruction from program memory. */
	CPU_STATE_DECODE, /* Decodes the fetched instruction. */
	CPU_STATE_EXECUTE /* Executes the decoded instruction. */
};


#endif /* CPU_H_ */