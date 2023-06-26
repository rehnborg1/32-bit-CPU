/********************************************************************************
* program_memory.c: data handles assembly code which is sent to control unit 
*				    where it is executed.
********************************************************************************/
#include "program_memory.h"

/* Macro definitions */
#define ISR_PCINTA     4
#define button2_check  9
#define button3_check  14
#define ISR_PCINTA_end 18
#define main           19
#define setup          21

#define LED1_toggle    35
#define LED1_off       37
#define LED1_on        42

#define LED2_toggle    47
#define LED2_off       49
#define LED2_on        54

#define LED3_toggle    59
#define LED3_off       61
#define LED3_on        66

#define main_loop      71


#define LED1 PORTA8
#define LED2 PORTA9
#define LED3 PORTA10

#define BUTTON1 PORTA11
#define BUTTON2 PORTA12
#define BUTTON3 PORTA13

static bool program_memory_initialized = false;

/* data contains assembly instruction code */
uint64_t data[PROGRAM_MEMORY_ADDRESS_WIDTH];

/********************************************************************************
* assemble: Returns instruction assembled to machine code.
*
*           - op_code: OP code of the instruction.
*           - op1    : First operand (destination).
*           - op2    : Second operand (constant or read location).
********************************************************************************/
static inline uint64_t assemble(const uint16_t op_code, const uint16_t op1, const uint32_t op2);

void program_memory_write(void)
{
   if (program_memory_initialized) return;
   program_memory_initialized = true;

   /********************************************************************************
   * RESET_vect: Reset vector and start address for the program. 
   *             Jumps to main subroutine to start the program.
   ********************************************************************************/
   data[0] = assemble(JMP, main, 0x00);
   data[1] = assemble(NOP, 0x00, 0x00);
   
   /********************************************************************************
   * PCINTA_vect: Interrupt vector for PCI-interrupt for I/O-ports. 
                  Jumps to ISR_PCINTA to handle the interrupt.
   ********************************************************************************/
   data[2] = assemble(JMP, ISR_PCINTA, 0x00);
   data[3] = assemble(NOP, 0x00, 0x00);
   
   /********************************************************************************
   * ISR_PCINTA: Interrupt routine handling PCI-interrupt for I/O-ports for switch 
   *             buttons which is connected to LEDs.
   ********************************************************************************/
   data[4] =  assemble(IN, R24, PINA);
   data[5] =  assemble(ANDI, R24, (1 << BUTTON1));
   data[6] =  assemble(JE, button2_check, 0x00);
   data[7] =  assemble(CALL, LED1_toggle, 0x00);
   data[8] =  assemble(JMP, ISR_PCINTA_end, 0x00);
   data[9] =  assemble(IN, R24, PINA);
   data[10] = assemble(ANDI, R24, (1 << BUTTON2));
   data[11] = assemble(JE, button3_check, 0x00);
   data[12] = assemble(CALL, LED2_toggle, 0x00);
   data[13] = assemble(JMP, ISR_PCINTA_end, 0x00);
   data[14] = assemble(IN, R24, PINA);
   data[15] = assemble(ANDI, R24, (1 << BUTTON3));
   data[16] = assemble(JE, ISR_PCINTA_end, 0x00);
   data[17] = assemble(CALL, LED3_toggle, 0x00);
   data[18] = assemble(RETI, 0x00, 0x00);
   
   /********************************************************************************
   * main: Initiates system start.
   ********************************************************************************/
   data[19] = assemble(CALL, setup, 0x00);
   data[20] = assemble(JMP, main_loop, 0x00);
   
   /********************************************************************************
   * setup: Activate leds, switch buttons and interrupts.
   ********************************************************************************/
   data[21] = assemble(COPYI, R16, (1 << LED1));
   data[22] = assemble(COPYI, R17, (1 << LED2));
   data[23] = assemble(COPYI, R18, (1 << LED3));
   data[24] = assemble(COPYI, R19, (1 << LED1) | (1 << LED2) | (1 << LED3));
   data[25] = assemble(OUT, DDRA, R19);
   data[26] = assemble(COPYI, XREG, 0x00);
   data[27] = assemble(COPYI, YREG, 0x00);
   data[28] = assemble(COPYI, ZREG, 0x00);
   data[29] = assemble(COPYI, R20, (1 << BUTTON1) | (1 << BUTTON2) | (1 << BUTTON3));
   data[30] = assemble(OUT, PORTA, R20);
   data[31] = assemble(SEI, 0x00, 0x00);
   data[32] = assemble(OUTI, ICR, (1 << PCIEA)); 
   data[33] = assemble(OUT, PCMSK, R20); 
   data[34] = assemble(RET, 0x00, 0x00);
   
	/********************************************************************************
	* LED1_toggle: Toggles LED1.
	********************************************************************************/
   data[35] = assemble(CPI, XREG, 0x00);
   data[36] = assemble(JE, LED1_on, 0x00);

	/********************************************************************************
	* LED1_off: LED1 off.
	********************************************************************************/
   data[37] = assemble(IN, R24, PORTA);
   data[38] = assemble(ANDI, R24, ~(1 << LED1));
   data[39] = assemble(OUT, PORTA, R24);
   data[40] = assemble(COPYI, XREG, 0x00);
   data[41] = assemble(RET, 0x00, 0x00);
   
	/********************************************************************************
	* LED1_on: LED1 on.
	********************************************************************************/
   data[42] = assemble(IN, R24, PORTA);
   data[43] = assemble(ORI, R24, (1 << LED1));
   data[44] = assemble(OUT, PORTA, R24);
   data[45] = assemble(COPYI, XREG, 0x01);
   data[46] = assemble(RET, 0x00, 0x00);
   
	/********************************************************************************
	* LED2_toggle: Toggles LED1.
	********************************************************************************/
   data[47] = assemble(CPI, YREG, 0x00);
   data[48] = assemble(JE, LED2_on, 0x00);

	/********************************************************************************
	* LED2_off: LED1 off.
	********************************************************************************/
   data[49] = assemble(IN, R24, PORTA);
   data[50] = assemble(ANDI, R24, ~(1 << LED2));
   data[51] = assemble(OUT, PORTA, R24);
   data[52] = assemble(COPYI, YREG, 0x00);
   data[53] = assemble(RET, 0x00, 0x00);
   
	/********************************************************************************
	* LED2_on: LED1 on.
	********************************************************************************/
   data[54] = assemble(IN, R24, PORTA);
   data[55] = assemble(ORI, R24, (1 << LED2));
   data[56] = assemble(OUT, PORTA, R24);
   data[57] = assemble(COPYI, YREG, 0x01);
   data[58] = assemble(RET, 0x00, 0x00);
   
	/********************************************************************************
	* LED3_toggle: Toggles LED3.
	********************************************************************************/
   data[59] = assemble(CPI, ZREG, 0x00);
   data[60] = assemble(JE, LED3_on, 0x00);

	/********************************************************************************
	* LED3_off: LED3 off.
	********************************************************************************/
   data[61] = assemble(IN, R24, PORTA);
   data[62] = assemble(ANDI, R24, ~(1 << LED3));
   data[63] = assemble(OUT, PORTA, R24);
   data[64] = assemble(COPYI, ZREG, 0x00);
   data[65] = assemble(RET, 0x00, 0x00);
   
	/********************************************************************************
	* LED3_on: LED3 on.
	********************************************************************************/
   data[66] = assemble(IN, R24, PORTA);
   data[67] = assemble(ORI, R24, (1 << LED3));
   data[68] = assemble(OUT, PORTA, R24);
   data[69] = assemble(COPYI, ZREG, 0x01);
   data[70] = assemble(RET, 0x00, 0x00);

	/********************************************************************************
	* main_loop: Initiates the system at start. A loop is then generated to keep the
	*            program running continuously.
	********************************************************************************/  
   data[71] = assemble(JMP, main_loop, 0x00);
   
   return;
}

/********************************************************************************
* program_memory_read: Reads information on specific address and returns value.
*                      If memory is full, return '0'.
********************************************************************************/
uint64_t program_memory_read(const uint16_t address)
{
   if(address < PROGRAM_MEMORY_ADDRESS_WIDTH && address >= 0)
   {
      return data[address];
   }
   else
   {
      return 0;
   }
}

/********************************************************************************
* assemble: Reads op-code, op1 and op2 in instruction. Bit shifts op-code and 
*           op1 towards MSB.
********************************************************************************/
static inline uint64_t assemble(const uint16_t op_code, const uint16_t op1, const uint32_t op2)
{
   const uint64_t instruction = ((uint64_t)(op_code) << 48) | ((uint64_t)(op1) << 32) | op2;
   return instruction;
}
