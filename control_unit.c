/********************************************************************************
* control_unit.c: Includes the states Fetch, Decode, Execute. And variables
*                 op-code, op1 and op2.
********************************************************************************/
#include "control_unit.h"

/* Functions */
static void check_for_irq(void);
static void generate_interrupt(const uint8_t interrupt_vector);
static void monitor_PINA(void);
static inline void update_io(void);

/* Variables */
static uint16_t pc;                                   /* Program counter */
static uint64_t ir;									  /* instruction register */
static uint16_t mar;								  /* Memory address register */
static uint8_t sr;									  /* Status register */

static uint16_t op_code;							  /* Stores the op_code */
static uint16_t op1;                                  /* Stores the first operand */
static uint32_t op2;                                  /* Stores the second operand */

static enum cpu_state state;                          /* Keeps track of fetch, decode and execute */
static uint32_t reg[CPU_REGISTER_ADDRESS_WIDTH];

static uint32_t PINA_previous;                        /* Stores the input signal of the last cycle to PINA */

/********************************************************************************
* control_unit_reset: Resets the control unit and its memories.
********************************************************************************/
void control_unit_reset(void)
{
	pc = 0x00;
	ir = 0x00;
	mar = 0x00;
	sr = 0x00;

	op_code = 0x00;
	op1 = 0x00;
	op2 = 0x00;

	state = CPU_STATE_FETCH;

	PINA_previous = 0x00;

	for (uint32_t i = 0; i < CPU_REGISTER_ADDRESS_WIDTH; ++i)
	{
		reg[i] = 0;
	}

	data_memory_reset();
	stack_reset();
	program_memory_write();
	return;
}

/********************************************************************************
* control_unit_run_next_state: CPU cycle.
********************************************************************************/
void control_unit_run_next_state(void)
{
	switch (state)
	{
		case CPU_STATE_FETCH:
		{
			ir = program_memory_read(pc); /* Fetches next instruction. */
			mar = pc;                     /* Stores address of current instruction. */
			pc++;                         /* Program counter points to next instruction. */
			state = CPU_STATE_DECODE;     /* Decodes the instruction during next clock cycle. */
			break;
		}
		case CPU_STATE_DECODE:
		{
			op_code = ir >> 48;           /* Bit 63 downto 48 consists of the OP code. */
			op1 = ir >> 32;               /* Bit 47 downto 32 consists of the first operand. */
			op2 = ir;                     /* Bit 31 downto 0 consists of the second operand. */
			state = CPU_STATE_EXECUTE;    /* Executes the instruction during next clock cycle. */
			break;
		}
		case CPU_STATE_EXECUTE:
		{
			switch (op_code)
			{
				case NOP:                /* Do nothing, next row. */
				{
					break;
				}
				case COPY:               /* Copies valuables between CPU registers with operands. The value is stored in the register indicated by op1. */
				{
					reg[op1] = reg[op2];
					break;
				}
				case COPYI:              /* Copies the value from op2 to the register indicated by op1. */
				{
					reg[op1] = op2;
					break;
				}
				case IN:                 /* Reading the I/O port using op2 and puts the value in the register indicated by op1. */
				{
					reg[op1] = data_memory_read(op2);
					break;
				}
				case INP:                /* Reading values from the referred address in op2 to the indicated register in op1. */
				{
					reg[op1] = data_memory_read(reg[op2]);
					break;
				}
				case OUT:                /* Reading the value from the I/O port indicated by op2 and puts it in the computer memory address indicated by op1. */
				{
					data_memory_write(op1, reg[op2]);
					break;
				}
				case OUTI:               /* Writes the value indicated by op2 to I/O port indicated by op1. */
				{
					data_memory_write(op1, op2);
					break;
				}
				case OUTP:               /* Writes the value from the adress indicated by op2 to I/O port indicated as an address by op1. */
				{
					data_memory_write(reg[op1], reg[op2]);
					break;
				}
				case CLR:                /* Erasing values in the CPU register. */
				{
					reg[op1] = 0x00;
					break;
				}
				case AND:                /* Executes AND with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				
				{
					reg[op1] = alu(AND, reg[op1], reg[op2], &sr);
					break;
				}
				case ANDI:               /* Executes AND with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(AND, reg[op1], op2, &sr);
					break;
				}
				case OR:                 /* Executes OR with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				{
					reg[op1] = alu(OR, reg[op1], reg[op2], &sr);
					break;
				}
				case ORI:                /* Executes OR with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(OR, reg[op1], op2, &sr);
					break;
				}
				case XOR:                /* Executes XOR with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				{
					reg[op1] = alu(XOR, reg[op1], reg[op2], &sr);
					break;
				}
				case XORI:               /* Executes XOR with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(XOR, reg[op1], op2, &sr);
					break;
				}
				case ADD:                /* Executes addition with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				{
					reg[op1] = alu(ADD, reg[op1], reg[op2], &sr);
					break;
				}
				case ADDI:               /* Executes addition with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(ADD, reg[op1], op2, &sr);
					break;
				}
				case SUB:                /* Executes subtraction with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				{
					reg[op1] = alu(SUB, reg[op1], reg[op2], &sr);
					break;
				}
				case SUBI:               /* Executes addition with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(SUB, reg[op1], op2, &sr);
					break;
				}
				case MUL:                 /* Executes multiplication with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				{
					reg[op1] = alu(MUL, reg[op1], reg[op2], &sr);
					break;
				}
				case MULI:                /* Executes multiplication with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(MUL, reg[op1], op2, &sr);
					break;
				}
				case DIV:                 /* Executes division with the values in CPU register indicated på op1 and op2. The result is stored in the CPU register indicated by op1. */
				{
					reg[op1] = alu(DIV, reg[op1], reg[op2], &sr);
					break;
				}
				case DIVI:                /* Executes division with the immidiate value indicated by op2 and the value from CPU register indicated by op1. */
				{
					reg[op1] = alu(DIV, reg[op1], op2, &sr);
					break;
				}
				case INC:                 /* Incremets the value to the CPU register indicated by operand 1. */
				{
					reg[op1] = alu(ADD, reg[op1], 1, &sr);
					break;
				}
				case DEC:                 /* Decremets the value to the CPU register indicated by operand 1. */
				{
					reg[op1] = alu(SUB, reg[op1], 1, &sr);
					break;
				}
				case POP:                 /* Pops a value from the stack to the CPU register indicated by op1. */
				{
					reg[op1] = stack_pop();
					break;
				}
				case PUSH:                /* Pushes the value from CPU register indicated by op1 on the stack */
				{
					stack_push(reg[op1]);
					break;
				}
				case CALL:                /* Saves the adress pushed from the stack to the value indicated by op1. */
				{
					stack_push(pc);
					pc = op1;
					break;
				}
				case RET:                 /* Returns the value that popped from the stack. */
				{
					pc = stack_pop();
					break;
				}
				case RETI:                /* The status register and the program returns to the state before RETI was executed adn the interrupt flag is reactivated. */
				{
					sr = stack_pop();
					pc = stack_pop();
					sr |= (1 << I);
					break;
				}
				case CP:                  /* Compares the values in the registers indicated by op1 and op2. */
				{
					(void)alu(SUB, reg[op1], reg[op2], &sr);
					break;
				}
				case CPI:                 /* Compares the value in the register indicated by op1 with the value of op2. */
				{
					(void)alu(SUB, reg[op1], op2, &sr);
					break;
				}
				case JMP:                 /* Jumps to the address indicated by op1. */
				{
					pc = op1;
					break;
				}
				case JE:                  /* If the Zero flag is activated, jumps to the address indicated by op1. */
				{
					if ((sr & (1 << Z))) pc = op1;
					break;
				}
				case JNE:                 /* If the Zero flag is deactivated, jumps to the address indicated by op1. */
				{
					if (!(sr & (1 << Z))) pc = op1;
					break;
				}
				case JGE:                 /* If the Signed flag is deactivated, jumps to the address indicated by op1. */
				{
					if (!(sr & (1 << S))) pc = op1;
					break;
				}
				case JGT:                 /* If the Zero and Signed flag is deactivated, jumps to the address indicated by op1. */
				{
					if (!(sr & (1 << S)) && !(sr & (1 << Z))) pc = op1;
					break;
				}
				case JLE:                 /* If the Zero and Signed flag is activated, jumps to the address indicated by op1. */
				{
					if ((sr & (1 << S)) || (sr & (1 << Z))) pc = op1;
					break;
				}
				case JLT:                 /* If the Signed flag is activated, jumps to the address indicated by op1. */
				{
					if ((sr & (1 << S))) pc = op1;
					break;
				}
				case LSL:                 /* Bit shifts the values indicated by op1 to the left. */
				{
					reg[op1] = reg[op1] << 1;
					break;
				}
				case LSR:                 /* Bit shifts the values indicated by op1 to the right. */
				{
					reg[op1] = reg[op1] >> 1;
					break;
				}
				case SEI:                 /* Activates the interrupt flag. */
				{
					sr |= (1 << I);
					break;
				}
				case CLI:                 /* Dectivates the interrupt flag. */
				{
					sr &= ~(1 << I);
					break;
				}
				default:                  /* System reset if an error occurs. */
				{
					control_unit_reset();
					break;
				}
			}

			state = CPU_STATE_FETCH;
			check_for_irq();
			break;
		}
		default:                          /* System reset if an error occurs. */
		{
			control_unit_reset();
			break;
		}
	}

	update_io();
	monitor_PINA();
	return;
}

/********************************************************************************
* control_unit_run_next_state: Executes the next CPU instruction cycle.
********************************************************************************/
void control_unit_run_next_instruction_cycle(void)
{
	do
	{
		control_unit_run_next_state();
	} while (state != CPU_STATE_EXECUTE);
	return;
}

/********************************************************************************
* check_for_irq: Checks for a interrupt. If an interrupt is inbound it executes
*                                        and jumps to the address in
*										 PCINTA_vect.
********************************************************************************/
static void check_for_irq(void)
{
	if ((sr & (1 << I)))
	{
		const uint32_t ifr = data_memory_read(IFR);
		const uint32_t icr = data_memory_read(ICR);

		if ((ifr & (1 << PCIFA)) && (icr & (1 << PCIEA)))
		{
			data_memory_clear_bit(IFR, PCIFA);
			generate_interrupt(PCINTA_vect);
		}
	}
	return;
}

/********************************************************************************
* generate_interrupt: Pushes content of all registers in the control unit to
*                     the stack and generates and interrupt, where the program
*                     counter is assigned the interrupt vector.
********************************************************************************/
static void generate_interrupt(const uint8_t interrupt_vector)
{
	stack_push(pc);
	stack_push(sr);
	sr &= ~(1 << I);
	pc = interrupt_vector;
	return;
}

/********************************************************************************
* monitor_PINA: Monitors pin change interrupts on I/O ports. All pins where
*               pin change monitoring is enabled (corresponding mask bit in
*               Pin Change Mask Register A i set) is monitored by comparing
*               current input signal versus the old one. If they don't match,
*               the corresponding PCIFA (pin change interrupt flag)
*               in the ifr (Interrupt Flag Register)
*               register is set to generate an interrupt request (IRQ).
********************************************************************************/
static void monitor_PINA(void)
{
	const uint32_t PINA_current = data_memory_read(PINA);
	const uint32_t pcmska = data_memory_read(PCMSK);

	for (uint32_t i = 0; i < DATA_MEMORY_DATA_WIDTH; ++i)
	{
		if ((pcmska & (1 << i)))
		{
			if ((bool)(PINA_current & (1 << i)) != (bool)(PINA_previous & (1 << i)))
			{
				data_memory_set_bit(IFR, PCIFA);
				break;
			}
		}
	}

	PINA_previous = PINA_current;
	return;
}

/********************************************************************************
* update_io: Updates the I/O ports to register A.
********************************************************************************/
static inline void update_io(void)
{
	const uint32_t ddra = data_memory_read(DDRA);
	const uint32_t porta = data_memory_read(PORTA);
	const uint32_t pina = ((uint32_t)(PINC) << 16) | ((uint16_t)(PINB) << 8) | PIND;

	DDRD = (uint8_t)(ddra);
	DDRB = (uint8_t)(ddra >> 8);
	DDRC = (uint8_t)(ddra >> 16);

	PORTD = (uint8_t)(porta);
	PORTB = (uint8_t)(porta >> 8);
	PORTC = (uint8_t)(porta >> 16);
	
	data_memory_write(PINA, pina);
	return;
}