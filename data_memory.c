/********************************************************************************
* data_memory.c: Functions handling the data memory.
********************************************************************************/
#include "data_memory.h"

/********************************************************************************
* data_memory: Array storing the memory addresses.
********************************************************************************/
static uint32_t data_memory[DATA_MEMORY_ADDRESS_WIDTH];

/********************************************************************************
* data_memory_reset: Resets the values of data memory.
********************************************************************************/
void data_memory_reset(void)
{
	for(uint32_t i = 0; i < DATA_MEMORY_ADDRESS_WIDTH; i++)
	{
		data_memory[i] = 0;
	}

	return;
}

/********************************************************************************
* data_memory_address_clear: Resets a specific address in data memory.
********************************************************************************/
void data_memory_address_clear(const uint16_t address)
{
	data_memory[address] = 0;

	return;
}

/********************************************************************************
* data_memory_write: Writes a value to indicated address in data memory.
********************************************************************************/
int data_memory_write(const uint16_t address, const uint32_t value)
{
	if (address < DATA_MEMORY_ADDRESS_WIDTH && address >= 0)
	{
		data_memory[address] = value;
		return 0;
	}
	else
	{
		return 1;
	}
}

/********************************************************************************
* data_memory_read: Reads and returns a specific address in data memory.
********************************************************************************/
uint32_t data_memory_read(const uint16_t address)
{
	if (address < DATA_MEMORY_ADDRESS_WIDTH && address >= 0)
	{
		return data_memory[address];
	}
	else
	{
		return 0;
	}
}

/********************************************************************************
* data_memory_set_bit: Sets specific bit to '1' onto given address.
********************************************************************************/
void data_memory_set_bit(const uint16_t address, const uint32_t bit)
{
	data_memory[address] |= (1 << bit);

	return;
}

/********************************************************************************
* data_memory_clear_bit: Sets specific bit to '0' onto given address.
********************************************************************************/
void data_memory_clear_bit(const uint16_t address, const uint32_t bit)
{
	data_memory[address] &= ~(1 << bit);

	return;
}