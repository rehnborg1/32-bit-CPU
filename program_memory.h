/********************************************************************************
* program_memory.h: Macro, functions and definitions for program_memory.c.
********************************************************************************/
#ifndef PROGRAM_MEMORY_H_
#define PROGRAM_MEMORY_H_

#include "control_unit.h"

#define PROGRAM_MEMORY_DATA_WIDTH 32
#define PROGRAM_MEMORY_ADDRESS_WIDTH 100

void program_memory_write(void);

uint64_t program_memory_read(const uint16_t address);

#endif /* PROGRAM_MEMORY_H_ */