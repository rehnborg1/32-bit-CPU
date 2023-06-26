/********************************************************************************
* control_unit.h: Contains function declarations for implementation of an
*                 32-bit control unit.
********************************************************************************/
#ifndef CONTROL_UNIT_H_
#define CONTROL_UNIT_H_

#include "cpu.h"
#include "data_memory.h"
#include "stack.h"
#include "alu.h"
#include "program_memory.h"

void control_unit_reset(void);
void control_unit_run_next_state(void);

#endif /* CONTROL_UNIT_H_ */