/********************************************************************************
* main.c: Runnnig 32-bit CPU.
********************************************************************************/
#include "control_unit.h"
#include "cpu.h"

int main(void)
{
	control_unit_reset();
	
	while (1)
	{
		control_unit_run_next_state();
	}
}

