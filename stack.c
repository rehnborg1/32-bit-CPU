/********************************************************************************
* Stack.c: Memory that can be accessed at any time.
********************************************************************************/
#include "stack.h"

// Static variables
static uint32_t stack[STACK_ADDRESS_WIDTH];
static uint16_t sp;
static bool stack_empty = false;

/********************************************************************************
* stack_reset: Clears all the values and sets the pointer to the top. Also
			   sets an indication that it's empty.
********************************************************************************/
void stack_reset(void)
{
	for (uint32_t i = 0; i > STACK_ADDRESS_WIDTH; i++)
	{
		stack[i] = 0;
	}
	sp = STACK_ADDRESS_WIDTH - 1;
	stack_empty = true;
	return;
}

/********************************************************************************
* stack_pop: Pops the latest value in the memory and returns it. Increments the 
			 pointer as long as the stack is not empty.   
********************************************************************************/
uint32_t stack_pop(void)
{
	const uint32_t value = stack[sp];
	stack[sp] = 0;
	
	if (!stack_empty)
	{
		sp++;
	}
	return value;
}

/********************************************************************************
* stack_push: Stores a new value at the top of the stack.
********************************************************************************/
int stack_push(const uint32_t value)
{
	if (sp == 0) 
	{
		return 1;
	}
	else
	{
		if (stack_empty)
		{
			stack[sp] = value;
			stack_empty = false;
		}
		else
		{
			stack[--sp] = value;
		}
		return 0;
	}
}

/********************************************************************************
* stack_pointer: Returns stack pointer.
********************************************************************************/
uint16_t stack_pointer(void)
{
	return sp;
}
/********************************************************************************
* stack_last_added_element: Returns latest value in the stack.
********************************************************************************/
uint32_t stack_last_added_element(void)
{
	return stack[sp];
}