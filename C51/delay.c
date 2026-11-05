#include <REGX52.H>
#include <INTRINS.H>

void Delay1ms()		//@12.000MHz
{
	unsigned char i, j;

	i = 2;
	j = 239;
	do
	{
		while (--j);
	} while (--i);
}


void delay(unsigned int x)
{
	unsigned int i;
   for (i=0;i<x;i++)
	{
		Delay1ms();
	}
}
