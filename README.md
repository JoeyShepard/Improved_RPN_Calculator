# Improved RPN Calculator

This is an improved version of the MSP430-based RPN Calculator [(link)](https://github.com/JoeyShepard/RPN_Calculator). 

This calculator uses an LPC1114 with a Cortex M0 for much better performance and adds a keystroke programming mode.
The display is upgraded to a VFD from Noritake. The circuit design is also greatly simplified using one SPI SRAM chip for the external memory
instead of parallel RAM with shift registers which reduces the design to only three chips total.

![Improved RPN Calculator](Improved_RPN_Calculator.JPG)

