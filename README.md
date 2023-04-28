# Mini-SRC
This project was created by Michael Geddes and Will Dormer for Queen's University course ELEC 374. The project is a 32-bit
RISC-style CPU developed in Verilog for deployment on an Altera Cyclone DE0-CV FPGA Board.

## CPU Features

- 32-Bit
- 30 Instructions
- 16.67 MHz Clock Speed
- 2 KiB of Memory
- 16 General Purpose Registers
- Hardware Multiply and Divide
- 1 32-Bit Output Port
- 1 32-Bit Input Port

## Instruction Set
The CPU has the following instructions:

### Load and Store Instructions
- ld (Load)
- ldi (Load Immediate)
- st (Store)

### Arithmetic and Logical Instructions
- add (Addition)
- sub (Subtraction)
- shr (Shift Right)
- shl (Shift Left)
- ror (Rotate Right)
- rol (Rotate Left)
- and (Bitwise AND)
- or (Bitwise OR)
- addi (Add immediate)
- andi (Bitwise AND with immediate)
- ori (Bitwise OR with immediate)
- mul (Multiply)
- div (Divide)
- neg (Negate)
- not (Bitwise NOT)

### Conditional Branch Instructions
- brzr (Branch if zero)
- brnz (Branch if not zero)
- brmi (Branch if negative)
- brpl (Branch if positive)

### Jump Instructions
- jr (Return from procedure)
- jal (Jump and Link)

### I/O Instructions
- in (Read from inport)
- out (Write to outport)


### Misc Instructions
- mfhl (Move from HI register)
- mflo (Move from LO register)
- nop (No operation)
- halt (Stop execution)
