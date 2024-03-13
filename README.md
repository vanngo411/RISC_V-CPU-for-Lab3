# RISC-V-Multi Cycle CPU runs on Alchitry Au
A multi cycle RISC-V CPU Core using Verilog and is runing on Alchitry Au.

## Abstract
This paper explores the design and implementation of a multi-cycle CPU based on the RISC-V instruction set architecture. Multi-cycle CPUs offer a balance between performance and complexity, making them a compelling choice for various computing applications. Leveraging the simplicity and efficiency of the RISC-V instruction set, our design aims to optimize critical path length while ensuring efficient execution of instructions. Through detailed analysis and simulation, we evaluate the performance and efficiency of the proposed multi-cycle CPU architecture. Our findings underscore the viability and effectiveness of RISC-V in facilitating the development of high-performance processors for diverse computing environments.

![image](https://raw.githubusercontent.com/vanngo411/MultiCycle_CPU_RISCV/main/Block%20Diagram.png)

## Final datapath

Within the RICSV_TOP module, the forwarding module plays a critical role in enhancing the efficiency of instruction execution by enabling data forwarding between pipeline stages. This module is designed to detect data dependencies and provide the necessary data to instructions in subsequent stages of the pipeline, thereby mitigating stalls and improving throughput. By monitoring the register destinations of instructions in flight and comparing them to the register sources of instructions currently in execution, the forwarding module determines if forwarding is required. Through dynamic selection of data sources using multiplexers, the forwarding module ensures that instructions receive the most up-to-date data available, optimizing performance without sacrificing correctness.

Conversely, the hazard detection module serves as a safeguard against potential hazards that could disrupt the orderly execution of instructions. It detects hazards arising from dependencies between instructions, such as data hazards and control hazards, and takes appropriate actions to resolve them. Data hazards occur when instructions depend on data produced by preceding instructions that have not yet completed execution. In such cases, the hazard detection module stalls the pipeline or initiates forwarding to ensure that instructions receive the correct data. Control hazards arise from branch instructions that alter the program counter, potentially causing incorrect instruction fetching. The hazard detection module detects these hazards and manages the control flow to maintain program correctness. By identifying and addressing hazards in real-time, the hazard detection module ensures smooth and uninterrupted execution of instructions within the RICSV_TOP module, contributing to overall system reliability and efficiency.

![image](https://github.com/vanngo411/MultiCycle_CPU_RISCV/blob/main/DataAndControlPath.png)

## Synthesized Desigh - Device
![image](https://github.com/vanngo411/RISC_V-CPU-for-Lab3/blob/main/Device.png)

## Schematic for LEDs output on Alchitry Au
![image](https://github.com/vanngo411/RISC_V-CPU-for-Lab3/blob/main/LEDs_Showing.png)

In the diagram above, we can see that there is a RAM that store 8 overwritten vlaues when logic RegWrite on and it is the main part for the top module connects to Alchitry Au. For each output value that write back to register file, a line is used to send to RAM for LEDs flashing. The signal to allow write-on comes from up counter that count from 0 to 7 and then over flow. To show the value of RAM on LEDs, we use a clock divider to be slow down LEDs changing state for each 4 second. 

## LEDs output on Alchitry Au
![image](https://github.com/vanngo411/RISC_V-CPU-for-Lab3/blob/main/Alchitry_Au.png)

The LEDs show up first four values for 0 because we don't have any data on write back signal. The fifth, sixth and seventh value have 125, 133 and 200. And then turn off again because no input before repeating again.

## Notes

File RISCV_TOP.v contains all module needed for top module to run on Alchitry Au.
