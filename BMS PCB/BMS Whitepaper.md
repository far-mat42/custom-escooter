
# Custom Lithium-Titanate (LTO) Battery Pack and Battery Management System (BMS)
*Design Whitepaper* | *Prepared by: Farris Matar* | *December 17, 2024*  
## Table of Contents
1 Background
2	Project Overview
2.1	Battery Requirements
2.2	Selection of Battery Cells
2.3	BMS Requirements
2.4	BMS Component Selection
2.4.1	AFE Selection: TI BQ76952
2.4.2	MCU Selection: STM32L412K8T6
3	BMS PCB Design
3.1	Charge/Discharge Paths
3.2	Cell Balancing FETs
3.3	Thermistor Connections
3.4	PCB Layout
3.5	Resistor Ladder Companion PCB
4	Software Design
5	Testing and Validation
5.1	AFE-MCU SPI Communication
5.2	Cell Voltage Measurement Accuracy
5.3	Discharge PCB Thermal Tests
6	Potential Improvements
6.1	Real-time Clock Circuit
6.2	Unseparated Charge and Discharge Paths
6.3	Support for Lower Cell Counts
6.4	Additional Thermistor Inputs
6.5	Coulomb Counting
6.6	PCB Errata
6.6.1	Flipped Diodes/FETs in Gate Drive Circuitry
6.6.2	STM32 Heartbeat Pull-down
7	Appendix
7.1	BMS Schematic

