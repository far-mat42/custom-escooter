# Custom Lithium-Titanate (LTO) Battery Pack and Battery Management System (BMS) Design Whitepaper

**Prepared by:** Farris Matar  
**Date:** December 17, 2024

---

# Table of Contents

1. [Background](#1-background)  
2. [Project Overview](#2-project-overview)  
   - [2.1 Battery Requirements](#21-battery-requirements)  
   - [2.2 Selection of Battery Cells](#22-selection-of-battery-cells)  
   - [2.3 BMS Requirements](#23-bms-requirements)  
   - [2.4 BMS Component Selection](#24-bms-component-selection)  
     - [2.4.1 AFE Selection: TI BQ76952](#241-afe-selection-ti-bq76952)  
     - [2.4.2 MCU Selection: STM32L412K8T6](#242-mcu-selection-stm32l412k8t6)  
3. [BMS PCB Design](#3-bms-pcb-design)  
   - [3.1 Charge/Discharge Paths](#31-charge-discharge-paths)  
   - [3.2 Cell Balancing FETs](#32-cell-balancing-fets)  
   - [3.3 Thermistor Connections](#33-thermistor-connections)  
   - [3.4 PCB Layout](#34-pcb-layout)  
   - [3.5 Resistor Ladder Companion PCB](#35-resistor-ladder-companion-pcb)  
4. [Software Design](#4-software-design)  
5. [Testing and Validation](#5-testing-and-validation)  
   - [5.1 AFE-MCU SPI Communication](#51-afe-mcu-spi-communication)  
   - [5.2 Cell Voltage Measurement Accuracy](#52-cell-voltage-measurement-accuracy)  
   - [5.3 Discharge PCB Thermal Tests](#53-discharge-pcb-thermal-tests)  
6. [Potential Improvements](#6-potential-improvements)  
   - [6.1 Real-time Clock Circuit](#61-real-time-clock-circuit)  
   - [6.2 Unseparated Charge and Discharge Paths](#62-unseparated-charge-and-discharge-paths)  
   - [6.3 Support for Lower Cell Counts](#63-support-for-lower-cell-counts)  
   - [6.4 Additional Thermistor Inputs](#64-additional-thermistor-inputs)  
   - [6.5 Coulomb Counting](#65-coulomb-counting)  
   - [6.6 PCB Errata](#66-pcb-errata)  
     - [6.6.1 Flipped Diodes/FETs in Gate Drive Circuitry](#661-flipped-diodesfets-in-gate-drive-circuitry)  
     - [6.6.2 STM32 Heartbeat Pull-down](#662-stm32-heartbeat-pull-down)  
7. [Appendix](#7-appendix)  
   - [7.1 BMS Schematic](#71-bms-schematic)  


---

## 1. Background
This project is part of a larger overall objective to assemble an e-scooter with all electronics built from scratch. The primary motivation of this endeavor is to learn about all the components that go into designing a small-scale electric vehicle and attempt to turn this knowledge into practical experience by building the components in order to create an e-scooter that is comparable to most consumer-range e-scooters. In order to verify the final build’s performance, it should be able to achieve the following specifications:

- Top speed of 30km/h
- Minimum range of 30km (approximately 1 hour of continuous use)
- Adjustable speed via a user-operated throttle
- Display to show power consumption, battery’s remaining capacity, and current speed

The battery and battery management system (BMS) were the first components to be designed for the e-scooter. The specifications above were used to guide the defining requirements of these components.  
At this stage, the only other component to be selected was the motor. The motor intended for use in the e-scooter is a 36V, 720W brushless DC motor. Since the motor will consume the most power by far compared to any other component in the e-scooter, it was important to have these parameters to further define this project’s requirements.

## 2. Project Overview
The battery’s role in the e-scooter is to store energy to be supplied for the motor and other electronic components when the e-scooter is being driven. The BMS’s role is to act as a layer of protection for the battery, constantly monitoring the voltage, current, temperature, and other metrics for both telemetry purposes and to activate protections that will disable battery charging and/or discharging when appropriate.

### 2.1 Battery Requirements
A summary of the main requirements for the battery is provided in Table 2.1 below.

**Table 2.1: Requirements for the e-scooter battery**

| Requirement | Reasoning |
|------------|----------|
| **Battery capacity at least 720 Wh** | 720Wh is a sizeable amount that will keep a balance between total capacity and battery weight. 720Wh allows powering the 720W motor continuously at max. power for about an hour, ensuring the minimum range is met. |
| **Battery nominal voltage is around 36V** | This ensures the motor voltage rating is not exceeded. Although a DC-DC converter can be used to regulate the motor voltage, keeping the nominal battery voltage around the rated motor voltage can reduce the need for this, improving efficiency. |
| **Battery discharging rate is at least 30A** | 30A of discharging allows outputting up to roughly 1080W of power with a nominal voltage around 36V, which is well beyond the expected power draw of the motor and provides a healthy margin for powering auxiliary electronics. |
| **Battery charging rate is at least 10A** | 10A of charging current will allow the battery to be fully charged in just over 2 hours, based on the nominal voltage and capacity requirements, which is a reasonable amount of time to wait while not stressing on the capabilities of the battery. |
| **Battery can operate in ambient temperatures between -15ºC and 30ºC.** | Because the e-scooter is to be used outside, it should be capable of running in a wide range of temperatures without a significant change in performance. |

### 2.2 Selection of Battery Cells
In addition to the specifications above, a battery chemistry that was relatively stable and safe to use was prioritized. After careful consideration, the battery chemistry selected was **lithium titanate (LTO)**. Lithium titanate is known for being more safe and thermally stable compared to other lithium-ion battery chemistries. Additionally, it is capable of very high charging and discharging rates, which are appropriate given the high power consumption of the e-scooter and the benefit of having a shorter charge time. LTO batteries also have excellent low-temperature discharging capabilities, typically being able to retain over 80% capacity at -30ºC.

The major downside compared to other chemistries is the energy density – for the same capacity, LTO batteries will typically be heavier and larger than their counterparts using other chemistries. Although this isn’t ideal for an e-scooter, it was accepted as a necessary tradeoff for its safety and high performance in the required specifications.  
For the specific cells to be used to build the battery, the **Toshiba SCiB 20Ah** cells were chosen. These cells have a nominal voltage of 2.3V, meaning with a **16s1p battery pack**, the total capacity would be approximately **736 Wh**. The energy density of the Toshiba SCiB cells were also higher than most commercially available LTO cells, reducing the weight disadvantage.

---
