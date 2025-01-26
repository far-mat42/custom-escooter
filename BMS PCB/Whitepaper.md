![Title Photo](https://github.com/user-attachments/assets/f25604ed-ffee-4ee7-9c4c-09e9fbd8e98d)
# Custom Lithium-Titanate (LTO) Battery Pack and Battery Management System (BMS)

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

---

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
For the specific cells to be used to build the battery, the [Toshiba SCiB 20Ah](https://www.global.toshiba/ww/products-solutions/battery/scib/product/cell/high-energy.html) cells were chosen. These cells have a nominal voltage of 2.3V, meaning with a 16s1p battery pack, the total capacity would be approximately 736 Wh. The energy density of the Toshiba SCiB cells were also higher than most commercially available LTO cells, reducing the weight disadvantage.

## 2.3 BMS Requirements

A summary of the main requirements for the BMS is provided in Table 2.2 below.

*Table 2.2: Requirements for the BMS PCB*

| Requirement | Reasoning |
|-------------|----------|
| BMS PCB and component ratings are sufficiently high to support the voltage, current, cell count, and temperature range the battery will operate with | This is self-explanatory – the BMS should not limit the battery from achieving its required specifications. |
| BMS is capable of measuring cell voltages, battery current, and battery temperature with less than 1% error. | 1% error ensures sufficient accuracy for the BMS to report the battery’s status and specify when to enable protections. |
| BMS is capable of performing cell balancing, supporting at least 500mA of balancing current. | Given the fairly high capacity requirement, a decently high balancing current should be used to ensure the final stage of charging does not take excessively long in the event of unbalanced cell voltages. |
| BMS will send telemetry about battery voltage, current, and temperature over UART | UART was selected for its simple connection and reliability over medium-length cables. Additionally, if signal integrity becomes an issue due to cable length, it can be improved easily by converting it into a differential protocol such as RS422 through a pair of transceivers. |

## 2.4 BMS Component Selection

The two key components that would guide the BMS PCB design were the analog front-end (AFE) and the microcontroller unit (MCU).

### 2.4.1 AFE Selection: TI BQ76952

The AFE’s purpose is to provide high-accuracy and precision measurements of the cell voltages, charging/discharging current, and battery temperature. An AFE can be given additional control in the BMS, such as controlling the charging and discharging MOSFETs and the cell balancing MOSFETs based on its measurements.

For this BMS, the [TI BQ76952 AFE](https://www.ti.com/product/BQ76952) was selected. It uses 24-bit ADCs for measuring voltages, temperatures (via external thermistors), and current, reporting excellent accuracy under test conditions. It also offers a full suite of protections including over/undervoltage, over/undercurrent during both charge and discharge, and over/undertemperature, in addition to supporting autonomous cell-balancing. TI also provides extensive documentation in its [technical reference manual](https://www.ti.com/lit/ug/sluuby2b/sluuby2b) on how to configure the AFE, providing an extensive variety of options based on the configuration of the battery and the level of control desired for the AFE in terms of operation and protection. All of these features provided good confidence in the BQ76952 allowing the BMS to achieve all the specified requirements.

### 2.4.2 MCU Selection: STM32L412K8T6

The MCU’s primary role is to program and communicate with the BQ76952 over I2C or SPI, as well as summarize and report information on the battery status given by the BQ76952 on a UART channel. These requirements are fairly simple and plenty of microcontrollers would have been able to accomplish them, however the STM32L412K8T6 was chosen for 2 specific reasons. The first is that STM’s microcontrollers are very commonly used both in industry and by hobbyists, providing plenty of reference material to assist with programming and debugging it for use on the BMS. The second is for its extremely low power consumption. The STM32L4 series of microcontrollers are capable of operating with very low operating current consumption (in the range of a few hundred microamps), as well as allowing several modes of operation for reducing the current draw of the MCU down to tens of nanoamps. This is perfect for minimizing the power consumed from the battery while the e-scooter is not in use.

---

## 3 BMS PCB Design


