# 🎵 ESP32-STM32 Hybrid Audio Bridge

**Status:** _Archived / Pivot to Phase 2 (I2S)_  
**Tags:** Embedded Systems, Audio Processing, Dual-Core Architecture

## 📖 Overview
This project is an experimental attempt to build a distributed Bluetooth Speaker system by manually bridging two microcontrollers without using a standard audio protocol like I2S. 

The goal was to offload specific tasks to dedicated hardware:
* **Master (ESP32):** Handles Bluetooth A2DP connectivity, packet reception, and decoding.
* **Slave (STM32 Nucleo-F446RE):** Handles Digital Signal Processing (DSP), buffering, and analog output via internal DAC.
* **Bridge:** Custom SPI communication protocol using DMA (Direct Memory Access).

## 🔌 Hardware Setup
Component,Role,Specifications
* ESP32 DevKit V1,Receiver & Master,"Bluetooth 4.2, 240MHz Dual Core"
* STM32 Nucleo-F446RE,Audio Engine,"180MHz, Internal 12-bit DAC"
* LM386 Module Amplifier,"Class AB Amp, Gain=20"
* Speaker,Output,"4Ω, 5W"
* Power Supply

## 🧪 Lessons Learned
Protocol Matters: SPI is excellent for data, but for real-time audio, I2S (Inter-IC Sound) is superior because it handles clock synchronization natively.Debugging Tools: The Logic Analyzer was crucial in identifying that the code wasn't "crashing," but rather suffering from timing jitter.System Design: Offloading processing is powerful, but the inter-chip communication overhead must be lower than the processing gain.

## 🚀 Future Roadmap (Phase 2)
* Replacing SPI bridge with PCM5102 I2S DAC.
* Implementing Port DSP algorithms (Bass Boost/EQ) directly to ESP32 or use STM32 as an I2S Post-Processor.

## 🛠️ Usage (For Educational Purposes)
If you want to replicate the SPI experiment:STM32: Open STM32_Audio_Slave project in STM32CubeIDE. Flash to Nucleo board.ESP32: Open ESP32_BT_Source in Arduino IDE. Install ESP32-A2DP library. Flash to ESP32.Connect: Ensure Common Ground is connected before powering up.
 *Built by [Jaikrishnan](https://github.com/Jaikrishnan-p15) as a study in embedded systems architecture.* 
## 🏗️ Architecture
```mermaid
graph LR
    Phone((Smartphone)) -- Bluetooth A2DP --> ESP32
    ESP32 -- SPI (PCM Data) --> STM32
    STM32 -- DMA --> Internal_DAC
    Internal_DAC -- Analog Signal --> LM386_Amp
    LM386_Amp --> Speaker



