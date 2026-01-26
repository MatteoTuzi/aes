# Lab 1 - Polling and Interrupts

This repository contains three implementations for managing GPIO peripherals on a Zybo Z7 board with MicroBlaze processor, demonstrating different approaches to input handling: polling and interrupt-driven architectures.

## Project Structure

The workspace contains three main implementations:

### a) `lab1_polling.c` - Polling Basic Version

**Description:**
A basic polling implementation that continuously checks the state of input switches and mirrors their binary pattern on the output LEDs.

**Technical Details:**
- **Architecture:** Polling-based (synchronous, blocking)
- **Input:** 4-bit switches connected to GPIO_1 (base address: 0x40010000)
- **Output:** 4-bit LEDs connected to GPIO_0 (base address: 0x40000000)
- **Operation:** 
  - Configures GPIO direction registers (TRI register: 0x04 offset)
  - Implements a continuous `while(1)` loop that reads switch values using `Xil_In32()`
  - Updates LEDs only when switch state changes (optimization to reduce unnecessary writes)
  - Uses direct memory-mapped register access (offset 0x00 for DATA register)
- **Key Features:**
  - Simple, straightforward implementation
  - Low latency for switch changes
  - CPU-intensive (continuously polling)
  - No interrupt controller required

### b) `lab1_interrupt_a.c` - Interrupt Version Part 1

**Description:**
An interrupt-driven implementation using the AXI Interrupt Controller (INTC) to handle two independent interrupt sources: switch changes and button presses.

**Technical Details:**
- **Architecture:** Interrupt-driven (asynchronous, event-driven)
- **Interrupt Controller:** AXI INTC (base address: 0x41200000)
- **Interrupt Sources:**
  - **IRQ0 (Switch):** GPIO_1 generates interrupt on switch state change
  - **IRQ1 (Button):** GPIO_2 generates interrupt on button press/release
- **Switch Interrupt Handler:**
  - Reads 4-bit switch values from GPIO_1 DATA register
  - Implements `FindMostSignificantBit()` algorithm to determine MSB index (0-3)
  - Displays MSB index in binary on LEDs (e.g., MSB=2 → LED=0010)
  - Shows 0x00 if no switches are ON
- **Button Interrupt Handler:**
  - Reads current LED pattern
  - Performs bitwise negation (inverts all 4 bits)
  - Implements software debounce (50ms delay using `usleep()`)
- **Interrupt Configuration:**
  - GPIO-level: Enables GIER (Global Interrupt Enable, offset 0x11C) and IPIER (IP Interrupt Enable, offset 0x128)
  - INTC-level: Configures IER (Interrupt Enable Register, offset 0x08) and MER (Master Enable Register, offset 0x1C)
  - Uses OR-mask operations (`|`) to preserve existing register values
  - Registers ISR with exception system using `Xil_ExceptionRegisterHandler()`
- **Acknowledge Sequence:**
  - GPIO acknowledge: Writes to IPISR (IP Interrupt Status Register, offset 0x120) - Toggle-On-Write operation
  - INTC acknowledge: Writes to IAR (Interrupt Acknowledge Register, offset 0x0C)
- **Key Features:**
  - Non-blocking main loop (CPU can perform other tasks)
  - Immediate response to events
  - Lower CPU usage compared to polling
  - Proper interrupt nesting and acknowledge handling

### c) `lab1_interrupt_b.c` - Interrupt Version Part 2

**Description:**
Extended interrupt implementation with modified button interrupt behavior: adds the last digit of student ID to current LED pattern before negation.

**Technical Details:**
- **Architecture:** Interrupt-driven (same as Part 1)
- **Switch Interrupt Handler:** Identical to Part 1
- **Button Interrupt Handler (Modified):**
  - Reads current LED pattern (4 LSB bits)
  - Adds `STUDENT_ID_LAST_DIGIT` constant (configurable, default: 0 for student ID 70/90/00490)
  - Masks result to 4 bits to prevent overflow: `(led_value + STUDENT_ID_LAST_DIGIT) & 0xF`
  - Performs bitwise negation: `(~result) & 0xF`
  - Writes final value to LEDs
  - Includes software debounce (50ms)
- **Mathematical Operation:**
  - Formula: `LED_new = ~((LED_current + student_id_digit) & 0xF) & 0xF`
  - Example: LED=0010 (2), student_id=0 → (2+0)&0xF=2 → ~2&0xF=13 (1101)
- **Key Features:**
  - Demonstrates arithmetic operations in interrupt context
  - Configurable student ID constant for personalization
  - Same interrupt architecture as Part 1
  - Maintains proper interrupt handling and acknowledge sequence

## Hardware Configuration

- **Target Board:** Digilent Zybo Z7 (Zynq-7000 SoC)
- **Processor:** MicroBlaze soft-core processor
- **GPIO Peripherals:**
  - GPIO_0 (0x40000000): 4-bit LED output
  - GPIO_1 (0x40010000): 4-bit switch input (IRQ0 source)
  - GPIO_2 (0x40020000): 4-bit button input (IRQ1 source)
- **Interrupt Controller:** AXI INTC (0x41200000)

## Register Map

### GPIO Registers (per peripheral)
- **DATA** (0x00): Data register for reading/writing I/O values
- **TRI** (0x04): Tri-state/direction register (1=input, 0=output)
- **GIER** (0x11C): Global Interrupt Enable Register (bit 31 enables interrupts)
- **IPISR** (0x120): IP Interrupt Status Register (Toggle-On-Write)
- **IPIER** (0x128): IP Interrupt Enable Register (channel-level enable)

### Interrupt Controller Registers
- **ISR** (0x00): Interrupt Status Register (read pending interrupts)
- **IER** (0x08): Interrupt Enable Register (enable interrupt sources)
- **IAR** (0x0C): Interrupt Acknowledge Register (clear interrupts, write-only)
- **MER** (0x1C): Master Enable Register (enable IRQ to processor)
