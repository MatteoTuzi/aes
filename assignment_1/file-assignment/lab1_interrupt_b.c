#include <xil_io.h>
#include <mb_interface.h>
#include "xil_exception.h"
#include "sleep.h"

/* --- Base addresses extracted from SYSTEM.HDF --- */
#define INTC_BASEADDR   0x41200000U  // axi_intc_0 - Interrupt Controller
#define GPIO_LED_BASE   0x40000000U  // axi_gpio_0 - LEDs (output)
#define GPIO_SW_BASE    0x40010000U  // axi_gpio_1 - Switches (input) - IRQ0
#define GPIO_BTN_BASE   0x40020000U  // axi_gpio_2 - Buttons (input) - IRQ1

/* --- Interrupt Controller register offsets --- */
#define INTC_ISR        0x00        // Interrupt Status Register - read which interrupts are active
#define INTC_IER        0x08        // Interrupt Enable Register - enable selected interrupts
#define INTC_IAR        0x0C        // Interrupt Acknowledge Register - clear interrupt requests (write-only)
#define INTC_MER        0x1C        // Master Enable Register - enable IRQ request to processor

/* --- GPIO register offsets --- */
#define GPIO_DATA       0x00        // Data register
#define GPIO_TRI        0x04        // Tri-state/direction register
#define GPIO_GIER       0x11C       // Global Interrupt Enable Register - enable interrupt request to INTC
#define GPIO_IPISR      0x120       // IP Interrupt Status Register - Toggle-On-Write operation
#define GPIO_IPIER      0x128       // IP Interrupt Enable Register - enable interrupts in each channel

/* --- Interrupt source masks --- */
#define IRQ_SW_MASK     0x1U        // Switch interrupt (IRQ0) - bit 0
#define IRQ_BTN_MASK    0x2U        // Button interrupt (IRQ1) - bit 1
#define IRQ_ALL_MASK    0x3U        // Both interrupts

/* --- Student ID configuration --- */
/* Last digit of student ID: 70/90/00490 */
#define STUDENT_ID_LAST_DIGIT    0U  // Last digit of student ID (0-9)

/* --- Function prototypes --- */
static void InitializeInterrupts(void);
static void InitializeGpio(void);
static int FindMostSignificantBit(uint32_t value);

/* -------------------------------------------------------------------------
   INTERRUPT HANDLER (ISR) - Part 2
   Handles two interrupt sources:
   - IRQ0 (Switch): Finds the most significant bit set and displays its index on LEDs
   - IRQ1 (Button): Adds the last digit of student ID to current LED pattern and negates
   ------------------------------------------------------------------------- */
void myISR (void) __attribute__ ((interrupt_handler));

void myISR (void) {
    volatile uint32_t pending_interrupts;
    uint32_t sw_value;
    uint32_t led_value;
    uint32_t result;
    int msb_index;

    // Read which interrupt sources are pending
    pending_interrupts = Xil_In32(INTC_BASEADDR + INTC_ISR);

    // Handle Switch interrupt (IRQ0)
    if (pending_interrupts & IRQ_SW_MASK) {
        // Read current switch values (4 LSB bits)
        sw_value = Xil_In32(GPIO_SW_BASE + GPIO_DATA) & 0xF;

        // Find the index of the most significant bit that is set
        msb_index = FindMostSignificantBit(sw_value);

        // Display MSB index on LEDs (0-3), or 0 if no switch is ON
        if (msb_index >= 0) {
            Xil_Out32(GPIO_LED_BASE + GPIO_DATA, (uint32_t)msb_index & 0xF);
        } else {
            Xil_Out32(GPIO_LED_BASE + GPIO_DATA, 0x0);
        }

        // Acknowledge interrupt: first in GPIO device, then in INTC
        // GPIO acknowledge (Toggle-On-Write)
        Xil_Out32(GPIO_SW_BASE + GPIO_IPISR, 0x1);
        
        // INTC acknowledge (write-only register)
        Xil_Out32(INTC_BASEADDR + INTC_IAR, IRQ_SW_MASK);
    }

    // Handle Button interrupt (IRQ1) - Part 2: Add student ID last digit and negate
    if (pending_interrupts & IRQ_BTN_MASK) {
        // Software debounce: wait 50ms to filter mechanical bounce
        usleep(50000);

        // Read current LED value (4 LSB bits)
        led_value = Xil_In32(GPIO_LED_BASE + GPIO_DATA) & 0xF;
        
        // Add the last digit of student ID to current LED pattern
        // Result is masked to 4 bits to prevent overflow
        result = (led_value + STUDENT_ID_LAST_DIGIT) & 0xF;
        
        // Negate (invert) the result
        result = (~result) & 0xF;
        
        // Write the final value to LEDs
        Xil_Out32(GPIO_LED_BASE + GPIO_DATA, result);

        // Acknowledge interrupt: first in GPIO device, then in INTC
        // GPIO acknowledge (Toggle-On-Write)
        Xil_Out32(GPIO_BTN_BASE + GPIO_IPISR, 0x1);
        
        // INTC acknowledge (write-only register)
        Xil_Out32(INTC_BASEADDR + INTC_IAR, IRQ_BTN_MASK);
    }
}

/* -------------------------------------------------------------------------
   MAIN
   Initialize system and wait for interrupt events.
   ------------------------------------------------------------------------- */
int main() {
    // Initialize GPIO directions and clear any pending interrupts
    InitializeGpio();
    
    // Configure interrupt controller and GPIO interrupt enables
    InitializeInterrupts();

    // Register interrupt handler with exception system
    // This is required to tell the system which function to call on interrupt
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(
        XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)myISR,
        NULL
    );
    Xil_ExceptionEnable();

    // Enable interrupts in MicroBlaze processor
    // Sets the Interrupt Enable (IE) bit in Machine Status Register (MSR)
    microblaze_enable_interrupts();

    // Main loop: processor waits for interrupt events
    // When interrupts occur, myISR() is automatically called
    while(1) {
        // Processor can perform other tasks here
        // All input handling is done via interrupts
    }

    return 0;
}

/* -------------------------------------------------------------------------
   InitializeGpio
   Configures GPIO direction registers and initializes outputs.
   ------------------------------------------------------------------------- */
static void InitializeGpio(void) {
    // Configure switches (GPIO_1) as input - 4 bits
    Xil_Out32(GPIO_SW_BASE + GPIO_TRI, 0xF);
    
    // Configure buttons (GPIO_2) as input - 4 bits
    Xil_Out32(GPIO_BTN_BASE + GPIO_TRI, 0xF);
    
    // Configure LEDs (GPIO_0) as output - 4 bits
    Xil_Out32(GPIO_LED_BASE + GPIO_TRI, 0x0);
    
    // Initialize LEDs to all OFF
    Xil_Out32(GPIO_LED_BASE + GPIO_DATA, 0x0);
}

/* -------------------------------------------------------------------------
   InitializeInterrupts
   Configures interrupt controller and enables interrupts in GPIO devices.
   First disables all interrupts, then enables them using OR-mask operations.
   ------------------------------------------------------------------------- */
static void InitializeInterrupts(void) {
    // Clear any pending interrupts in GPIO devices (Toggle-On-Write)
    Xil_Out32(GPIO_SW_BASE + GPIO_IPISR, 0x1);
    Xil_Out32(GPIO_BTN_BASE + GPIO_IPISR, 0x1);
    
    // Clear any pending interrupts in INTC
    Xil_Out32(INTC_BASEADDR + INTC_IAR, IRQ_ALL_MASK);

    // Disable all interrupt sources before configuration
    Xil_Out32(GPIO_SW_BASE + GPIO_GIER, 0x0);
    Xil_Out32(GPIO_BTN_BASE + GPIO_GIER, 0x0);
    Xil_Out32(GPIO_SW_BASE + GPIO_IPIER, 0x0);
    Xil_Out32(GPIO_BTN_BASE + GPIO_IPIER, 0x0);
    Xil_Out32(INTC_BASEADDR + INTC_IER, 0x0);
    Xil_Out32(INTC_BASEADDR + INTC_MER, 0x0);

    // Enable GPIO-level interrupts using OR-mask (preserves other bits)
    // Enable global interrupt generation in switch GPIO (bit 31)
    Xil_Out32(GPIO_SW_BASE + GPIO_GIER, 0x80000000U);
    
    // Enable channel interrupt for switches (bit 0)
    Xil_Out32(GPIO_SW_BASE + GPIO_IPIER, 0x1U);

    // Enable global interrupt generation in button GPIO (bit 31)
    Xil_Out32(GPIO_BTN_BASE + GPIO_GIER, 0x80000000U);
    
    // Enable channel interrupt for buttons (bit 0)
    Xil_Out32(GPIO_BTN_BASE + GPIO_IPIER, 0x1U);

    // Configure and enable INTC using OR-mask
    // Enable interrupt sources individually (IRQ0 for switches, IRQ1 for buttons)
    Xil_Out32(INTC_BASEADDR + INTC_IER, IRQ_SW_MASK | IRQ_BTN_MASK);
    
    // Enable interrupts globally in INTC
    // Bit 0: Hardware Interrupt Enable (HIE)
    // Bit 1: Master Interrupt Enable (ME)
    Xil_Out32(INTC_BASEADDR + INTC_MER, 0x3U);
}

/* -------------------------------------------------------------------------
   FindMostSignificantBit
   Finds the index (0-3) of the most significant bit set in a 4-bit value.
   Returns -1 if no bits are set.
   ------------------------------------------------------------------------- */
static int FindMostSignificantBit(uint32_t value) {
    int i;
    
    // Check bits from MSB (bit 3) to LSB (bit 0)
    for (i = 3; i >= 0; i--) {
        if (value & (1U << i)) {
            return i;  // Return index of first set bit found
        }
    }
    
    return -1;  // No bits set
}
