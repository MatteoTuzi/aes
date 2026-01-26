#include <xil_io.h>

/* --- Base addresses extracted from SYSTEM.HDF --- */
#define GPIO_LED_BASE   0x40000000U  // axi_gpio_0 - LED (output)
#define GPIO_SW_BASE    0x40010000U  // axi_gpio_1 - Switches (input)
#define GPIO_2_BASE     0x40020000U  // axi_gpio_2

/* --- GPIO register offsets --- */
#define GPIO_DATA       0x00        // Data register
#define GPIO_TRI        0x04        // Tri-state/direction register
                                    // Bit=1: input (tri-state), Bit=0: output

/* -------------------------------------------------------------------------
   MAIN
   Polling implementation: periodically checks the input switches
   and updates the LEDs with the current input values.
   
   Reads from GPIO_1 (switches) and writes to GPIO_0 (LEDs).
   Uses only the 4 least significant bits (0xF) for switches and LEDs.
   ------------------------------------------------------------------------- */
int main() {
    volatile uint32_t sw_value;
    uint32_t last_value = 0x0;  // Store last switch state

    // Configure GPIO_SW (GPIO_1): 4 bits as input (to read switches)
    Xil_Out32(GPIO_SW_BASE + GPIO_TRI, 0xF);
    
    // Configure GPIO_LED (GPIO_0): 4 bits as output (to control LEDs)
    Xil_Out32(GPIO_LED_BASE + GPIO_TRI, 0x0);
    
    // Initialize LEDs to 0
    Xil_Out32(GPIO_LED_BASE + GPIO_DATA, 0x0);

    // Polling loop: periodically checks inputs and updates LEDs only when value changes
    while(1) {
        // Read current value from switches (GPIO_1) - only 4 LSB bits
        // volatile ensures that hardware memory is always read
        sw_value = Xil_In32(GPIO_SW_BASE + GPIO_DATA) & 0xF;
        
        // Update LEDs only when switch value has changed
        if (sw_value != last_value) {
            // Update LEDs (GPIO_0) with the latest changed value (only 4 LSB bits)
            Xil_Out32(GPIO_LED_BASE + GPIO_DATA, sw_value & 0xF);
            last_value = sw_value;
        }
    }

    return 0;
}
