/**
 * Project template
 * RT & Embedded Systems course material
 *
 * Prepared by: Jakub Mnich (jakub.mnich@pwr.edu.pl)
 * Wrocław University of Science and Technology
 * March 2022
 *
 * Works with STM32F407VGT6 (STM32F407G-DISC1)
 ******************************************************************************
 */

/*
 * Note:
 * "stm32xxxxxx.h" are header files delivered by the MCU manufacturer. These files
 * contain information allowing us to refer to registers by their names rather than
 * their actual addresses in the memory. This makes it much easier to write the
 * code and it will also be more readable.
 */
#include "stm32f407xx.h"

#define LED_PIN_0 12 // PD12
#define LED_PIN_1 13 // PD13
#define LED_PIN_2 14 // PD14

#define TICKS_FOR_CLOCK_CHANGE 8000

void Set_AHB_Frequency(uint8_t setting);

volatile uint32_t tickCount = 0;
volatile uint32_t changeClockTickCounter = 0;
volatile uint8_t clock_setting = 0;
volatile static uint32_t SYSTEM_MS = 0;

void ConfigureSysTick(uint32_t sys_freq);

void ConfigureSysTick(uint32_t sys_freq) {
    SysTick->LOAD = (sys_freq / 2000U) - 1U; // 2ms tick
    SysTick->VAL = 0x00;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

void SysTick_Handler(void) {
    tickCount++;
    SYSTEM_MS++; // Increment SYSTEM_MS on each SysTick interrupt
    changeClockTickCounter++;

    if (tickCount >= 1000) { // 1000 ms delay
        switch(clock_setting) {
            case 0:
                GPIOD->ODR ^= (1 << LED_PIN_0); // Toggle LED 0
                break;
            case 1:
                GPIOD->ODR ^= (1 << LED_PIN_1); // Toggle LED 1
                break;
            case 2:
                GPIOD->ODR ^= (1 << LED_PIN_2); // Toggle LED 2
                break;
        }
        tickCount = 0;
    }

    if (changeClockTickCounter >= TICKS_FOR_CLOCK_CHANGE) {
        clock_setting = (clock_setting + 1) % 3; // Cycle through clock settings (0, 1, 2)
        Set_AHB_Frequency(clock_setting); // Change the system clock
        changeClockTickCounter = 0;
    }
}

uint32_t millis(void) {
    return SYSTEM_MS;
}

void Set_AHB_Frequency(uint8_t setting) {
    // Reset CFGR register
	RCC->CFGR = 0b00;
	RCC->CFGR = 0x0;

    // Turn off HSE, HSI, and PLL
    RCC->CR &= ~(1 << 16); // HSEON = 0
    RCC->CR &= ~(1 << 0);  // HSION = 0
    RCC->CR &= ~(1 << 24); // PLLON = 0

    // Turn off all LEDs before changing the clock setting
    GPIOD->ODR &= ~(1 << LED_PIN_0);
    GPIOD->ODR &= ~(1 << LED_PIN_1);
    GPIOD->ODR &= ~(1 << LED_PIN_2);

    switch(setting) {
        case 0: //  HSI source AHB clk = 16 MHz
        	RCC->CR |= (1 << 0); // Enable HSI
        	while(!(RCC->CR & (1 << 1))); // Wait for HSI ready
        	RCC->CFGR |= (0b0000 << 4); // HPRE: No division (0000)
            RCC->CFGR |= (0b00 << 0); // SW: HSI selected as system clock (00)
            //SystemCoreClock = 16000000; // Update SystemCoreClock variable
            break;

        case 1: // HSE source AHB clk = 4 MHz
        	RCC->CR |= (1 << 16); // Enable HSE
        	while(!(RCC->CR & (1 << 17))); // Wait for HSE ready
        	RCC->CFGR |= (0b1000 << 4); // HPRE: System clock divided by 2 (1000)
        	RCC->CFGR |= (0b01 << 0); // SW: HSE selected as system clock (01)
            //SystemCoreClock = 4000000; // Update SystemCoreClock variable
        	break;

        case 2: // PLL with HSE input, with settings resulting in AHB clk = 168 MHz
            RCC->PLLCFGR |= (1 << 22) | (8 << 0) | (336 << 6) | (0b00 << 16); // PLL source = HSE
            RCC->CR |= (1 << 16); // Enable HSE
            while(!(RCC->CR & (1 << 17)));
            RCC->CR |= (1 << 24); // Enable PLL
            while(!(RCC->CR & (1 << 25))); // Wait for PLL ready
            FLASH->ACR |= (5 << 0); // LATENCY = 5 wait states
            RCC->CFGR |= (0b10 << 0); // SW set to PLL
            while((RCC->CFGR & (0b11 << 2)) != (0b10 << 2)); // Wait until PLL is used
            //SystemCoreClock = 168000000; // Update SystemCoreClock variable
            break;
    }

    //ConfigureSysTick(SystemCoreClock); // Reconfigure SysTick with the new system core clock
}

int main(void) {
	RCC->AHB1ENR |= (1 << 3); // Enable clock for GPIOD
    GPIOD->MODER |= (1 << (2 * LED_PIN_0)) | (1 << (2 * LED_PIN_1)) | (1 << (2 * LED_PIN_2)); // Set PD12, PD13, and PD14 as output

    ConfigureSysTick(16000000); // Initial SysTick setup for HSI clock

    NVIC_SetPriority(SysTick_IRQn, 0); // Set SysTick interrupt priority

    while (1) {
        // Main loop
    }
}
