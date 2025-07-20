/*
 * Author: Kiran Jojare
 * Project: Embedded Interview Grind - Day 1
 * Date: July 20, 2025
 * Topic: Software Debouncing of a Mechanical Push Button using Timer Interrupt (Bare-Metal C)
 * Target: STM32F401RE (bare-metal, no HAL)
 * Description:
 *   This code demonstrates how to debounce a mechanical button using Timer2
 *   and EXTI interrupt on STM32F4. It avoids false triggers caused by bouncing
 *   by disabling the GPIO interrupt and verifying the button state after 20ms.
 */

#include "stm32f4xx.h"  // Device header (needed for register definitions)

// Global flag to indicate a valid button press
volatile uint8_t button_pressed_flag = 0;

/*
 * Initializes PA0 as input with internal pull-up
 */
void GPIO_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;         // Enable clock for GPIOA
    GPIOA->MODER &= ~(0x3 << (0 * 2));           // Set PA0 to input mode
    GPIOA->PUPDR &= ~(0x3 << (0 * 2));           // Clear PUPDR bits
    GPIOA->PUPDR |=  (0x1 << (0 * 2));           // Enable pull-up for PA0
}

/*
 * Initializes EXTI0 (external interrupt on PA0) for falling edge
 */
void EXTI0_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;        // Enable SYSCFG clock
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;  // Map EXTI0 to PA0

    EXTI->IMR |= EXTI_IMR_IM0;                   // Unmask EXTI0
    EXTI->FTSR |= EXTI_FTSR_TR0;                 // Trigger on falling edge (button press)
    NVIC_EnableIRQ(EXTI0_IRQn);                  // Enable EXTI0 IRQ in NVIC
}

/*
 * Initializes Timer2 for a 20ms delay (used for debounce)
 * Prescaler and ARR are calculated based on 16 MHz system clock
 */
void Timer2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;          // Enable Timer 2 clock
    TIM2->PSC = 16000 - 1;                       // Prescaler = 16000 -> 1 kHz counting frequency
    TIM2->ARR = 20 - 1;                          // Auto-reload = 20ms
    TIM2->DIER |= TIM_DIER_UIE;                  // Enable update interrupt
    TIM2->CR1 &= ~TIM_CR1_CEN;                   // Don't start timer yet
    NVIC_EnableIRQ(TIM2_IRQn);                   // Enable Timer2 IRQ
}

/*
 * EXTI0 Interrupt Handler - Called on falling edge (button press)
 * Disables EXTI temporarily and starts debounce timer
 */
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;                  // Clear pending bit

        EXTI->IMR &= ~EXTI_IMR_IM0;              // Disable EXTI0 interrupt
        TIM2->CNT = 0;                           // Reset timer counter
        TIM2->CR1 |= TIM_CR1_CEN;                // Start debounce timer
    }
}

/*
 * Timer2 Interrupt Handler - Called after 20ms debounce time
 * Validates button press and re-enables EXTI
 */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;                 // Clear interrupt flag
        TIM2->CR1 &= ~TIM_CR1_CEN;               // Stop timer

        if (!(GPIOA->IDR & GPIO_IDR_ID0)) {      // Check if PA0 is still LOW (button pressed)
            button_pressed_flag = 1;             // Valid button press confirmed
        }

        EXTI->IMR |= EXTI_IMR_IM0;               // Re-enable EXTI0 interrupt
    }
}

/*
 * Main application loop
 * Checks for valid button press and acts on it
 */
int main(void) {
    GPIO_init();
    EXTI0_init();
    Timer2_init();

    while (1) {
        if (button_pressed_flag) {
            // === Application Action: LED toggle, print, etc ===
            // Example: Toggle PA5 (user LED)
            // GPIOA->ODR ^= (1 << 5); 

            button_pressed_flag = 0;             // Clear flag
        }
    }
}

/*
 * Summary of Learnings:
 * ----------------------
 * - Why software debouncing is needed (mechanical contact bounces)
 * - Importance of disabling EXTI temporarily to avoid multiple triggers
 * - Use of Timer2 to wait out the bouncing period (20ms)
 * - Validating button state after debounce interval
 * - Register-level initialization of GPIO, EXTI, and Timer on STM32F4
 * - Using ISR handlers for real-time response
 */
