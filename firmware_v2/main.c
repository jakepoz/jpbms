#include <stdio.h>

#include "pins.h"
#include "config.h"
#include "usart.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h> // Include this header for __WFI

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))


/*!< Low Frequency Mode enable */
// Must be set with ADC Freq < 3.5Mhz, and we run at 2.1 main clock
#define ADC_CCR_LFMEN                       ((uint32_t)0x02000000U)
#define ADC_CFGR2_TOVS                      ((uint32_t)0x80000200U)     /*!< Triggered Oversampling */
#define ADC_CFGR2_OVSS                      ((uint32_t)0x000001E0U)     /*!< OVSS [3:0] bits (Oversampling shift) */
#define ADC_CFGR2_OVSS_0                    ((uint32_t)0x00000020U)     /*!< Bit 0 */
#define ADC_CFGR2_OVSS_1                    ((uint32_t)0x00000040U)     /*!< Bit 1 */
#define ADC_CFGR2_OVSS_2                    ((uint32_t)0x00000080U)     /*!< Bit 2 */
#define ADC_CFGR2_OVSS_3                    ((uint32_t)0x00000100U)     /*!< Bit 3 */
#define ADC_CFGR2_OVSR                      ((uint32_t)0x0000001CU)     /*!< OVSR  [2:0] bits (Oversampling ratio) */
#define ADC_CFGR2_OVSR_0                    ((uint32_t)0x00000004U)     /*!< Bit 0 */
#define ADC_CFGR2_OVSR_1                    ((uint32_t)0x00000008U)     /*!< Bit 1 */
#define ADC_CFGR2_OVSR_2                    ((uint32_t)0x00000010U)     /*!< Bit 2 */
#define ADC_CFGR2_OVSE                      ((uint32_t)0x00000001U)     /*!< Oversampler Enable */

#define ADC_BUFFER_SIZE 5
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint32_t adc_conversions = 0;
volatile uint32_t adc_accum[ADC_BUFFER_SIZE];

volatile uint32_t system_secs;
// The MSI at 2.1Mhz is the default startup clock speed


void sys_tick_handler(void)
{
//    gpio_toggle(LED_ERROR_PORT, LED_ERROR_PIN);

//    gpio_clear(LED_ERROR_PORT, LED_ERROR_PIN);
//    gpio_set(LED_ERROR_PORT, LED_ERROR_PIN);
//    gpio_toggle(BALANCE_PORT, BALANCE_3_PIN);

    system_secs++;
}

static void enter_sleep_mode(void) {
//    PWR_CR |= PWR_CR_PDDS;
//    SCB_SCR |= SCB_SCR_SLEEPDEEP;

   __asm__ volatile ("wfi");
}


/* Set up a timer to create 1mS ticks. */
static void systick_init(void)
{
    systick_counter_disable();

    /* clock rate / 1000 to get 1sec interrupt rate */
    systick_set_reload(rcc_ahb_frequency - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_clear();
    systick_counter_enable();
    systick_interrupt_enable();
}

static void delay_us(uint32_t cycles) {
    // Calculate the number of iterations needed, assuming each iteration is 10 NOPs
    uint32_t iterations = (rcc_ahb_frequency / 1000000) * cycles / 10;

    __asm__ volatile (
            "1: \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "sub %[iterations], %[iterations], #1 \n"
            "cmp %[iterations], #0 \n"
            "bne 1b \n"
            : [iterations] "+r" (iterations) // Output: iterations is modified in place
    );
}


static void gpiob_init(void) {
    rcc_periph_clock_enable(RCC_GPIOB);

    // Setup the LEDs
    gpio_clear(GPIOB, LED_ERROR_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR_PIN);

    // Setup the balance pins
    gpio_clear(BALANCE_PORT, BALANCE_1_PIN | BALANCE_2_PIN | BALANCE_3_PIN);
    gpio_mode_setup(BALANCE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BALANCE_1_PIN | BALANCE_2_PIN | BALANCE_3_PIN);

    // Setup the mosfet drivers output
    gpio_clear(DRIVERS_EN_PORT, DRIVERS_EN_PIN);
    gpio_mode_setup(DRIVERS_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DRIVERS_EN_PIN);
}

static void leds_init(void) {
    rcc_periph_clock_enable(RCC_TIM22);

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_ON_PIN | LED_ACTIVE_PIN);
    gpio_set_af(GPIOB, GPIO_AF4, LED_ON_PIN | LED_ACTIVE_PIN);

    // Set prescaler to 2 to get 1.05 MHz timer frequency
    timer_set_prescaler(TIM22, 2 - 1);

    // Set the period to 1000 to get a ~1 kHz PWM frequency
    timer_set_period(TIM22, 1000 - 1);

    // Configure TIM22_CH1 and TIM22_CH2 as PWM outputs
    timer_set_oc_mode(TIM22, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM22, TIM_OC2, TIM_OCM_PWM1);


}

static void leds_enable(void) {
    timer_enable_oc_output(TIM22, TIM_OC1);
    timer_enable_oc_output(TIM22, TIM_OC2);

    timer_enable_counter(TIM22);
}

static void leds_disable(void) {
    timer_disable_oc_output(TIM22, TIM_OC1);
    timer_disable_oc_output(TIM22, TIM_OC2);

    timer_disable_counter(TIM22);
}

void exti0_1_isr(void) {
    // Check if the interrupt is from EXTI0
    if (exti_get_flag_status(EXTI0)) {
        // Clear the interrupt flag
        exti_reset_request(EXTI0);

        gpio_toggle(LED_ERROR_PORT, LED_ERROR_PIN);
    }
}

static void switch_init(void) {
    // Setup the switch input
    // TODO the datasheet or our calculations were wrong and the pin leakage current is higher than expected
    // So we have to use the built in pullup
    gpio_mode_setup(SWITCH_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, SWITCH_PIN);

    rcc_periph_clock_enable(RCC_SYSCFG);

    // Select the source input for EXTI0
    exti_select_source(EXTI0, SWITCH_PORT);

    // Configure EXTI0 to trigger on falling edge
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);

    // Enable EXTI0 interrupt
    exti_enable_request(EXTI0);

    // Enable the EXTI0 interrupt in the NVIC
    nvic_enable_irq(NVIC_EXTI0_1_IRQ);
}


//void dma1_channel1_isr(void)
//{
//    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
//        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
//        gpio_toggle(LED_ERROR_PORT, LED_ERROR_PIN);
//    }
//}

static volatile bool adc_conversion_finished = false;

void adc_comp_isr(void)
{

    if (adc_eos(ADC1)) {
        ADC_ISR(ADC1) = ADC_ISR_EOS;
        adc_conversion_finished = true;

        for (uint8_t i = 0; i < ADC_BUFFER_SIZE; i++) {
            adc_accum[i] += adc_buffer[i];
        }
        adc_conversions++;
    }

//    __asm__ volatile (
//        // gpio_set(LED_ERROR_PORT, LED_ERROR_PIN)
//            "LDR R0, =%[port_bsrr]\n\t"               // Load the address of LED_ERROR_PORT into R0
//            "LDR R1, =%[pin]\n\t"                // Load the value of LED_ERROR_PIN into R1
//            "STR R1, [R0, #24]\n\t"                   // Store the value of LED_ERROR_PIN at the address in R0 (LED_ERROR_PORT)
//
//            // if (adc_eos(ADC1))
//            "LDR R2, =%[adc]\n\t"                // Load the address of ADC1 into R2
//            "LDR R3, [R2, #0x00]\n\t"            // Load the ADC_ISR register value at ADC1 into R3
//            "LDR R4, =%[eos]\n\t"                // Load the value of ADC_ISR_EOS into R4
//            "TST R3, R4\n\t"                     // Test if EOS bit is set
//            "BEQ 1f\n\t"                         // Branch to label 1 if EOS bit is not set
//            "STR R4, [R2, #0x00]\n\t"            // Store the value of ADC_ISR_EOS into ADC_ISR register
//
//            // adc_conversion_finished = true
//            "LDR R5, =%[conversion_finished]\n\t" // Load the address of adc_conversion_finished into R5
//            "MOVS R6, #1\n\t"                     // Load the value 1 into R6 (true)
//            "STRB R6, [R5]\n\t"                   // Store the value 1 into adc_conversion_finished
//
//            // adc_conversions++
//            "LDR R7, =%[conversions]\n\t"         // Load the address of adc_conversions into R7
//            "LDR R6, [R7]\n\t"                    // Load the current value of adc_conversions into R6
//            "ADD R6, R6, #1\n\t"                  // Increment the value in R6 by 1
//            "STR R6, [R7]\n\t"                    // Store the incremented value back to adc_conversions
//
//            // Increment the accums
//            "LDR R6, =adc_accum\n"     // Load address of adc_accum
//            "LDR R7, =adc_buffer\n"    // Load address of adc_buffer
//
//            "LDRH R1, [R7, #0]\n"      // Load adc_buffer[0]
//            "LDR R2, [R6, #0]\n"      // Load adc_accum[0]
//            "ADD R1, R1, R2\n"        // Add values
//            "STR R1, [R6, #0]\n"      // Store result back to adc_accum[0]
//
//            "LDRH R1, [R7, #2]\n"      // Load adc_buffer[0]
//            "LDR R2, [R6, #4]\n"      // Load adc_accum[0]
//            "ADD R1, R1, R2\n"        // Add values
//            "STR R1, [R6, #4]\n"      // Store result back to adc_accum[1]
//
//            "LDRH R1, [R7, #4]\n"      // Load adc_buffer[0]
//            "LDR R2, [R6, #8]\n"      // Load adc_accum[0]
//            "ADD R1, R1, R2\n"        // Add values
//            "STR R1, [R6, #8]\n"      // Store result back to adc_accum[2]
//
//            // gpio_clear(LED_ERROR_PORT, LED_ERROR_PIN)
//            "1:\n\t"
//            "LDR R0, =%[port_bsrr]\n\t"               // Load the address of LED_ERROR_PORT into R0
//            "LDR R1, =%[pin]\n\t"                // Load the value of LED_ERROR_PIN into R1
//            "LSL R1, R1, #16\n\t"
//            "STR R1, [R0, #24]\n\t"               // Clear the value of LED_ERROR_PIN at the address in R0+4 (LED_ERROR_PORT)
//            :
//            : [port_bsrr] "i" (LED_ERROR_PORT), [pin] "i" (LED_ERROR_PIN),
//    [adc] "i" (ADC1), [eos] "i" (ADC_ISR_EOS),
//    [conversion_finished] "i" (&adc_conversion_finished), [conversions] "i" (&adc_conversions),
//    [adc_accum] "i" (adc_accum), [adc_buffer] "i" (adc_buffer)
//    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
//    );
}

static void adc_init(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_DMA);

    ADC_CCR(ADC1) |= ADC_CCR_LFMEN;
    ADC_CFGR2(ADC1) |= ADC_CFGR2_CKMODE_PCLK;

    gpio_mode_setup(VSOLAR_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VSOLAR_ADC_PIN);
    gpio_mode_setup(VBATT_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VBATT_ADC_PIN);
    gpio_mode_setup(SOLAR_CURRENT_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, SOLAR_CURRENT_ADC_PIN);

    gpio_mode_setup(ADC_MULTIPLEX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN | ADC_MULTIPLEX_VBATT_PIN);

    adc_power_off(ADC1);
    adc_calibrate(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_single_conversion_mode(ADC1);


    adc_power_on(ADC1);
}

static void adc_set_oversample_256(bool oversample) {
    // Configure oversampling
    // Oversampling can only be configured if the adc is off
    if (!!oversample != !!(ADC_CFGR2(ADC1) & ADC_CFGR2_OVSS)) {
        printf("adc restart\n");

        adc_power_off(ADC1);

        ADC_CFGR2(ADC1) &= ~ADC_CFGR2_OVSS;
        ADC_CFGR2(ADC1) &= ~ADC_CFGR2_OVSR;

        if (oversample) {
            ADC_CFGR2(ADC1) |= ADC_CFGR2_OVSR_0 | ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_2; // 256x oversampling
            ADC_CFGR2(ADC1) |= ADC_CFGR2_OVSS_3; // Divide by 256 afterwards;
            ADC_CFGR2(ADC1) |= ADC_CFGR2_OVSE;
        } else {
            ADC_CFGR2(ADC1) &= ~ADC_CFGR2_OVSE;
        }

        adc_power_on(ADC1);
    }
}

static void adc_start_conversion_dma(uint32_t channel_mask, bool oversample) {
    // Set the sample rate
    ADC_SMPR1(ADC1) = ADC_SMPR_SMP_160DOT5CYC;

    // Select which channels to read
    ADC_CHSELR(ADC1) = channel_mask;

    adc_set_oversample_256(oversample);

    // ADC DMA
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_SIZE);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    // Clear end of sequence flag
    adc_conversion_finished = false;

    // Enable interrupt for testing
    ADC_IER(ADC1) |= ADC_IER_EOSIE;
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);


    adc_disable_dma_circular_mode(ADC1);
    adc_enable_dma(ADC1);

    // Start converting in a loop into the buffer
    adc_start_conversion_regular(ADC1);
}

static void adc_begin_convert_tim2_trigger(uint32_t channel_mask) {
    // This function is meant to start ADC conversions on TIM2 triggers
    // on TIM2 overflow, we start a conversion, so we are synced to always get it during the same
    // point in the cycle

    // The result is still DMAed into the same buffer as usual

    // The ADC needs to be powered off to adjust the trigger mechanism
    adc_power_off(ADC1);

    // Set the sample rate
    ADC_SMPR1(ADC1) = ADC_SMPR_SMP_39DOT5CYC;

    // Select which channels to read
    ADC_CHSELR(ADC1) = channel_mask;

    // Turn off oversampling
    ADC_CFGR2(ADC1) &= ~ADC_CFGR2_OVSS;
    ADC_CFGR2(ADC1) &= ~ADC_CFGR2_OVSR;
    ADC_CFGR2(ADC1) &= ~ADC_CFGR2_OVSE;

    // Configure ADC to use TIM2 TRGO event as an external trigger
    ADC_CFGR1(ADC1) |= ADC_CFGR1_EXTSEL_VAL(ADC_CFGR1_EXTSEL_TIM2_TRGO);
    ADC_CFGR1(ADC1) |= ADC_CFGR1_EXTEN_RISING_EDGE;

    adc_power_on(ADC1);

    // Clear end of sequence flag
    ADC_ISR(ADC1) = ADC_ISR_EOS;

    // Enable ADC DMA
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    if (__builtin_popcount(channel_mask) < ADC_BUFFER_SIZE)
        dma_set_number_of_data(DMA1, DMA_CHANNEL1, __builtin_popcount(channel_mask));
    else
        dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_SIZE);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    adc_enable_dma_circular_mode(ADC1);
    adc_enable_dma(ADC1);

    adc_start_conversion_regular(ADC1);
}

static void adc_end_convert_tim2_trigger(void) {
     // This should wait for any conversions to finish
    adc_power_off(ADC1);

    ADC_CFGR1(ADC1) &= ADC_CFGR1_EXTSEL;
    ADC_CFGR1(ADC1) &= ~ADC_CFGR1_EXTEN_BOTH_EDGES;

    adc_power_on(ADC1);
}

/*
 * The problem is that the STM32L053 doesn't have a way to generate PWM pulses with a phase offset.
 * We need to make a pulse on OC1/OC2, then, once that pulse is done, start a pulse on OC3/OC4.
 */
static uint16_t buck_boost_last_duty = 0;

static uint16_t buck_boost_get_duty(void) {
    return buck_boost_last_duty;
}

static void buck_boost_set_duty(uint16_t new_dc, int16_t delta) {
    int32_t dc = (int32_t)new_dc + delta;
    if (dc < 0) {
        dc = 0;
    }
    else if (dc > BUCK_BOOST_PERIOD - 20) {
        dc = BUCK_BOOST_PERIOD - 20;
    }

    buck_boost_last_duty = dc;

    // TODO The high side switch on the battery side is still in asynchronous mode, so we need to set it to something
    timer_set_oc_value(TIM2, TIM_OC1, 0);
    timer_set_oc_value(TIM2, TIM_OC2, BUCK_BOOST_PERIOD - dc);
    timer_set_oc_value(TIM2, TIM_OC3, BUCK_BOOST_PERIOD - dc);

    // Fixed amount to run charge pump
    timer_set_oc_value(TIM2, TIM_OC4, CLAMP(BUCK_BOOST_PERIOD - dc + 5, 5, BUCK_BOOST_PERIOD - 5));
}

static void buck_boost_init(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_af(BB_SW_A_PORT, GPIO_AF5, BB_SW_A_PIN);
    gpio_set_af(BB_D_A_PORT, GPIO_AF2, BB_D_A_PIN);
    gpio_set_af(BB_SW_B_PORT, GPIO_AF2, BB_SW_B_PIN);
    gpio_set_af(BB_D_B_PORT, GPIO_AF2, BB_D_B_PIN);

    gpio_mode_setup(BB_SW_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BB_SW_A_PIN);
    gpio_mode_setup(BB_D_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BB_D_A_PIN);
    gpio_mode_setup(BB_SW_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BB_SW_B_PIN);
    gpio_mode_setup(BB_D_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BB_D_B_PIN);

    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Send out triggers on updates to sync the ADC during charging
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);

    // HA,
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM2, TIM_OC1);
    timer_set_oc_value(TIM2, TIM_OC1, 0);

    // LA
    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_low(TIM2, TIM_OC2);
    timer_set_oc_value(TIM2, TIM_OC2, BUCK_BOOST_PERIOD - 0);

    // HB
    timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_polarity_low(TIM2, TIM_OC3);
    timer_set_oc_value(TIM2, TIM_OC3, BUCK_BOOST_PERIOD - 0);

    // LB
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM2, TIM_OC4);
    timer_set_oc_value(TIM2, TIM_OC4, 0);

    // Also remember that you can only turn on HA or HB if corresponding LA/LB have been on recently
    // Because the charge pump needs to operate to get the gate voltage high enough


    // Set the period to N - 1, because the overflow happens on a match, so it's like a >= loop
    // Then, setting an oc_value to be equal to N means it will never have a pulse
    timer_set_period(TIM2, BUCK_BOOST_PERIOD - 1);

    // Enable update interrupt
//     timer_enable_irq(TIM2, TIM_DIER_UIE);
//     nvic_enable_irq(NVIC_TIM2_IRQ);

}

static void buck_boost_enable(void) {
    timer_enable_oc_output(TIM2, TIM_OC1);
    timer_enable_oc_output(TIM2, TIM_OC2);
    timer_enable_oc_output(TIM2, TIM_OC3);
    timer_enable_oc_output(TIM2, TIM_OC4);
    timer_set_counter(TIM2, 0);
    timer_enable_counter(TIM2);
}

static void buck_boost_disable(void) {
    timer_disable_counter(TIM2);
    timer_disable_oc_output(TIM2, TIM_OC1);
    timer_disable_oc_output(TIM2, TIM_OC2);
    timer_disable_oc_output(TIM2, TIM_OC3);
    timer_disable_oc_output(TIM2, TIM_OC4);
}

// Switches us over to a higher speed clock for more PWM precision
// when running the buck boost
static void buck_boost_enable_clock(void) {
    // You need to start using more power at a higher clock speed
    pwr_set_vos_scale(PWR_SCALE1);

    rcc_osc_on(RCC_HSI16);
    rcc_wait_for_osc_ready(RCC_HSI16);

    flash_prefetch_enable();
    flash_set_ws(FLASH_ACR_LATENCY_1WS);

    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
    rcc_set_ppre1(RCC_CFGR_PPRE1_NODIV);
    rcc_set_ppre2(RCC_CFGR_PPRE2_NODIV);

    rcc_ahb_frequency = 16000000;
    rcc_apb1_frequency = 16000000;
    rcc_apb2_frequency = 16000000;

    // Setup systick again with new AHB frequency
    systick_init();

    // Update the usart because the apb2 frequency has changed
    usart_comm_init();

    // Change the tim22 prescaler
    timer_set_prescaler(TIM22, 15 - 1); // Shooting for a 1 Mhz clock

    rcc_set_sysclk_source(RCC_HSI16);
    while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16);
}

static void buck_boost_disable_clock(void) {
    // Switch back to low frequency / low power operations
    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
    rcc_set_ppre1(RCC_CFGR_PPRE1_NODIV);
    rcc_set_ppre2(RCC_CFGR_PPRE2_NODIV);

    rcc_ahb_frequency = 2097000;
    rcc_apb1_frequency = 2097000;
    rcc_apb2_frequency = 2097000;

    systick_init();

    // Update the usart because the apb2 frequency has changed
    usart_comm_init();

    // Change the tim22 prescaler
    timer_set_prescaler(TIM22, 2 - 1); // Shooting for a 1 Mhz clock

    rcc_set_sysclk_source(RCC_MSI);
    while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_MSI);

    rcc_osc_off(RCC_HSI16);

    flash_prefetch_disable();
    flash_set_ws(FLASH_ACR_LATENCY_0WS);

    pwr_set_vos_scale(PWR_SCALE3);
}

/*
* Startup
    * Set up the most basic peripherals, 2Mhz MSE clock
* Sample
    * Measure VBATT and VSOLAR
    * If VSOLAR > threshold and vbatt needs to charge, goto Charge
    * If VBATT is full, goto BalanceCheck
    * If VBATT below threshold, go to LowBattSleep
* LowBattSleep
    * Flash error led once
    * Turn off all peripherals
    * Wake up after 10 seconds, goto Sample
* NormalSleep
    * Turn off all peripherals
    * Wake up after 2 seconds, goto Sample
* Charge
    * Might need to increase CPU frequency to have more PWM resolution
    * Turn on buckboost
    * Sample VSolar and VBatt at X Hz
    * Need to run MPPT, basically increase/decrease duty cycle to see which way is most power
    * Green LED frequency will show charge state
    * Yellow LED frequency will show solar power input
    * If VBatt is full, goto BalanceCheck
    * If Vsolar drops too low, goto NormalSleep
    * Can still do sleep mode in between measurements, etc.
    * Revert to lower clock speed when leaving this state
* BalanceCheck
    * Turn yellow LED solid on
    * Sample VBATT, CELL4, and CELL8V with many samples
    * If any cell has dangerously low voltage, goto LowBattSleep
    * If one cell is > threshold above both others, goto Balance against highest cell
    * If one cell is < threshold below both others, goto Balance against highest cell
    * Turn off Yellow LED when leaving state, so you have yellow light blinking during balancing
    * Idea, could blink yellow LED N times, depending on which cell is being balanced
    * Or better yet, put an LED on the balancer itself to indicate
* Balance
    * Turn on balance pin
    * Sleep 10 seconds, turn off balance pin, goto BalanceCheck
    *
    *
TODO: Add watchdog timer
TODO: Add power brownout monitor etc
TODO: Add reset on Hardfaults, etc
*/
enum main_state {
    STATE_STARTUP=0,

    STATE_START_SAMPLE,
    STATE_SELECT_SAMPLE_MULTIPLEXER,
    STATE_WAIT_SAMPLE_MULTIPLEXER,
    STATE_WAIT_SAMPLE,
    STATE_SAMPLE,

    STATE_LOW_BATT_SLEEP,
    STATE_NORMAL_SLEEP,

    STATE_START_CHARGE,
    STATE_START_CHARGE_MEASURE_BASELINE,
    STATE_START_CHARGE_MEASURE_BASELINE_WAIT,
    STATE_CHARGE,
    STATE_END_CHARGE,

    STATE_BALANCE,
};

volatile enum main_state cur_state = STATE_STARTUP;
volatile uint8_t cur_balance_cell = 0;
volatile uint32_t cur_state_start_secs = 0;
volatile uint32_t last_balance_check = 0xffffffff;

volatile uint16_t cur_sample_channel = 0;
volatile uint16_t cur_vbatt, cur_8v, cur_4v, cur_vsolar, cur_vrefint;

enum mppt_state {
    MPPT_INITIAL=0,

    MPPT_GOING_UP,
    MPPT_GOING_DOWN,
};
volatile enum mppt_state cur_mppt_state = MPPT_INITIAL;
volatile int32_t last_mppt_power = 0;
volatile uint32_t mppt_ticks = 0;
volatile uint16_t start_charge_vbatt, start_charge_vsolar, start_charge_current;

static void change_state(enum main_state new_state) {
    cur_state = new_state;
    cur_state_start_secs = system_secs;

    // In debug mode, we pulse the LED to indicate which state we are in
//    for (uint8_t i = 0; i < new_state; i++) {
//        gpio_set(LED_ERROR_PORT, LED_ERROR_PIN);
//        gpio_clear(LED_ERROR_PORT, LED_ERROR_PIN);
//    }
}


int main(void) {
    gpiob_init();
    leds_init();
    systick_init();
    usart_comm_init();
    switch_init();

    adc_init();
    buck_boost_init();


    // Sleep mode tests
    rcc_periph_clock_enable(RCC_PWR);
    pwr_disable_backup_domain_write_protect();

    // Lowest possible internal voltage scaling to save power, limits to 4mhz clock
    pwr_set_vos_scale(PWR_SCALE3);

//    while(1) {
//        enter_sleep_mode();
//    }

    printf("startup\n");


    while (1) {
        if (cur_state == STATE_STARTUP) {
            // Give one second to startup, so all voltages can stabilize
            if (system_secs > 1) {
                change_state(STATE_START_SAMPLE);
                //change_state(STATE_START_CHARGE);
            }
        } else if (cur_state == STATE_START_SAMPLE) {
            gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
            cur_sample_channel = 0;

            change_state(STATE_SELECT_SAMPLE_MULTIPLEXER);
        } else if (cur_state == STATE_SELECT_SAMPLE_MULTIPLEXER) {
            if (cur_sample_channel == 0) {
                adc_enable_vrefint();
            }
            else if (cur_sample_channel == 1) {
                adc_disable_vrefint();
                gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
                delay_us(2000); // Need to wait for the multiplexer to switch off the previous voltage or else you'll have two enabled at the same time which shows a distinct current spike
                gpio_set(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN);
            }
            else if (cur_sample_channel == 2) {
                gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
                delay_us(2000); // Need to wait for the multiplexer to switch off the previous voltage or else you'll have two enabled at the same time which shows a distinct current spike
                gpio_set(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_CELL4V_PIN);
            }
            else if (cur_sample_channel == 3) {
                gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
                delay_us(2000); // Need to wait for the multiplexer to switch off the previous voltage or else you'll have two enabled at the same time which shows a distinct current spike
                gpio_set(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_CELL8V_PIN);
            }
            else if (cur_sample_channel == 4) {
                gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
            }


            change_state(STATE_WAIT_SAMPLE_MULTIPLEXER);
        } else if (cur_state == STATE_WAIT_SAMPLE_MULTIPLEXER) {
            delay_us(5000);

            if (cur_sample_channel == 0 )
                adc_start_conversion_dma(ADC_CHSELR_CHSEL(VREFINT_ADC_CH), true);
            else if (cur_sample_channel >= 1 && cur_sample_channel <= 3)
                adc_start_conversion_dma(ADC_CHSELR_CHSEL(VBATT_ADC_CH), true);
            else if (cur_sample_channel == 4)
                adc_start_conversion_dma(ADC_CHSELR_CHSEL(VSOLAR_ADC_CH), true);

            change_state(STATE_WAIT_SAMPLE);
        } else if (cur_state == STATE_WAIT_SAMPLE) {
            if (adc_conversion_finished){
                if (cur_sample_channel == 0) {
                    cur_vrefint = adc_buffer[0];

                    cur_sample_channel++;
                    change_state(STATE_SELECT_SAMPLE_MULTIPLEXER);
                }
                else if (cur_sample_channel == 1) {
                    cur_vbatt = (adc_buffer[0] * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);

                    cur_sample_channel++;
                    change_state(STATE_SELECT_SAMPLE_MULTIPLEXER);
                }
                else if (cur_sample_channel == 2) {
                    cur_4v = (adc_buffer[0] * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);

                    cur_sample_channel++;
                    change_state(STATE_SELECT_SAMPLE_MULTIPLEXER);
                }
                else if (cur_sample_channel == 3) {
                    cur_8v = (adc_buffer[0] * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);

                    cur_sample_channel++;
                    change_state(STATE_SELECT_SAMPLE_MULTIPLEXER);
                }
                else if (cur_sample_channel == 4) {
                    cur_vsolar = (adc_buffer[0] * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);

                    cur_sample_channel++;
                    change_state(STATE_SAMPLE);
                }
            } else {
                enter_sleep_mode();
            }
        } else if (cur_state == STATE_SAMPLE) {
            printf("sample %d %d %d %d %d %d\n", cur_vbatt, cur_8v, cur_4v, cur_vsolar, cur_vrefint, VREFINT_CAL);

            uint16_t cell1 = cur_4v;
            uint16_t cell2 = cur_8v - cur_4v;
            uint16_t cell3 = cur_vbatt - cur_8v;

            if (cur_vbatt < VBATT_LOW_THRESHOLD) {
                printf("low vbatt %d\n", cur_vbatt);
                change_state(STATE_LOW_BATT_SLEEP);
            }
            else if (cell1 < SINGLE_CELL_LOW_THRESHOLD) {
                printf("low cell_1 %d\n", cell1);
                change_state(STATE_LOW_BATT_SLEEP);
            } else if (cell2< SINGLE_CELL_LOW_THRESHOLD) {
                printf("low cell_2 %d\n", cell2);
                change_state(STATE_LOW_BATT_SLEEP);
            } else if (cell3 < SINGLE_CELL_LOW_THRESHOLD) {
                printf("low cell_3 %d\n",cell3);
                change_state(STATE_LOW_BATT_SLEEP);
            } else if (cur_vsolar > VSOLAR_START_CHARGING_THRESHOLD && cur_vbatt < VBATT_MAX_THRESHOLD) {
                change_state(STATE_START_CHARGE);
            } else if (system_secs - last_balance_check > 60) {
                printf("balance %d %d %d\n", cell1, cell2, cell3);
                last_balance_check = system_secs;

                if (cell1 > cell2 && cell1 > cell3 &&
                    ((cell1 - cell2) > MIN_BALANCE_DIFF_THRESHOLD || (cell1 - cell3) > MIN_BALANCE_DIFF_THRESHOLD)) {
                    change_state(STATE_BALANCE);
                    cur_balance_cell = 1;
                } else if (cell2 > cell1 && cell2 > cell3 &&
                           ((cell2 - cell1) > MIN_BALANCE_DIFF_THRESHOLD || (cell2 - cell3) > MIN_BALANCE_DIFF_THRESHOLD)) {
                    change_state(STATE_BALANCE);
                    cur_balance_cell = 2;
                } else if (cell3 > cell1 && cell3 > cell2 &&
                           ((cell3 - cell1) > MIN_BALANCE_DIFF_THRESHOLD || (cell3 - cell2) > MIN_BALANCE_DIFF_THRESHOLD)) {
                    change_state(STATE_BALANCE);
                    cur_balance_cell = 3;
                } else {
                    change_state(STATE_NORMAL_SLEEP);
                }
            }
            else {
                change_state(STATE_NORMAL_SLEEP);
            }

            // Be sure the multiplexer is off when leaving this state
            gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
        } else if (cur_state == STATE_LOW_BATT_SLEEP) {
            //gpio_set(LED_ERROR_PORT, LED_ERROR_PIN);

            enter_sleep_mode();

            if (system_secs - cur_state_start_secs > 30) {
                change_state(STATE_START_SAMPLE);
            }
        } else if (cur_state == STATE_NORMAL_SLEEP) {
            enter_sleep_mode();

            if (system_secs - cur_state_start_secs > 2) {
                change_state(STATE_START_SAMPLE);
            }
        } else if (cur_state == STATE_START_CHARGE) {
            // Set the multiplexer to be measuring VBATT on future conversions
            gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
            gpio_set(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN);

            // Enable the drivers
            gpio_set(DRIVERS_EN_PORT, DRIVERS_EN_PIN);

            // Get a baseline of the solar current, so you can zero-out the sensor properly
            adc_start_conversion_dma(ADC_CHSELR_CHSEL(SOLAR_CURRENT_ADC_CH), true);

            change_state(STATE_START_CHARGE_MEASURE_BASELINE_WAIT);
        } else if (cur_state == STATE_START_CHARGE_MEASURE_BASELINE_WAIT) {
            if (adc_conversion_finished){
                change_state(STATE_START_CHARGE_MEASURE_BASELINE);
            } else {
                enter_sleep_mode();
            }
        } else if (cur_state == STATE_START_CHARGE_MEASURE_BASELINE) {
            start_charge_vsolar = cur_vsolar;
            start_charge_vbatt = cur_vbatt;
            start_charge_current = adc_buffer[0];

            printf("startcharge %d %d %d\n", start_charge_vbatt, start_charge_vsolar, start_charge_current);

            for (uint8_t i = 0; i < ADC_BUFFER_SIZE; i++) {
                adc_buffer[i] = 0;
                adc_accum[i] = 0;
                adc_conversions = 0;
            }

            adc_begin_convert_tim2_trigger(ADC_CHSELR_CHSEL(VBATT_ADC_CH) |
                                           ADC_CHSELR_CHSEL(VSOLAR_ADC_CH) |
                                           ADC_CHSELR_CHSEL(SOLAR_CURRENT_ADC_CH));

            buck_boost_enable_clock();
            buck_boost_enable();

            leds_enable();

            // Start with a small initial_duty cycle
            last_mppt_power = 0;
            cur_mppt_state = MPPT_INITIAL;
            buck_boost_set_duty(25, 0);

            change_state(STATE_CHARGE);
        } else if (cur_state == STATE_CHARGE) {
            // If you are in this state, then you are continously sampling the ADC
            if (adc_conversions >= BUCK_BOOST_AVERAGE_SAMPLES) {
                //gpio_set(LED_ERROR_PORT, LED_ERROR_PIN);

                nvic_disable_irq(NVIC_ADC_COMP_IRQ);
                uint32_t vsolar = adc_accum[0];
                uint32_t vbatt = adc_accum[1];
                int32_t solar_cur = adc_accum[2];
                uint32_t last_adc_conversions = adc_conversions;

                adc_accum[0] = 0;
                adc_accum[1] = 0;
                adc_accum[2] = 0;
                adc_conversions = 0;
                nvic_enable_irq(NVIC_ADC_COMP_IRQ);

                vbatt = ((vbatt / last_adc_conversions) * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);
                vsolar = ((vsolar / last_adc_conversions) * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);
                solar_cur = ((solar_cur / last_adc_conversions - start_charge_current) * VDDA_CONVERSION_RATIO_NUM * VREFINT_CAL) / (VDDA_CONVERSION_RATIO_DEN * cur_vrefint);
                int32_t cur_power = vsolar * solar_cur;

                printf("charge %ld %ld %ld %d %ld %d\n", vbatt, vsolar, solar_cur, buck_boost_get_duty(), cur_power, cur_mppt_state);

                uint32_t target_vsolar = start_charge_vsolar * 88 / 100;

                // VBATT AND VSOLAR are not on the same scale, so you need to adjust the ratio
                uint32_t max_dcm_duty = (BUCK_BOOST_PERIOD * vbatt) / ((vsolar * VSOLAR_TO_VBATT_RATIO_NUM) / VSOLAR_TO_VBATT_RATIO_DEN + vbatt);

                printf("maxduty %ld\n", max_dcm_duty);

                if (vsolar > target_vsolar && buck_boost_get_duty() < max_dcm_duty) {
                    buck_boost_set_duty(buck_boost_get_duty(), +1);
                }
                else if (vsolar < target_vsolar) {
                    buck_boost_set_duty(buck_boost_get_duty(), -1);
                }

                if (vbatt < VBATT_LOW_THRESHOLD || vbatt > VBATT_MAX_THRESHOLD) {
                    change_state(STATE_END_CHARGE);
                }

                if (vsolar < VSOLAR_STOP_CHARGING_THRESHOLD) {
                    change_state(STATE_END_CHARGE);
                }

                if (vsolar > VSOLAR_MEASUREMENT_ERROR_THRESHOLD) {
                    change_state(STATE_END_CHARGE);
                }

                last_mppt_power = cur_power;
                //gpio_clear(LED_ERROR_PORT, LED_ERROR_PIN);
            }

            uint32_t millis = systick_get_value() / (systick_get_reload() / 1000);
            //timer_set_oc_value(TIM22, TIM_OC1, (millis % 1000 > 500) ? 1000 - (millis % 1000) : millis % 1000);
            timer_set_oc_value(TIM22, TIM_OC1, 100);


            timer_set_oc_value(TIM22, TIM_OC2, millis % (((130 - 23) * 1000 + 200) / 130));


            // Periodically we want to stop charging and get a measurement of the system, maybe go into balance, etc.
            if (system_secs - cur_state_start_secs > 300) {
                change_state(STATE_END_CHARGE);
            }

            enter_sleep_mode();
        } else if (cur_state == STATE_END_CHARGE) {
            adc_end_convert_tim2_trigger();
            buck_boost_disable();
            buck_boost_disable_clock();

            gpio_clear(ADC_MULTIPLEX_PORT, ADC_MULTIPLEX_VBATT_PIN | ADC_MULTIPLEX_CELL4V_PIN | ADC_MULTIPLEX_CELL8V_PIN);
            gpio_clear(DRIVERS_EN_PORT, DRIVERS_EN_PIN);

            leds_disable();

            // The adc is going to read a very low value on vbatt on the first sampling after
            // so we want to wait a second before moving to that state
            // We do that by doing a round of sleep first
            change_state(STATE_NORMAL_SLEEP);
        } else if (cur_state == STATE_BALANCE) {
            gpio_clear(BALANCE_PORT, BALANCE_1_PIN | BALANCE_2_PIN | BALANCE_3_PIN);

            if (cur_balance_cell == 1)
                gpio_set(BALANCE_PORT, BALANCE_1_PIN);
            else if (cur_balance_cell == 2)
                gpio_set(BALANCE_PORT, BALANCE_2_PIN);
            else if (cur_balance_cell == 3)
                gpio_set(BALANCE_PORT, BALANCE_3_PIN);

            if (system_secs - cur_state_start_secs > 10) {
                change_state(STATE_START_SAMPLE);
            } else {
                enter_sleep_mode();
            }
        }
    }
}

