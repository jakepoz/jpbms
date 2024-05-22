#include <stdio.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h> // Include this header for __WFI

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

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

#define USART1_PORT GPIOA
#define USART1_TX_PIN GPIO9
#define USART1_RX_PIN GPIO10

#define BALANCE_PORT GPIOB
#define BALANCE_1_PIN GPIO7
#define BALANCE_2_PIN GPIO8
#define BALANCE_3_PIN GPIO9

#define LED_ON_PORT GPIOB
#define LED_ON_PIN GPIO4

#define LED_ACTIVE_PORT GPIOB
#define LED_ACTIVE_PIN GPIO5

#define LED_ERROR_PORT GPIOB
#define LED_ERROR_PIN GPIO6

#define VBATT_ADC_PORT GPIOA
#define VBATT_ADC_PIN GPIO0
#define VBATT_ADC_CH 0

#define VSOLAR_ADC_PORT GPIOA
#define VSOLAR_ADC_PIN GPIO1
#define VSOLAR_ADC_CH 1

#define SOLAR_CURRENT_ADC_PORT GPIOA
#define SOLAR_CURRENT_ADC_PIN GPIO2
#define SOLAR_CURRENT_ADC_CH 2

#define CELL_4V_ADC_PORT GPIOA
#define CELL_4V_ADC_PIN GPIO3
#define CELL_4V_ADC_CH 3

#define CELL_8V_ADC_PORT GPIOA
#define CELL_8V_ADC_PIN GPIO4
#define CELL_8V_ADC_CH 4

#define BB_SW_A_PORT GPIOA
#define BB_SW_A_PIN GPIO15

#define BB_D_A_PORT GPIOB
#define BB_D_A_PIN GPIO3

#define BB_SW_B_PORT GPIOB
#define BB_SW_B_PIN GPIO10

#define BB_D_B_PORT GPIOB
#define BB_D_B_PIN GPIO11

#define ADC_BUFFER_SIZE 3
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

volatile uint32_t system_secs;
extern void initialise_monitor_handles(void);

// The MSI at 2.1Mhz is the default startup clock speed



static inline __attribute__((always_inline)) void __wfi(void)
{
    __asm volatile ("wfi");
}

void sys_tick_handler(void)
{
//    gpio_clear(LED_ERROR_PORT, LED_ERROR_PIN);
//    gpio_set(LED_ERROR_PORT, LED_ERROR_PIN);
//    gpio_toggle(BALANCE_PORT, BALANCE_3_PIN);

    system_secs++;
}

static void enter_sleep_mode(void) {
//    PWR_CR |= PWR_CR_PDDS;
//    SCB_SCR |= SCB_SCR_SLEEPDEEP;

    __asm__("wfi");
}


/* Set up a timer to create 1mS ticks. */
static void init_systick(void)
{
    /* clock rate / 1000 to get 1sec interrupt rate */
    systick_set_reload(rcc_ahb_frequency - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_clear();
    systick_counter_enable();
    systick_interrupt_enable();
}

static void init_gpios(void) {
    rcc_periph_clock_enable(RCC_GPIOB);

    // Setup the LEDs
    gpio_clear(GPIOB, LED_ERROR_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR_PIN);


//    // Setup the Charge/discharge gates
//    gpio_clear(GATE_PORT, GATE_DISCHARGE_PIN | GATE_CHARGE_PIN);
//    gpio_mode_setup(GATE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GATE_DISCHARGE_PIN | GATE_CHARGE_PIN);
//

    // Setup the balance pins
    gpio_clear(BALANCE_PORT, BALANCE_1_PIN | BALANCE_2_PIN | BALANCE_3_PIN);
    gpio_mode_setup(BALANCE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BALANCE_1_PIN | BALANCE_2_PIN | BALANCE_3_PIN);
}

static void init_leds(void) {
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

    // Enable TIM22 output compare channels
    timer_enable_oc_output(TIM22, TIM_OC1);
    timer_enable_oc_output(TIM22, TIM_OC2);

    // Enable the timer counter
    timer_enable_counter(TIM22);
}

static void init_usart(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    /* Setup GPIO pins for USART1 transmit and receive. */
    gpio_mode_setup(USART1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART1_RX_PIN | USART1_TX_PIN);

    /* Setup USART1 TX pin as alternate function. */
    gpio_set_af(USART1_PORT, GPIO_AF4, USART1_RX_PIN | USART1_TX_PIN);

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

//void dma1_channel1_isr(void)
//{
//    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
//        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
//        gpio_toggle(LED_ERROR_PORT, LED_ERROR_PIN);
//    }
//}

void adc_comp_isr(void)
{
    if (adc_eos(ADC1)) {
        ADC_ISR(ADC1) = ADC_ISR_EOS;

    }
}

static void init_adc(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_DMA);

    ADC_CCR(ADC1) |= ADC_CCR_LFMEN;
    ADC_CFGR2(ADC1) |= ADC_CFGR2_CKMODE_PCLK;

    gpio_mode_setup(VSOLAR_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VSOLAR_ADC_PIN);
    gpio_mode_setup(VBATT_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VBATT_ADC_PIN);
    gpio_mode_setup(SOLAR_CURRENT_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, SOLAR_CURRENT_ADC_PIN);
    gpio_mode_setup(CELL_4V_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CELL_4V_ADC_PIN);
    gpio_mode_setup(CELL_8V_ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CELL_8V_ADC_PIN);

    adc_power_off(ADC1);
    adc_calibrate(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_continuous_conversion_mode(ADC1);

    adc_power_on(ADC1);

    // Longest possible sampling, since we have a high input impedance
    // But to be fair, maybe its too long?
    ADC_SMPR1(ADC1) = ADC_SMPR_SMP_160DOT5CYC;


    // Select which channels to read
//    ADC_CHSELR(ADC1) |= ADC_CHSELR_CHSEL(VBATT_ADC_CH) | ADC_CHSELR_CHSEL(VSOLAR_ADC_CH) |
//                        ADC_CHSELR_CHSEL(SOLAR_CURRENT_ADC_CH) |
//                        ADC_CHSELR_CHSEL(CELL_4V_ADC_CH) | ADC_CHSELR_CHSEL(CELL_8V_ADC_CH);
    ADC_CHSELR(ADC1) |= ADC_CHSELR_CHSEL(VBATT_ADC_CH) |
                        ADC_CHSELR_CHSEL(CELL_4V_ADC_CH) | ADC_CHSELR_CHSEL(CELL_8V_ADC_CH);

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
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    ADC_IER(ADC1) |= ADC_IER_EOSIE;
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);

    adc_enable_dma_circular_mode(ADC1);
    adc_enable_dma(ADC1);

    // Start converting in a loop into the buffer
    adc_start_conversion_regular(ADC1);
}

static void init_buck_boost(void) {
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


    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM2, TIM_OC1);
    timer_enable_oc_output(TIM2, TIM_OC1);
    timer_set_oc_value(TIM2, TIM_OC1, 5);

    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM2, TIM_OC2);
    timer_enable_oc_output(TIM2, TIM_OC2);
    timer_set_oc_value(TIM2, TIM_OC2, 5);

    timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM2, TIM_OC3);
    timer_enable_oc_output(TIM2, TIM_OC3);
    timer_set_oc_value(TIM2, TIM_OC3, 0);

    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM2, TIM_OC4);
    timer_enable_oc_output(TIM2, TIM_OC4);
    timer_set_oc_value(TIM2, TIM_OC4, 0);

    timer_set_period(TIM2, 200);
    timer_enable_counter(TIM2);

}

int _write(int fd, char *ptr, int len)
{
    int i = 0;

    /*
     * Write "len" of char from "ptr" to file id "fd"
     * Return number of char written.
     *
     * Only work for STDOUT, STDIN, and STDERR
     */
    if (fd > 2) {
        return -1;
    }
    while (*ptr && (i < len)) {
        usart_send_blocking(USART1, *ptr);
        if (*ptr == '\n') {
            usart_send_blocking(USART1, '\r');
        }
        i++;
        ptr++;
    }
    return i;
}

static float read_vbatt(void) {
    return adc_buffer[0] * 2.5f / 4095 * (220.0f + 47.5f) / 47.5f;
}

static float read_cell4v(void) {
    return adc_buffer[1] * 2.5f / 4095 * (220.0f + 47.5f) / 47.5f;
}

static float read_cell8v(void) {
    return adc_buffer[2] * 2.5f / 4095 * (220.0f + 47.5f) / 47.5f;
}

int main(void) {
    init_gpios();
    init_leds();
    init_systick();
    init_usart();

    init_adc();
    init_buck_boost();


    // Sleep mode tests
    rcc_periph_clock_enable(RCC_PWR);
    pwr_disable_backup_domain_write_protect();

    // Lowest possible internal voltage scaling to save power, limits to 4mhz clock
    //pwr_set_vos_scale(PWR_SCALE3);

//    initialise_monitor_handles();
    printf("starting\n");


    while (1) {
        float c1 = read_cell4v();
        float c2 = read_cell8v() - c1;
        float c3 = read_vbatt() - c2 - c1;

        //printf("%f %f %f\n", c1, c2, c3);



        gpio_toggle(LED_ERROR_PORT, LED_ERROR_PIN);

        uint32_t millis = systick_get_value() * 1000 / systick_get_reload();
//        timer_set_oc_value(TIM22, TIM_OC1, (millis % 1000 > 500) ? 1000 - (millis % 1000) : millis % 1000);
//        timer_set_oc_value(TIM22, TIM_OC2, millis % 500);

        //enter_sleep_mode();
    }
}
