#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include "usart.h"
#include "pins.h"
#include "config.h"

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


void init_usart(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    /* Setup GPIO pins for USART1 transmit and receive. */
    gpio_mode_setup(USART1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART1_RX_PIN | USART1_TX_PIN);

    /* Setup USART1 TX pin as alternate function. */
    gpio_set_af(USART1_PORT, GPIO_AF4, USART1_RX_PIN | USART1_TX_PIN);

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, USART1_BAUDRATE);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
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


int _close(int file __attribute__((unused))) {
    return -1;
}

int _fstat(int file __attribute__((unused)), struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file __attribute__((unused))) {
    return 1;
}

int _lseek(int file __attribute__((unused)), int ptr __attribute__((unused)), int dir __attribute__((unused))) {
    return 0;
}

int _read(int file __attribute__((unused)), char *ptr __attribute__((unused)), int len __attribute__((unused))) {
    errno = EIO;
    return -1;
}
