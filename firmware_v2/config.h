#ifndef CONFIG_H
#define CONFIG_H

#define USART1_BAUDRATE 115200

#define BUCK_BOOST_PERIOD 200
#define BUCK_BOOST_AVERAGE_SAMPLES 10000

#define VOLTAGE_TO_ADC(voltage) ((int)((voltage) * 4095 * 1600 / (2.5 * (7500 + 1600)) + 0.5))
#define ADC_TO_VOLTAGE(adc) (((adc) - 0.5) * (2.5 * (7500 + 1600)) / (4095 * 1600))

// Based on 1.2V per amp
#define ADC_TO_CURRENT(adc_value) (((adc_value) * 2.5 / 4096) / 1.2)
#define CURRENT_TO_ADC(current) ((uint16_t)(((current) * 1.2 / 2.5) * 4096))

#define MAX_SOLAR_POWER 500000

#define SINGLE_CELL_LOW_THRESHOLD VOLTAGE_TO_ADC(3.6f)
#define VBATT_LOW_THRESHOLD VOLTAGE_TO_ADC(11.0f)
#define VBATT_MAX_THRESHOLD VOLTAGE_TO_ADC(12.1f)
#define VSOLAR_START_CHARGING_THRESHOLD VOLTAGE_TO_ADC(3.0f)

#define MIN_BALANCE_DIFF_THRESHOLD VOLTAGE_TO_ADC(0.2f)

#endif
