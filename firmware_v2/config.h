#ifndef CONFIG_H
#define CONFIG_H

#define USART1_BAUDRATE 115200

#define BUCK_BOOST_PERIOD 200
#define BUCK_BOOST_AVERAGE_SAMPLES 5000

#define VOLTAGE_TO_VBATT_ADC(voltage) ((int)((voltage) * 4095 * 1600 / (2.5 * (7575 + 1600)) + 0.5))
#define VBATT_ADC_TO_VOLTAGE(adc) (((adc) - 0.5) * (2.5 * (7575 + 1600)) / (4095 * 1600))

#define VOLTAGE_TO_VSOLAR_ADC(voltage) ((int)((voltage) * 4095 * 2 / (2.5 * (18.0 + 2.0)) + 0.5))
#define VSOLAR_ADC_TO_VOLTAGE(adc) (((adc) - 0.5) * (2.5 * (18.0 + 2.0)) / (4095 * 2.0))

// An approximation to VOLTAGE_TO_VSOLAR_ADC(1) / VOLTAGE_TO_VBATT_ADC(1)
#define VSOLAR_TO_VBATT_RATIO_NUM 100
#define VSOLAR_TO_VBATT_RATIO_DEN 57

#define VREFINT_CAL MMIO16(0x1FF80078)

#define VDDA_DATASHEET_VOLTAGE (3.0)
// This is the ratio between our own 2.5V and the 3V that the ADC was calibrated at
#define VDDA_CONVERSION_RATIO_NUM 6
#define VDDA_CONVERSION_RATIO_DEN 5


// Based on 1.2V per amp
#define ADC_TO_CURRENT(adc_value) (((adc_value) * 2.5 / 4095) / 1.2)
#define CURRENT_TO_ADC(current) ((uint16_t)(((current) * 1.2 / 2.5) * 4095))

#define MAX_SOLAR_POWER 500000

#define SINGLE_CELL_LOW_THRESHOLD VOLTAGE_TO_VBATT_ADC(3.6f)
#define SINGLE_CELL_NOT_PRESENT_THRESHOLD VOLTAGE_TO_VBATT_ADC(0.15f)
#define VBATT_LOW_THRESHOLD VOLTAGE_TO_VBATT_ADC(11.0f)
#define VBATT_MAX_THRESHOLD VOLTAGE_TO_VBATT_ADC(12.1f)

#define VSOLAR_START_CHARGING_THRESHOLD VOLTAGE_TO_VSOLAR_ADC(15.0f)
#define VSOLAR_STOP_CHARGING_THRESHOLD VOLTAGE_TO_VSOLAR_ADC(10.0f)

// If we are measuring anything too close to 4095, which is the max, then we should just shutdown
#define VSOLAR_MEASUREMENT_ERROR_THRESHOLD 4090

#define MIN_BALANCE_DIFF_THRESHOLD VOLTAGE_TO_VBATT_ADC(0.1f)

#endif
