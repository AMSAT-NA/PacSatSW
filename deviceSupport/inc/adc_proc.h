/*
 * adc_proc.c
 *
 *  Created on: Dec 3, 2025
 *      Author: Corey Minyard AE5KM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef ADC_PROC
#define ADC_PROC

typedef void (*adc_handler_func)(unsigned int pin, unsigned int value);

/* Set up and do the first conversion. */
bool init_adc_proc(void);

/* Start a new conversion. */
void adc_start_conversion(void);

/* Process data from a conversion. */
void adc_process_data(void);

/*
 * Install a handler for a particular ADC pin.  Pass in a NULL func to
 * disable.
 */
void adc_install_handler(unsigned int pin, adc_handler_func func);

enum adc_pins {
    ADC_PIN_VER0 = 20,
    ADC_PIN_VER1 = 21,
    ADC_PIN_VER2 = 14,
    ADC_PIN_VER3 = 15,

    ADC_PIN_PA_TEMP = 23,
    ADC_PIN_POWER_TEMP = 8,
    ADC_PIN_CPU_TEMP = 16,

    ADC_PIN_PWR_FLAG_5V = 10,
    ADC_PIN_PWR_FLAG_LNA = 11,
    ADC_PIN_PWR_FLAG_SSPA = 4,
    ADC_PIN_PWR_FLAG_AX5043 = 7,

    ADC_PIN_VOLTAGE_3v3 = 6, // Measures at half voltage
    ADC_PIN_VOLTAGE_1v2 = 13,
    ADC_PIN_VOLTAGE_5v = 12, // Measures at half voltage
    ADC_PIN_VOLTAGE_BATTERY = 1, // Unknown for now, resistors not there.

    ADC_PIN_FORWARD_POWER = 3,
    ADC_PIN_REVERSE_POWER = 2,

    ADC_PIN_EXTERN_CONTROL = 18,
    ADC_PIN_BOARD_NUM = 17,
};

enum temperature_values {
    TEMPERATURE_VAL_CPU,
    TEMPERATURE_VAL_POWER,
    TEMPERATURE_VAL_PA,

    NUM_TEMPERATURE_VALUES /* Must be last. */
};
extern int board_temps[NUM_TEMPERATURE_VALUES];

enum voltage_values {
    VOLTAGE_VAL_3v3,
    VOLTAGE_VAL_1v2,
    VOLTAGE_VAL_5v,
    VOLTAGE_VAL_BATTERY,

    NUM_VOLTAGE_VALUES /* Must be last. */
};
extern int board_voltages[NUM_VOLTAGE_VALUES];

/*
 * Power good flags from the various power control devices.
 */
enum power_flag_values {
    POWER_FLAG_5V,
    POWER_FLAG_LNA,
    POWER_FLAG_SSPA,
    POWER_FLAG_AX5043,

    NUM_POWER_FLAG_VALUES /* Must be last. */
};
extern bool board_power_flags[NUM_POWER_FLAG_VALUES];

/* The version number of the board read from the version resistors. */
extern unsigned int board_version;

/*
 * The board number for fault tolerance (0, 1 or 2).  Zero means it is
 * a simplex board, no mate board was detected.
 */
extern unsigned int board_num;

#endif
