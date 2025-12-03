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
};

enum temperature_values {
    TEMPERATURE_VAL_CPU,
    TEMPERATURE_VAL_POWER,
    TEMPERATURE_VAL_PA,

    NUM_TEMPERATURE_VALUES /* Must be last. */
};
extern int board_temps[NUM_TEMPERATURE_VALUES];

extern unsigned int board_version;

#endif
