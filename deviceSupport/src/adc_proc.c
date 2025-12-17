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

#include "pacsat.h"
#include "interTaskNotify.h"
#include "adc.h"
#include "adc_proc.h"

static volatile enum {
    ADC_IDLE,
    ADC_IN_CONVERSION,
    ADC_IN_PROCESSING,
} adc_conv_state;

static adcData_t adc_data[24];
static adc_handler_func adc_handlers[24];

void
adc_install_handler(unsigned int pin, adc_handler_func func)
{
    adc_handlers[pin] = func;
}

unsigned int conv_adc_to_millivolts(unsigned int val)
{
    /* 3.3 volt reference. */
    return (val * 3300) / 4096;
}

void
adc_process_data(void)
{
    uint32 count, i;

    count = adcGetData(adcREG1, adcGROUP1, adc_data);
    for (i = 0; i < count; i++) {
        unsigned int pin = adc_data[i].id;

        if (adc_handlers[pin])
            adc_handlers[pin](pin, conv_adc_to_millivolts(adc_data[i].value));
    }
    adc_conv_state = ADC_IDLE;
}

void
adc_start_conversion(void)
{
    if (adc_conv_state != ADC_IDLE)
        return;
    adcStopConversion(adcREG1, adcGROUP1);
    adc_conv_state = ADC_IN_CONVERSION;
    adcStartConversion(adcREG1, adcGROUP1);
}

void
adcNotification(adcBASE_t *adc, uint32 group)
{
    Intertask_Message statusMsg;

    adc_conv_state = ADC_IN_PROCESSING;

    statusMsg.MsgType = TacADCProcessMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

int board_temps[NUM_TEMPERATURE_VALUES];

/*
 * Convert from degrees C to millivolts.
 *
 * Table is for a NTCG103JF103FTDS NTC thermsistor.
 */
#define NUM_TEMPERATURE_CONV_VALUES 166
static uint16 temp_conv[NUM_TEMPERATURE_CONV_VALUES] = {
    3134, 3125, 3116, 3106, 3096, 3086, 3075, 3064, 3053, 3041, // -40
    3028, 3015, 3002, 2988, 2973, 2958, 2943, 2927, 2910, 2893, // -30
    2876, 2858, 2839, 2820, 2800, 2780, 2759, 2738, 2716, 2694, // -20
    2671, 2647, 2623, 2599, 2574, 2549, 2523, 2496, 2470, 2443, // -10
    2415, 2387, 2358, 2330, 2301, 2271, 2241, 2211, 2181, 2151, //   0
    2120, 2089, 2058, 2026, 1995, 1964, 1932, 1901, 1870, 1838, //  10
    1806, 1775, 1743, 1712, 1681, 1650, 1619, 1589, 1558, 1528, //  20
    1498, 1468, 1439, 1410, 1381, 1352, 1324, 1296, 1269, 1241, //  30
    1215, 1188, 1162, 1137, 1112, 1087, 1063, 1038, 1015, 992,  //  40
    969, 947, 925, 904, 883, 862, 842, 822, 803, 784,           //  50
    765, 747, 729, 712, 695, 678, 662, 646, 631, 616,           //  60
    601, 587, 573, 559, 546, 532, 520, 507, 495, 483,           //  70
    472, 461, 450, 439, 428, 418, 408, 399, 389, 380,           //  80
    371, 362, 354, 346, 337, 330, 322, 315, 307, 300,           //  90
    293, 287, 280, 274, 267, 261, 255, 250, 244, 238,           // 100
    233, 228, 223, 218, 213, 208, 204, 199, 195, 191,           // 110
    186, 182, 179, 175, 171, 167                                // 120
};

static int
millivolts_to_temp(unsigned int mv)
{
    int np, pos1 = 0, pos2 = NUM_TEMPERATURE_CONV_VALUES - 1;

    /* Binary search the table. */
    for (;;) {
        np = (pos2 + pos1) / 2;

        if (pos2 - pos1 <= 1) {
            int avg = (temp_conv[pos1] + temp_conv[pos2]) / 2;

            np = pos2;
            if (mv > avg)
                np = pos1;
            break;
        }
        if (mv > temp_conv[np])
            pos2 = np;
        else if (mv < temp_conv[np])
            pos1 = np;
        else
            break;
    }
    return np - 40;
}

static void
handle_adc_temp(unsigned int pin, unsigned int millivolts)
{
    int temp = millivolts_to_temp(millivolts);

    // TODO - Add handling for the values going out of range.
    switch (pin) {
    case ADC_PIN_PA_TEMP:
        board_temps[TEMPERATURE_VAL_PA] = temp;
        break;

    case ADC_PIN_POWER_TEMP:
        board_temps[TEMPERATURE_VAL_POWER] = temp;
        break;

    case ADC_PIN_CPU_TEMP:
        board_temps[TEMPERATURE_VAL_CPU] = temp;
        break;

    default:
        break;
    }
}

int board_voltages[NUM_VOLTAGE_VALUES];

static void
handle_adc_voltage(unsigned int pin, unsigned int millivolts)
{
    // TODO - Add handling for the values going out of range.
    switch (pin) {
    case ADC_PIN_VOLTAGE_3v3:
	millivolts *= 2;
	board_voltages[VOLTAGE_VAL_3v3] = millivolts;
	break;

    case ADC_PIN_VOLTAGE_5v:
	millivolts *= 2;
	board_voltages[VOLTAGE_VAL_5v] = millivolts;
	break;
    
    case ADC_PIN_VOLTAGE_1v2:
	board_voltages[VOLTAGE_VAL_1v2] = millivolts;
	break;

    case ADC_PIN_VOLTAGE_BATTERY:
	board_voltages[VOLTAGE_VAL_BATTERY] = millivolts;
	break;

    default:
	break;
    }
}

bool board_power_flags[NUM_POWER_FLAG_VALUES];

static void
handle_adc_power_flag(unsigned int pin, unsigned int millivolts)
{
    // TODO - Add handling for the the flags going bad.
    switch (pin) {
    case ADC_PIN_PWR_FLAG_5V:
	board_power_flags[POWER_FLAG_5V] = millivolts > 1650;
	break;

    case ADC_PIN_PWR_FLAG_LNA:
	board_power_flags[POWER_FLAG_LNA] = millivolts > 1650;
	break;

    case ADC_PIN_PWR_FLAG_SSPA:
	board_power_flags[POWER_FLAG_SSPA] = millivolts > 1650;
	break;

    case ADC_PIN_PWR_FLAG_AX5043:
	board_power_flags[POWER_FLAG_AX5043] = millivolts > 1650;
	break;

    default:
	break;
    }
}

bool print_rf_power;

static void
handle_adc_rf_power(unsigned int pin, unsigned int millivolts)
{
    switch (pin) {
    case ADC_PIN_FORWARD_POWER:
	if (print_rf_power)
	    printf("Forward power: %d\n", millivolts);
	break;

    case ADC_PIN_REVERSE_POWER:
	if (print_rf_power)
	    printf("Reverse power: %d\n", millivolts);
	break;
    }
}

unsigned int board_version;
unsigned int board_num;

static void
read_adc_board_version(void)
{
    board_version =
        (adc_data[ADC_PIN_VER0].value > 2048)
        | ((adc_data[ADC_PIN_VER1].value > 2048) << 1)
        | ((adc_data[ADC_PIN_VER2].value > 2048) << 2)
        | ((adc_data[ADC_PIN_VER3].value > 2048) << 3);

#ifdef AFSK_HARDWARE
    if (GPIOIsOn(OtherPresense))
	board_num = (adc_data[ADC_PIN_BOARD_NUM].value > 2048) + 1;
#endif

    printf("Board version is %d, board number %d\n", board_version, board_num);
}

bool
init_adc_proc(void)
{
    bool count;

    adc_start_conversion();
    for (count = 0;
         count < 20 && !adcIsConversionComplete(adcREG1, adcGROUP1);
         count++)
        vTaskDelay(CENTISECONDS(1));
    if (!adcIsConversionComplete(adcREG1, adcGROUP1)) {
            debug_print("ADC conversion failed\n");
            return false;
    }

    adc_process_data();
    read_adc_board_version();

    adcEnableNotification(adcREG1, adcGROUP1);

#ifdef AFSK_HARDWARE
    adc_install_handler(ADC_PIN_CPU_TEMP, handle_adc_temp);
    adc_install_handler(ADC_PIN_POWER_TEMP, handle_adc_temp);
    adc_install_handler(ADC_PIN_PA_TEMP, handle_adc_temp);

    adc_install_handler(ADC_PIN_VOLTAGE_3v3, handle_adc_voltage);
    adc_install_handler(ADC_PIN_VOLTAGE_1v2, handle_adc_voltage);
    adc_install_handler(ADC_PIN_VOLTAGE_5v, handle_adc_voltage);
    //adc_install_handler(ADC_PIN_VOLTAGE_BATTERY, handle_adc_voltage);

    adc_install_handler(ADC_PIN_PWR_FLAG_5V, handle_adc_power_flag);
    adc_install_handler(ADC_PIN_PWR_FLAG_LNA, handle_adc_power_flag);
    adc_install_handler(ADC_PIN_PWR_FLAG_SSPA, handle_adc_power_flag);
    adc_install_handler(ADC_PIN_PWR_FLAG_AX5043, handle_adc_power_flag);

    adc_install_handler(ADC_PIN_FORWARD_POWER, handle_adc_rf_power);
    adc_install_handler(ADC_PIN_REVERSE_POWER, handle_adc_rf_power);
#endif

    return true;
}
