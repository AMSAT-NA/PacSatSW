/*
 * Max31331Rtc.c
 *
 *  Created on: Aug 5, 2023
 *      Author: Chris Thompson G0KLA
 *
 * Portions of this code are based on the Analog Devices example code and
 * subject to:
 *
* Copyright(C) Analog Devices Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files(the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Analog Devices Inc.
* shall not be used except as stated in the Analog Devices Inc.
* Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Analog Devices Inc.retains all ownership rights.
********************************************************************************
 *
 * The remainder of this code is Open Source:
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
#include <pacsat.h>
#include "Max31331Rtc.h"
#include "i2cDriver.h"
#include <time.h>

#define BCD2BIN(val) (((val) & 15) + ((val) >> 4) * 10)
#define BIN2BCD(val) ((((val) / 10) << 4) + (val) % 10)
#define SWAPBYTES(val)  (((val & 0xFF) << 8) | ((val & 0xFF00) >> 8))

static uint8_t buf[2],send;

int8_t hours_reg_to_hour(const max3133x_hours_reg_t *hours_reg);
void rtc_regs_to_time(struct tm *time, const max3133x_rtc_time_regs_t *regs, uint16_t *sub_sec);
int time_to_rtc_regs(max3133x_rtc_time_regs_t *regs, const struct tm *time, hour_format_t format);
void to_12hr(uint8_t hr, uint8_t *hr_12, uint8_t *pm);

/**
 * POSIX definition of seconds since the Epoch
 * https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap04.html#tag_04_15
 *
 * The last three terms of the expression add in a day for each year that follows a leap year
 * starting with the first leap year since the Epoch. The first term adds a day every 4 years
 * starting in 1973, the second subtracts a day back out every 100 years starting in 2001, and
 * the third adds a day back in every 400 years starting in 2001. The divisions in the formula
 * are integer divisions; that is, the remainder is discarded leaving only the integer quotient.
 *
 * This is tested to produce the same result as the TI function call:
 * mktime(&tm_time) - 2208988800L + 6 * 60 * 60
 *
 */
uint32_t rtc_mktime(struct rtc_tm *tm_time) {
    uint32_t t = tm_time->tm_sec + tm_time->tm_min*60 + tm_time->tm_hour*3600 + tm_time->tm_yday*86400 +
        (tm_time->tm_year-70)*31536000 + ((tm_time->tm_year-69)/4)*86400 -
        ((tm_time->tm_year-1)/100)*86400 + ((tm_time->tm_year+299)/400)*86400;
    return t;
}

/**
 * Need to clear the reset bit and enable the oscillator or it will not tick
 *
 */
bool InitRtc31331(void){
    buf[0] = MAX31331_RTC_RESET;
    buf[1] = 0x00; // reset bit cleared
    if (!I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,buf,2,0,0)) {
        debug_print("Error clearing reset of RTC\n");
        return FALSE;
    }

    vTaskDelay(CENTISECONDS(1));

    buf[0] = MAX31331_RTC_CONFIG1;
    buf[1] = 0x03; // Disable DIN pin, enable oscillator and timeout
    if (!I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,buf,2,0,0)) return FALSE;

    vTaskDelay(CENTISECONDS(1));

    buf[0] = MAX31331_RTC_CONFIG2;
    buf[1] = 0x04; // set INTB/CLKOUT pin to Clockout and set Clock out to 1Hz
    if (!I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,buf,2,0,0)) return FALSE;

    vTaskDelay(CENTISECONDS(1));

    buf[0] = MAX31331_TRICKLE_REG;
    buf[1] = 0x01; // Enable trickle charge through 3k and schottky diode
    if (!I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,buf,2,0,0)) return FALSE;

    return TRUE;
    }

bool GetStatus31331(uint8_t *cfg){
    buf[0] = MAX31331_STATUS;
    return I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,buf,1,cfg,1);
}
bool GetRtcTime31331(uint32_t *time){
    int8_t regs[20];
    send = 0;//MAX31331_SECONDS;
    if (!I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,&send,1,&regs,20)) return FALSE;
//    int j;
//    debug_print("Time: ");
//    for (j=0;j<20;j++)
//    debug_print("%0x ",regs[j]);
//    debug_print("\n");

    max3133x_rtc_time_regs_t *time_regs = (max3133x_rtc_time_regs_t*) &regs[MAX31331_SECONDS_1_128]; // offset to the first time register

    struct tm tm_time;
    rtc_regs_to_time(&tm_time, time_regs, NULL);

    debug_print("Date: %d-%d-%d %02d:%02d:%02d UTC\n",(tm_time.tm_year+1900), tm_time.tm_mon+1, tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);

    *time = mktime(&tm_time) - 2208988800L + 6 * 60 * 60;  // Adjust because TI Time library used Epoch of 1-1-1900 UTC - 6. Time protocol in RFC 868 specifies offset as 2208988800L
    return TRUE;
}

bool SetRtcTime31331(uint32_t *unixtime) {
    max3133x_rtc_time_regs_t regs;
    struct tm *time;

    if (*unixtime < 1691675756) { // 10 Aug 2023 because that is when I wrote this line
        debug_print("Unix time seems to be in the past!");
        return FALSE;
    }
    time_t t  = (time_t)(*unixtime + 2208988800L - 6 * 60 * 60); // Adjust because TI Time library used Epoch of 1-1-1900 UTC - 6
    time = gmtime(&t);
    if (time == NULL) return FALSE;

//    char buf[30];
//    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", time);
//    debug_print(" trying to set RTC to time: %s\n", buf);


    int rc = time_to_rtc_regs(&regs, time, HOUR24);
    if (rc != MAX3133X_NO_ERR) return FALSE;

    /* TODO - this is a bit of a hack to use our I2C function.  We don't use sub seconds and it is the
     * first register in the structure.  So we write the address of the SECONDS register as the raw
     * value.  This will be sent as the first byte and the address.  Then the seconds value and all
     * subsequent values will follow.  */
    regs.seconds_1_128_reg.raw = MAX31331_SECONDS;
    if (!I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,&regs,sizeof(max3133x_rtc_time_regs_t),0,0)) return FALSE;

    return TRUE;
}

int8_t hours_reg_to_hour(const max3133x_hours_reg_t *hours_reg) {
    hour_format_t format = hours_reg->bits_24hr.f_24_12 == 1 ? HOUR12 : HOUR24;
    if (format == HOUR24) {
        return BCD2BIN(hours_reg->bcd_24hr.value);
    } else {
        if (hours_reg->bits_12hr.am_pm) {
            if (hours_reg->bcd_12hr.value < 12)
                return (hours_reg->bcd_12hr.value + 12);
        } else {
            if (hours_reg->bcd_12hr.value == 12)
                return (hours_reg->bcd_12hr.value - 12);
        }
    }
    return -1;
}


void rtc_regs_to_time(struct tm *time, const max3133x_rtc_time_regs_t *regs, uint16_t *sub_sec) {
    if (sub_sec != NULL)
        *sub_sec = (1000 * regs->seconds_1_128_reg.raw) / 128.0;

    /* tm_sec seconds [0,61] */
    time->tm_sec = BCD2BIN(regs->seconds_reg.bcd.value);

    /* tm_min minutes [0,59] */
    time->tm_min = BCD2BIN(regs->minutes_reg.bcd.value);

    /* tm_hour hour [0,23] */
    time->tm_hour = hours_reg_to_hour(&regs->hours_reg);
    /*hour_format_t format = regs->hours_reg.bits_24hr.f_24_12 == 1 ? HOUR12 : HOUR24;
    if (format == HOUR24) {
        time->tm_hour = BCD2BIN(regs->hours_reg.bcd_24hr.value);
    } else if (format == HOUR12) {
        uint8_t hr24 = to_24hr(BCD2BIN(regs->hours_reg.bcd_12hr.value), regs->hours_reg.bits_12hr.am_pm);
        time->tm_hour = hr24;
    }*/

    /* tm_wday day of week [0,6] (Sunday = 0) */
    time->tm_wday = BCD2BIN(regs->day_reg.bcd.value) - 1;

    /* tm_mday day of month [1,31] */
    time->tm_mday = BCD2BIN(regs->date_reg.bcd.value);

    /* tm_mon month of year [0,11] */
    time->tm_mon = BCD2BIN(regs->month_reg.bcd.value) - 1;

    /* tm_year years since 2000 */
    if (regs->month_reg.bits.century)
        time->tm_year = BCD2BIN(regs->year_reg.bcd.value) + 200;
    else
        time->tm_year = BCD2BIN(regs->year_reg.bcd.value) + 100;

    /* tm_yday day of year [0,365] */
    time->tm_yday = 0; /* TODO */

    /* tm_isdst daylight savings flag */
    time->tm_isdst = 0; /* TODO */
}

int time_to_rtc_regs(max3133x_rtc_time_regs_t *regs, const struct tm *time, hour_format_t format) {
    /*********************************************************
     * +----------+------+---------------------------+-------+
     * | Member   | Type | Meaning                   | Range |
     * +----------+------+---------------------------+-------+
     * | tm_sec   | int  | seconds after the minute  | 0-61* |
     * | tm_min   | int  | minutes after the hour    | 0-59  |
     * | tm_hour  | int  | hours since midnight      | 0-23  |
     * | tm_mday  | int  | day of the month          | 1-31  |
     * | tm_mon   | int  | months since January      | 0-11  |
     * | tm_year  | int  | years since 1900          |       |
     * | tm_wday  | int  | days since Sunday         | 0-6   |
     * | tm_yday  | int  | days since January 1      | 0-365 |
     * | tm_isdst | int  | Daylight Saving Time flag |       |
     * +----------+------+---------------------------+-------+
     * * tm_sec is generally 0-59. The extra range is to accommodate for leap
     *   seconds in certain systems.
     *********************************************************/
    regs->seconds_reg.bcd.value = BIN2BCD(time->tm_sec);

    regs->minutes_reg.bcd.value = BIN2BCD(time->tm_min);

    if (format == HOUR24) {
        regs->hours_reg.bcd_24hr.value = BIN2BCD(time->tm_hour);
        regs->hours_reg.bits_24hr.f_24_12 = HOUR24;
    } else if (format == HOUR12) {
        uint8_t hr_12, pm;
        to_12hr(time->tm_hour, &hr_12, &pm);
        regs->hours_reg.bcd_12hr.value = BIN2BCD(hr_12);
        regs->hours_reg.bits_12hr.f_24_12 = HOUR12;
        regs->hours_reg.bits_12hr.am_pm = pm;
    } else {
        debug_print("RTC: Invalid Hour Format!\n");
        return MAX3133X_INVALID_TIME_ERR;
    }

    regs->day_reg.bcd.value = BIN2BCD(time->tm_wday + 1);

    regs->date_reg.bcd.value = BIN2BCD(time->tm_mday);

    regs->month_reg.bcd.value = BIN2BCD(time->tm_mon + 1);

    if (time->tm_year >= 200) {
        regs->month_reg.bits.century = 1;
        regs->year_reg.bcd.value = BIN2BCD(time->tm_year - 200);
    } else if (time->tm_year >= 100) {
        regs->month_reg.bits.century = 0;
        regs->year_reg.bcd.value = BIN2BCD(time->tm_year - 100);
    } else {
        debug_print("RTC: Invalid set date!\n");
        return MAX3133X_INVALID_DATE_ERR;
    }

    return MAX3133X_NO_ERR;
}

void to_12hr(uint8_t hr, uint8_t *hr_12, uint8_t *pm) {
    if (hr == 0) {
        *hr_12 = 12;
        *pm = 0;
    } else if (hr < 12) {
        *hr_12 = hr;
        *pm = 0;
    } else if (hr == 12) {
        *hr_12 = 12;
        *pm = 1;
    } else {
        *hr_12 = hr - 12;
        *pm = 1;
    }
}
