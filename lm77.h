/*
    lm77.h - Part of lm_sensors, Linux kernel modules for hardware
             monitoring
    Copyright (c) 2005 Michael Renzmann <mrenzmann@otaku42.de>
    
    Heavily based on lm75.h from Mark M. Hoffman <mhoffman@lightlink.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/*
    This file contains common code for encoding/decoding LM77 type
    temperature readings.
*/

#include <linux/i2c-proc.h>

/* straight from the datasheet */
#define LM77_TEMP_MIN (-550)
#define LM77_TEMP_MAX 1250

/* In the temperature registers the lowest 3 bits are not part of the
 * temperature values (either unused or representing alarm status,
 * depending on the register). 
 */
static inline s16 LM77_TEMP_TO_REG(int temp)
{
	int ntemp = SENSORS_LIMIT(temp, LM77_TEMP_MIN, LM77_TEMP_MAX);
	return (ntemp / 5) * 8;
}

static inline int LM77_TEMP_FROM_REG(s16 reg)
{
        return (reg / 8) * 5;
}
