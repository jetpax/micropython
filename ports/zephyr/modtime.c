/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2016 Linaro Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <zephyr/kernel.h>

#include "py/obj.h"
#include "py/mphal.h"
#include "shared/timeutils/timeutils.h"

static mp_obj_t mp_time_time_get(void) {
    /* The absence of FP support is deliberate. The Zephyr port uses
     * single precision floats so the fraction component will start to
     * lose precision on devices with a long uptime.
     */
    return mp_obj_new_int(mp_hal_wall_time_ms() / 1000);
}

// Wall-clock time: uptime plus the offset set by machine.RTC().datetime(...).
// Before an RTC/NTP set, the offset is 0 so this still reads as uptime.
static void mp_time_localtime_get(timeutils_struct_time_t *tm) {
    timeutils_seconds_since_epoch_to_struct_time(mp_hal_wall_time_ms() / 1000, tm);
}
