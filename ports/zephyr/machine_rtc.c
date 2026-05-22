/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 jetpax
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

// Software RTC. There is no battery-backed RTC on this SoC, so the wall clock
// is a runtime offset (mp_hal_wall_clock_offset_ms) added to the monotonic
// uptime. machine.RTC().datetime(...) sets that offset; the time module reads
// it back through mp_hal_wall_time_ms(). ntptime.settime() drives the set.

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "shared/timeutils/timeutils.h"
#include "extmod/modmachine.h"

#include <zephyr/kernel.h>

typedef struct _machine_rtc_obj_t {
    mp_obj_base_t base;
} machine_rtc_obj_t;

static const machine_rtc_obj_t machine_rtc_obj = {{&machine_rtc_type}};

static mp_obj_t machine_rtc_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return MP_OBJ_FROM_PTR(&machine_rtc_obj);
}

// datetime() -> 8-tuple; datetime((y,mon,d,wday,h,min,s,subsec)) -> set.
// Tuple order matches CPython-style machine.RTC across ports; the weekday
// field is ignored on set (derived) and computed on get.
static mp_obj_t machine_rtc_datetime(size_t n_args, const mp_obj_t *args) {
    if (n_args == 1) {
        timeutils_struct_time_t tm;
        timeutils_seconds_since_epoch_to_struct_time(mp_hal_wall_time_ms() / 1000, &tm);
        mp_obj_t tuple[8] = {
            mp_obj_new_int(tm.tm_year),
            mp_obj_new_int(tm.tm_mon),
            mp_obj_new_int(tm.tm_mday),
            mp_obj_new_int(tm.tm_wday),
            mp_obj_new_int(tm.tm_hour),
            mp_obj_new_int(tm.tm_min),
            mp_obj_new_int(tm.tm_sec),
            mp_obj_new_int(0),
        };
        return mp_obj_new_tuple(8, tuple);
    }

    mp_obj_t *items;
    mp_obj_get_array_fixed_n(args[1], 8, &items);
    mp_timestamp_t epoch_s = timeutils_seconds_since_epoch(
        mp_obj_get_int(items[0]),    // year
        mp_obj_get_int(items[1]),    // month
        mp_obj_get_int(items[2]),    // day
        mp_obj_get_int(items[4]),    // hour
        mp_obj_get_int(items[5]),    // minute
        mp_obj_get_int(items[6]));   // second
    mp_int_t subsec_ms = mp_obj_get_int(items[7]);
    mp_hal_wall_clock_offset_ms = (int64_t)epoch_s * 1000 + subsec_ms - k_uptime_get();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_rtc_datetime_obj, 1, 2, machine_rtc_datetime);

static const mp_rom_map_elem_t machine_rtc_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_datetime), MP_ROM_PTR(&machine_rtc_datetime_obj) },
};
static MP_DEFINE_CONST_DICT(machine_rtc_locals_dict, machine_rtc_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_rtc_type,
    MP_QSTR_RTC,
    MP_TYPE_FLAG_NONE,
    make_new, machine_rtc_make_new,
    locals_dict, &machine_rtc_locals_dict
    );
