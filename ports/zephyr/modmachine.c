/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2023 Damien P. George
 * Copyright (c) 2016 Paul Sokolovsky
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

// This file is never compiled standalone, it's included directly from
// extmod/modmachine.c via MICROPY_PY_MACHINE_INCLUDEFILE.

#include <stdio.h>
#include <zephyr/sys/reboot.h>

#include "modmachine.h"

#if defined(CONFIG_BCM2835_FIRMWARE)
#include <zephyr/drivers/firmware/bcm2835.h>
#endif

#ifdef CONFIG_REBOOT
#define MICROPY_PY_MACHINE_RESET_ENTRY { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&machine_reset_obj) },
#else
#define MICROPY_PY_MACHINE_RESET_ENTRY
#endif

// machine.freq() is wired only on boards whose SoC exposes a CPU clock
// query -- here the BCM2835/2710 VideoCore firmware. Other Zephyr boards
// keep machine.freq() absent rather than reporting a bogus value.
#if defined(CONFIG_BCM2835_FIRMWARE)
#define MICROPY_PY_MACHINE_FREQ_ENTRY { MP_ROM_QSTR(MP_QSTR_freq), MP_ROM_PTR(&machine_freq_zephyr_obj) },
#else
#define MICROPY_PY_MACHINE_FREQ_ENTRY
#endif

#define MICROPY_PY_MACHINE_EXTRA_GLOBALS \
    MICROPY_PY_MACHINE_RESET_ENTRY \
    MICROPY_PY_MACHINE_FREQ_ENTRY \
    { MP_ROM_QSTR(MP_QSTR_reset_cause), MP_ROM_PTR(&machine_reset_cause_obj) }, \
    { MP_ROM_QSTR(MP_QSTR_Pin), MP_ROM_PTR(&machine_pin_type) }, \
    { MP_ROM_QSTR(MP_QSTR_RTC), MP_ROM_PTR(&machine_rtc_type) }, \
    { MP_ROM_QSTR(MP_QSTR_Timer), MP_ROM_PTR(&machine_timer_type) }, \

static mp_obj_t machine_reset(void) {
    sys_reboot(SYS_REBOOT_COLD);
    // Won't get here, Zephyr has infiniloop on its side
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(machine_reset_obj, machine_reset);

static mp_obj_t machine_reset_cause(void) {
    printf("Warning: %s is not implemented\n", __func__);
    return MP_OBJ_NEW_SMALL_INT(42);
}
MP_DEFINE_CONST_FUN_OBJ_0(machine_reset_cause_obj, machine_reset_cause);

#if defined(CONFIG_BCM2835_FIRMWARE)
// CPU clock in Hz, queried from the VideoCore firmware (ARM clock).
// Getter only -- the ARM core frequency is owned by VC firmware. The
// object is named *_zephyr_obj to avoid colliding with extmod's own
// machine_freq_obj, which modmachine.h always declares (as a different
// function-object type) even on ports without MACHINE_BARE_METAL_FUNCS.
static mp_obj_t machine_freq_zephyr(void) {
    uint32_t hz = 0;
    int err = bcm2835_property_get_clock_rate(BCM2835_CLOCK_ARM, &hz);
    if (err != 0) {
        mp_raise_OSError(-err);
    }
    return mp_obj_new_int_from_uint(hz);
}
MP_DEFINE_CONST_FUN_OBJ_0(machine_freq_zephyr_obj, machine_freq_zephyr);
#endif

static void mp_machine_idle(void) {
    k_yield();
}
