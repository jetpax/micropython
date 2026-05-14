/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Mike Teachman
 * Copyright (c) 2026 jetpax
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
// extmod/machine_i2s.c via MICROPY_PY_MACHINE_I2S_INCLUDEFILE.
//
// Backend for the RING_BUF model: extmod owns the ring buffer and the
// blocking / non-blocking / asyncio copy logic; this file bridges that
// ring buffer to Zephyr's I2S API.
//
// Zephyr's I2S driver already owns the DMA, the buffer slab, and the
// completion ISR -- so where the rp2 backend uses a DMA ISR to move data
// between hardware and the ring buffer, this backend uses a worker
// k_thread that calls the blocking i2s_read() / i2s_write() and copies
// to/from the ring buffer. The thread runs at a higher priority than the
// REPL thread so a blocking write()/readinto() (which spins in extmod
// filling/draining the ring buffer) makes progress.
//
// First cut: 16-bit stereo, I2S (Philips) format, master mode -- matching
// the i2s_bcm2835 controller driver. MONO, 32-bit and target (slave)
// clocking are rejected at init.

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>

#include "py/mphal.h"
#include "py/runtime.h"

// I2S.RX / I2S.TX (the Python-visible constants, set in mpconfigport.h)
// are defined equal to Zephyr's enum i2s_dir values, so self->mode can be
// passed straight to i2s_configure() / i2s_trigger() as an i2s_dir.
MP_STATIC_ASSERT(MICROPY_PY_MACHINE_I2S_CONSTANT_RX == I2S_DIR_RX);
MP_STATIC_ASSERT(MICROPY_PY_MACHINE_I2S_CONSTANT_TX == I2S_DIR_TX);

// Number of I2S peripherals. The BCM2710 has one PCM block; other Zephyr
// SoCs would extend i2s_devices[] / the slab + stack arrays.
#define MAX_I2S (1)

// Worker-thread stack. The thread only does i2s_read/write, a ring-buffer
// memcpy and mp_sched_schedule -- but Zephyr's I2S + logging paths can be
// stack-hungry, so keep generous headroom (DRAM is not scarce here).
#define WORKER_STACK_SIZE (4096)

// Zephyr I2S buffer slab. block_size must be a multiple of 64 -- the
// BCM2835 DMA is not cache-coherent and the driver invalidates whole
// cache lines on RX buffers (see drivers/i2s/i2s_bcm2835.c). 2048 bytes
// is 512 frames of 16-bit stereo, ~10.7 ms at 48 kHz.
#define I2S_SLAB_BLOCK_SIZE  (2048)
#define I2S_SLAB_BLOCK_COUNT (4)
#define I2S_SLAB_ALIGN       (64)

// Blocking timeout for i2s_read/i2s_write and slab alloc inside the
// worker loop -- bounds how long deinit waits for the worker to notice
// the stop flag, and how long an i2s call waits on a briefly-full queue.
#define I2S_RW_TIMEOUT_MS (100)

// TX is primed with silence blocks before START: the controller driver
// needs at least one queued block to begin transmitting.
#define I2S_PRIME_BLOCKS (2)

// Per-iteration appbuf <-> ring-buffer copy budget for non-blocking mode.
// Kept several times the per-block hardware rate so the ring buffer does
// not under/overflow between worker iterations. Tunable.
#define SIZEOF_NON_BLOCKING_COPY_IN_BYTES (I2S_SLAB_BLOCK_SIZE * 4)

// id -> Zephyr device. DEVICE_DT_GET_OR_NULL yields NULL when the i2s
// node is absent or disabled; mp_machine_i2s_make_new_instance rejects
// that with a clear error rather than failing the link.
static const struct device *const i2s_devices[MAX_I2S] = {
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2s)),
};

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t base;
    uint8_t i2s_id;
    mp_hal_pin_obj_t sck;
    mp_hal_pin_obj_t ws;
    mp_hal_pin_obj_t sd;
    uint8_t mode;       // I2S_DIR_RX or I2S_DIR_TX
    int8_t bits;
    format_t format;
    int32_t rate;
    int32_t ibuf;
    mp_obj_t callback_for_non_blocking;
    io_mode_t io_mode;
    ring_buf_t ring_buffer;
    uint8_t *ring_buffer_storage;
    non_blocking_descriptor_t non_blocking_descriptor;
    // Zephyr backend
    const struct device *dev;
    struct k_mem_slab *slab;
    struct k_thread thread;
    k_tid_t tid;
    volatile bool stop;  // set by deinit, polled by the worker
} machine_i2s_obj_t;

// One Zephyr buffer slab and one worker-thread stack per I2S peripheral.
K_MEM_SLAB_DEFINE_STATIC(i2s_slab_0, I2S_SLAB_BLOCK_SIZE, I2S_SLAB_BLOCK_COUNT,
    I2S_SLAB_ALIGN);
static struct k_mem_slab *const i2s_slabs[MAX_I2S] = { &i2s_slab_0 };

static K_THREAD_STACK_ARRAY_DEFINE(worker_stacks, MAX_I2S, WORKER_STACK_SIZE);

// Maps an 8-byte RX ring-buffer frame to the user's appbuf format. extmod's
// readinto() path always consumes 8-byte (32-bit stereo) frames; a -1 byte
// is discarded. Indexed by get_frame_mapping_index().
static const int8_t i2s_frame_map[NUM_I2S_USER_FORMATS][I2S_RX_FRAME_SIZE_IN_BYTES] = {
    {-1, -1,  0,  1, -1, -1, -1, -1 },  // Mono, 16-bit
    { 0,  1,  2,  3, -1, -1, -1, -1 },  // Mono, 32-bit
    {-1, -1,  0,  1, -1, -1,  2,  3 },  // Stereo, 16-bit
    { 0,  1,  2,  3,  4,  5,  6,  7 },  // Stereo, 32-bit
};

static int8_t get_frame_mapping_index(int8_t bits, format_t format) {
    if (format == MONO) {
        return (bits == 16) ? 0 : 1;
    } else {
        return (bits == 16) ? 2 : 3;
    }
}

// TX: pop a slab block's worth of bytes from the ring buffer, padding with
// silence on underrun. The ring buffer holds raw appbuf bytes (extmod's
// write() path does no remapping), which for 16-bit stereo is exactly the
// 4-byte/frame layout the controller driver expects.
static void feed_block_from_ringbuf(machine_i2s_obj_t *self, uint8_t *block) {
    size_t avail = ringbuf_available_data(&self->ring_buffer);
    size_t n = MIN(avail, (size_t)I2S_SLAB_BLOCK_SIZE);

    for (size_t i = 0; i < n; i++) {
        ringbuf_pop(&self->ring_buffer, &block[i]);
    }
    if (n < I2S_SLAB_BLOCK_SIZE) {
        memset(block + n, 0, I2S_SLAB_BLOCK_SIZE - n);
    }
}

// RX: push a slab block into the ring buffer as 8-byte (32-bit stereo)
// frames. extmod's readinto() path always consumes 8-byte frames and
// remaps via i2s_frame_map; the controller driver delivers 16-bit stereo
// (4-byte frames), so each 16-bit sample is placed in the high half of a
// 32-bit word, which the Stereo-16 frame map then picks back out.
static void empty_block_to_ringbuf(machine_i2s_obj_t *self, uint8_t *block,
    size_t size) {
    size_t frames = size / 4;

    if (ringbuf_available_space(&self->ring_buffer) < frames * 8) {
        return;  // overflow -- application is not reading fast enough
    }
    for (size_t f = 0; f < frames; f++) {
        const uint8_t *s = &block[f * 4];
        ringbuf_push(&self->ring_buffer, 0);
        ringbuf_push(&self->ring_buffer, 0);
        ringbuf_push(&self->ring_buffer, s[0]);
        ringbuf_push(&self->ring_buffer, s[1]);
        ringbuf_push(&self->ring_buffer, 0);
        ringbuf_push(&self->ring_buffer, 0);
        ringbuf_push(&self->ring_buffer, s[2]);
        ringbuf_push(&self->ring_buffer, s[3]);
    }
}

// Worker thread: bridges the extmod ring buffer to Zephyr's I2S API.
static void i2s_worker(void *p1, void *p2, void *p3) {
    machine_i2s_obj_t *self = p1;
    (void)p2;
    (void)p3;

    if (self->mode == I2S_DIR_TX) {
        // Prime with silence so the controller has blocks to start on.
        for (int i = 0; i < I2S_PRIME_BLOCKS; i++) {
            void *block;
            if (k_mem_slab_alloc(self->slab, &block, K_FOREVER) != 0) {
                break;
            }
            memset(block, 0, I2S_SLAB_BLOCK_SIZE);
            if (i2s_write(self->dev, block, I2S_SLAB_BLOCK_SIZE) != 0) {
                k_mem_slab_free(self->slab, block);
                break;
            }
        }
        if (i2s_trigger(self->dev, I2S_DIR_TX, I2S_TRIGGER_START) == 0) {
            while (!self->stop) {
                void *block;
                if (k_mem_slab_alloc(self->slab, &block,
                    K_MSEC(I2S_RW_TIMEOUT_MS)) != 0) {
                    continue;  // queue full -- re-check stop, retry
                }
                // Non-blocking mode: top up the ring buffer from the appbuf
                // before draining it into the hardware block.
                if (self->io_mode == NON_BLOCKING &&
                    self->non_blocking_descriptor.copy_in_progress) {
                    copy_appbuf_to_ringbuf_non_blocking(self);
                }
                feed_block_from_ringbuf(self, block);
                if (i2s_write(self->dev, block, I2S_SLAB_BLOCK_SIZE) != 0) {
                    k_mem_slab_free(self->slab, block);
                    break;  // controller went to ERROR
                }
            }
        }
        i2s_trigger(self->dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    } else {  // I2S_DIR_RX
        if (i2s_trigger(self->dev, I2S_DIR_RX, I2S_TRIGGER_START) == 0) {
            while (!self->stop) {
                void *block;
                size_t size;
                int ret = i2s_read(self->dev, &block, &size);
                if (ret == -EAGAIN) {
                    continue;  // timeout -- re-check stop, retry
                }
                if (ret != 0) {
                    break;  // controller went to ERROR
                }
                empty_block_to_ringbuf(self, block, size);
                k_mem_slab_free(self->slab, block);
                // Non-blocking mode: drain the ring buffer into the appbuf.
                if (self->io_mode == NON_BLOCKING &&
                    self->non_blocking_descriptor.copy_in_progress) {
                    fill_appbuf_from_ringbuf_non_blocking(self);
                }
            }
        }
        i2s_trigger(self->dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
    }
}

static void mp_machine_i2s_init_helper(machine_i2s_obj_t *self,
    mp_arg_val_t *args) {
    // Pins are accepted (and kept for repr) but not used for muxing -- the
    // i2s controller's pins come from its devicetree node's pinctrl, the
    // same way machine_spi.c leaves SPI muxing to devicetree.
    self->sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
    self->ws = mp_hal_get_pin_obj(args[ARG_ws].u_obj);
    self->sd = mp_hal_get_pin_obj(args[ARG_sd].u_obj);

    uint8_t mode = args[ARG_mode].u_int;
    if (mode != I2S_DIR_RX && mode != I2S_DIR_TX) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid mode"));
    }

    // First cut tracks the i2s_bcm2835 driver: 16-bit stereo only.
    int8_t bits = args[ARG_bits].u_int;
    if (bits != 16) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid bits (only 16 supported)"));
    }
    format_t format = args[ARG_format].u_int;
    if (format != STEREO) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid format (only STEREO supported)"));
    }

    int32_t rate = args[ARG_rate].u_int;
    int32_t ibuf = args[ARG_ibuf].u_int;
    if (ibuf <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid ibuf"));
    }

    self->mode = mode;
    self->bits = bits;
    self->format = format;
    self->rate = rate;
    self->ibuf = ibuf;
    self->callback_for_non_blocking = MP_OBJ_NULL;
    self->io_mode = BLOCKING;
    self->non_blocking_descriptor.copy_in_progress = false;
    self->ring_buffer_storage = NULL;
    self->tid = NULL;
    self->stop = false;

    struct i2s_config cfg = {
        .word_size = bits,
        .channels = 2,
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER,
        .frame_clk_freq = rate,
        .mem_slab = self->slab,
        .block_size = I2S_SLAB_BLOCK_SIZE,
        .timeout = I2S_RW_TIMEOUT_MS,
    };
    int ret = i2s_configure(self->dev, mode, &cfg);
    if (ret < 0) {
        mp_raise_OSError(-ret);
    }

    self->ring_buffer_storage = m_new(uint8_t, ibuf);
    ringbuf_init(&self->ring_buffer, self->ring_buffer_storage, ibuf);

    // The worker runs above the creating (REPL) thread so a blocking
    // write()/readinto() spin in extmod is preempted and makes progress.
    int worker_prio = k_thread_priority_get(k_current_get()) - 1;
    self->tid = k_thread_create(&self->thread, worker_stacks[self->i2s_id],
        K_THREAD_STACK_SIZEOF(worker_stacks[self->i2s_id]),
        i2s_worker, self, NULL, NULL, worker_prio, 0, K_NO_WAIT);
}

static machine_i2s_obj_t *mp_machine_i2s_make_new_instance(mp_int_t i2s_id) {
    if (i2s_id < 0 || i2s_id >= MAX_I2S) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid id"));
    }

    const struct device *dev = i2s_devices[i2s_id];
    if (dev == NULL || !device_is_ready(dev)) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S device not ready"));
    }

    machine_i2s_obj_t *self;
    if (MP_STATE_PORT(machine_i2s_obj[i2s_id]) == NULL) {
        self = mp_obj_malloc(machine_i2s_obj_t, &machine_i2s_type);
        MP_STATE_PORT(machine_i2s_obj[i2s_id]) = self;
        self->i2s_id = i2s_id;
        self->ring_buffer_storage = NULL;
        self->tid = NULL;
    } else {
        self = MP_STATE_PORT(machine_i2s_obj[i2s_id]);
        mp_machine_i2s_deinit(self);
    }

    self->dev = dev;
    self->slab = i2s_slabs[i2s_id];
    return self;
}

static void mp_machine_i2s_deinit(machine_i2s_obj_t *self) {
    if (self->ring_buffer_storage == NULL) {
        return;  // never initialised, or already deinitialised
    }

    // Stop the worker cleanly: it polls self->stop every loop (its i2s
    // calls use a finite timeout), then drains the controller on exit.
    self->stop = true;
    if (self->tid != NULL) {
        k_thread_join(&self->thread, K_FOREVER);
        self->tid = NULL;
    }

    m_del(uint8_t, self->ring_buffer_storage, self->ibuf);
    self->ring_buffer_storage = NULL;
}

static void mp_machine_i2s_irq_update(machine_i2s_obj_t *self) {
    // The worker thread reads self->io_mode directly each iteration; no
    // separate update step is needed.
    (void)self;
}

// Called from the port soft-reset path (main.c) to tear down any I2S
// streams left running. Also NULLs the root pointers so the next boot
// starts clean -- the zephyr port has no per-module init0 hook.
void machine_i2s_deinit_all(void) {
    for (uint8_t i = 0; i < MAX_I2S; i++) {
        machine_i2s_obj_t *self = MP_STATE_PORT(machine_i2s_obj[i]);
        if (self != NULL) {
            mp_machine_i2s_deinit(self);
            MP_STATE_PORT(machine_i2s_obj[i]) = NULL;
        }
    }
}

MP_REGISTER_ROOT_POINTER(void *machine_i2s_obj[1]);
