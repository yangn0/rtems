/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Mailbox Clocks Property Tags
 */

/*
 * Copyright (C) 2023 Utkarsh Verma
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_CLOCKS_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_CLOCKS_H

#include <stdint.h>

#define CLOCKS_GET_CLOCK_STATE_TAG \
    { 0x00030001, sizeof(clocks_get_clock_state_tag_buffer) }
#define CLOCKS_SET_CLOCK_STATE_TAG \
    { 0x00038001, sizeof(clocks_set_clock_state_tag_buffer) }
#define CLOCKS_GET_CLOCK_RATE_TAG \
    { 0x00030002, sizeof(clocks_get_clock_rate_tag_buffer) }
#define CLOCKS_SET_CLOCK_RATE_TAG \
    { 0x00038002, sizeof(clocks_set_clock_rate_tag_buffer) }
#define CLOCKS_GET_MAX_CLOCK_RATE_TAG \
    { 0x00030004, sizeof(clocks_get_max_clock_rate_tag_buffer) }
#define CLOCKS_GET_MIN_CLOCK_RATE_TAG \
    { 0x00030007, sizeof(clocks_get_min_clock_rate_tag_buffer) }
#define CLOCKS_GET_TURBO_TAG \
    { 0x00030009, sizeof(clocks_get_turbo_tag_buffer) }
#define CLOCKS_SET_TURBO_TAG \
    { 0x00038009, sizeof(clocks_set_turbo_tag_buffer) }
#define CLOCKS_GET_ONBOARD_LED_STATUS_TAG \
    { 0x00030041, sizeof(clocks_get_onboard_led_status_tag_buffer) }
#define CLOCKS_SET_ONBOARD_LED_STATUS_TAG \
    { 0x00038041, sizeof(clocks_set_onboard_led_status_tag_buffer) }
#define CLOCKS_GET_CLOCK_RATE_MEASURED_TAG \
    { 0x00030047, sizeof(clocks_get_clock_rate_measured_tag_buffer) }

typedef enum {
    EMMC_CLOCK_ID = 1,
    UART_CLOCK_ID,
    ARM_CLOCK_ID,
    CORE_CLOCK_ID,
    V3D_CLOCK_ID,
    H264_CLOCK_ID,
    ISP_CLOCK_ID,
    SDRAM_CLOCK_ID,
    PIXEL_CLOCK_ID,
    PWM_CLOCK_ID,
    HEVC_CLOCK_ID,
    EMMC2_CLOCK_ID,
    M2MC_CLOCK_ID,
    PIXEL_BVB_CLOCK_ID,

    CLOCK_COUNT,
} clock_id;

typedef union {
    struct {
        uint32_t clock_id;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t state;
    } response;
} clocks_get_clock_state_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
        uint32_t state;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t state;
    } response;
} clocks_set_clock_state_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t rate_hz;
    } response;
} clocks_get_clock_rate_tag_buffer;

typedef union {
    const struct {
        uint32_t pin_number;
        uint32_t status;
    } response;
} clocks_get_onboard_led_status_tag_buffer;

typedef union {
    const struct {
        uint32_t pin_number;
        uint32_t status;
    } response;
} clocks_test_onboard_led_status_tag_buffer;

typedef union {
    struct {
        uint32_t pin_number;
        uint32_t status;
    } request;

    const struct {
        uint32_t pin_no;
        uint32_t status;
    } response;
} clocks_set_onboard_led_status_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t rate_hz;
    } response;
} clocks_get_clock_rate_measured_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
        uint32_t rate_hz;
        uint32_t skip_turbo;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t rate_hz;
    } response;
} clocks_set_clock_rate_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t rate_hz;
    } response;
} clocks_get_max_clock_rate_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t rate_hz;
    } response;
} clocks_get_min_clock_rate_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t level;
    } response;
} clocks_get_turbo_tag_buffer;

typedef union {
    struct {
        uint32_t clock_id;
        uint32_t level;
    } request;

    const struct {
        uint32_t clock_id;
        uint32_t level;
    } response;
} clocks_set_turbo_tag_buffer;

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_CLOCKS_H */
