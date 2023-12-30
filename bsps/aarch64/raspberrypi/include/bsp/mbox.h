/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Mailbox Driver
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_H

#include <bspopts.h>
#include <stdint.h>

#if RTEMS_BSP == raspberrypi4b
#include "bsp/bcm2711.h"

#define BSP_MBOX_BASE BCM2711_MBOX_BASE
#define BSP_MBOX_SIZE BCM2711_MBOX_SIZE

#define BSP_MBOX0_BASE BCM2711_MBOX0_BASE
#define BSP_MBOX1_BASE BCM2711_MBOX1_BASE
#endif /* raspberrypi4b */

typedef uint32_t mbox_mail;

typedef enum {
    /* MBOX_POWER_MANAGEMENT_CHANNEL = 0, */
    /* MBOX_FRAMEBUFFER_CHANNEL, */
    /* MBOX_VIRTUAL_UART_CHANNEL, */
    /* MBOX_VCHIQ_CHANNEL, */
    /* MBOX_LEDS_CHANNEL, */
    /* MBOX_BUTTONS_CHANNEL, */
    /* MBOX_TOUCHSCREEN_CHANNEL, */
    MBOX_PROPERTY_TAGS_ARM_TO_VC_CHANNEL = 8,
    /* MBOX_PROPERTY_TAGS_VC_TO_ARM_CHANNEL, */
} mbox_channel;

mbox_mail mbox_mail_compose(const mbox_channel, const uint32_t data);
mbox_mail mbox_read(void);
void mbox_write(const mbox_mail mail);

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_H */
