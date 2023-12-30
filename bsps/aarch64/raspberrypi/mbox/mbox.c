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

#include "bsp/mbox.h"

#include <bsp/utility.h>
#include <rtems/rtems/cache.h>
#include <stdint.h>

#define REG(addr) *(volatile uint32_t*)(addr)

#define MBOX0_BASE BSP_MBOX0_BASE
#define MBOX1_BASE BSP_MBOX1_BASE

#define MBOX_RW_REG(base)     REG(base + 0x00)
#define MBOX_STATUS_REG(base) REG(base + 0x18)
#define MBOX_STATUS_EMPTY     BSP_BIT32(30)
#define MBOX_STATUS_FULL      BSP_BIT32(31)

#define MBOX_MAIL_CHANNEL_MASK BSP_MSK32(0, 3)
#define MBOX_MAIL_DATA_MASK    ~MBOX_MAIL_CHANNEL_MASK

mbox_mail mbox_mail_compose(const mbox_channel channel, const uint32_t data) {
    mbox_mail mail = data & MBOX_MAIL_DATA_MASK;
    mail |= channel & MBOX_MAIL_CHANNEL_MASK;

    return mail;
}

mbox_mail mbox_read(void) {
    /* Wait until there is data to be read */
    while ((MBOX_STATUS_REG(MBOX0_BASE) & MBOX_STATUS_EMPTY) != 0)
        ;

    const mbox_mail mail = (mbox_mail)MBOX_RW_REG(MBOX0_BASE);
    return mail;
}

void mbox_write(const mbox_mail mail) {
    /* Wait until space is available */
    while ((MBOX_STATUS_REG(MBOX1_BASE) & MBOX_STATUS_FULL) != 0)
        ;

    MBOX_RW_REG(MBOX1_BASE) = (uint32_t)mail;
}
