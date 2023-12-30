/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Mailbox Property Message
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_MESSAGE_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_MESSAGE_H

#include <bspopts.h>
#include <stddef.h>
#include <stdint.h>

#include "tags.h"

/*
 * NOTE:
 * This has to be 16-byte aligned as the least significant nibble is
 * discarded from the pointer and assumed to be zero while making the
 * mailbox call.
 */
#define mbox_property_message_buffer uint8_t __attribute__((aligned(16)))

typedef struct {
    uint32_t size;
    volatile uint32_t status;
} mbox_property_message_header;

typedef struct {
    mbox_property_message_header header;
    mbox_property_tag buffer[];
} mbox_property_message;

void mbox_property_message_buffer_init(const uint8_t* buffer,
                                       const size_t size);
int mbox_property_message_init(mbox_property_message* message,
                               const size_t size,
                               const mbox_property_tag_metadata* tag_metadata,
                               const unsigned int tag_count);
mbox_property_tag* mbox_property_message_get_tag(
    const mbox_property_message* message, const unsigned int index);

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_MESSAGE_H */
