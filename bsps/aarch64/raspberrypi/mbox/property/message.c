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

#include "bsp/mbox/property/message.h"

#include <bsp/utility.h>
#include <libcpu/mmu-vmsav8-64.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp/mbox.h"
#include "bsp/mbox/property/tags.h"
#include "rtems/rtems/cache.h"

#define NULL_TAG_ID                 0
#define STATUS_CODE_PROCESS_REQUEST 0

void mbox_property_message_buffer_init(const uint8_t *buffer,
                                       const size_t size) {
    /* TODO: Make this buffer-specific */
    rtems_cache_disable_data();
}

int mbox_property_message_init(mbox_property_message *message,
                               const size_t size,
                               const mbox_property_tag_metadata *tag_metadata,
                               const unsigned int tag_count) {
    uintptr_t buffer_end = (uintptr_t)message + size;

    mbox_property_tag *tag = message->buffer;
    for (unsigned int i = 0; i < tag_count; i++) {
        if (mbox_property_tag_init(tag, buffer_end - (uintptr_t)tag,
                                   &tag_metadata[i]) != 0)
            return 1;

        tag = mbox_property_tag_next(tag);
    }

    uint32_t *null_tag = (uint32_t *)tag;
    *null_tag          = NULL_TAG_ID;

    message->header.size   = (uintptr_t)(null_tag + 1) - (uintptr_t)message;
    message->header.status = STATUS_CODE_PROCESS_REQUEST;

    return 0;
}

mbox_property_tag *mbox_property_message_get_tag(
    const mbox_property_message *message, const unsigned int index) {
    mbox_property_tag *tag = (void *)message->buffer;
    if (tag->header.id == NULL_TAG_ID)
        return NULL;

    for (unsigned int i = 0; i < index; i++) {
        if (tag->header.id == NULL_TAG_ID)
            return NULL;

        tag = mbox_property_tag_next(tag);
    }

    return tag;
}
