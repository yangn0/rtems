/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Mailbox Property Tags
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

#include "bsp/mbox/property/tags.h"

#include <bsp/utility.h>
#include <stddef.h>
#include <stdint.h>

#define TAG_STATUS_RESPONSE_MASK BSP_BIT32(31)

int mbox_property_tag_init(mbox_property_tag *tag, const size_t size,
                           const mbox_property_tag_metadata *metadata) {
    if (sizeof(*tag) > size)
        return 1;

    tag->header.id          = metadata->id;
    tag->header.buffer_size = metadata->size;
    tag->header.status_code &= ~TAG_STATUS_RESPONSE_MASK;

    return 0;
}

mbox_property_tag *mbox_property_tag_next(const mbox_property_tag *current) {
    uintptr_t next_tag_addr = (uintptr_t)(current) + sizeof(current->header) +
                              current->header.buffer_size;
    const unsigned int alignment = sizeof(uint32_t);

    /* Tags are 32-bit aligned */
    if (next_tag_addr % alignment > 0)
        next_tag_addr += alignment - next_tag_addr % alignment;

    return (mbox_property_tag *)next_tag_addr;
}
