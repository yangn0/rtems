/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Console Configuration
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_H

#include <bspopts.h>

#if RTEMS_BSP == raspberrypi4b
#include "console/raspberrypi4b.def"

#define CONSOLE_DEVICES RASPBERRYPI4B_CONSOLE_DEVICES
#endif /* raspberrypi4b */

#define CONSOLE_DEVICE_PORT2ENUM(port_no) UART##port_no
#define CONSOLE_DEVICE_ENUM(port_no, ...) CONSOLE_DEVICE_PORT2ENUM(port_no),
typedef enum {
    /* clang-format off */
    CONSOLE_DEVICES(CONSOLE_DEVICE_ENUM)
    /* clang-format on */

    CONSOLE_DEVICE_COUNT,
} bsp_console_device_port;
#undef CONSOLE_DEVICE_ENUM

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_H */
