/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup DevIRQGIC
 *
 * @brief This header file provides interfaces of the ARM Generic Interrupt
 *   Controller (GIC) support specific to the Arm architecture.
 */

/*
 * Copyright (C) 2020 On-Line Applications Research Corporation (OAR)
 * Written by Kinsey Moore <kinsey.moore@oarcorp.com>
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

#ifndef _RTEMS_DEV_IRQ_ARM_GIC_ARM_H
#define _RTEMS_DEV_IRQ_ARM_GIC_ARM_H

#include <bsp/irq-generic.h>
#include <rtems/score/armv4.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup DevIRQGIC
 *
 * @{
 */

static inline uint32_t arm_interrupt_enable_interrupts(void)
{
  return _ARMV4_Status_irq_enable();
}

static inline void arm_interrupt_restore_interrupts(uint32_t status)
{
  _ARMV4_Status_restore(status);
}

static inline void arm_interrupt_facility_set_exception_handler(void)
{
  /*
   * There is no need to install _ARMV4_Exception_interrupt() since this
   * handler is already set by start.S.
   */
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _RTEMS_DEV_IRQ_ARM_GIC_ARM_H */
