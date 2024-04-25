/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup DevIRQGIC
 *
 * @brief This header file provides interfaces of the ARM Generic Interrupt
 *   Controller (GIC) support specific to the AArch64 architecture.
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

#ifndef _RTEMS_DEV_IRQ_ARM_GIC_AARCH64_H
#define _RTEMS_DEV_IRQ_ARM_GIC_AARCH64_H

#include <rtems/score/cpu.h>
#include <rtems/score/cpu_irq.h>

#include <bsp/irq-generic.h>

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
  uint32_t status = _CPU_ISR_Get_level();
  /* Enable interrupts for nesting */
  _CPU_ISR_Set_level(0);
  return status;
}

static inline void arm_interrupt_restore_interrupts(uint32_t status)
{
  /* Restore interrupts to previous level */
  _CPU_ISR_Set_level(status);
}

static inline void arm_interrupt_facility_set_exception_handler(void)
{
  AArch64_set_exception_handler(
    AARCH64_EXCEPTION_SPx_IRQ,
    _AArch64_Exception_interrupt_no_nest
  );
  AArch64_set_exception_handler(
    AARCH64_EXCEPTION_SP0_IRQ,
    _AArch64_Exception_interrupt_nest
  );
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _RTEMS_DEV_IRQ_ARM_GIC_AARCH64_H */
