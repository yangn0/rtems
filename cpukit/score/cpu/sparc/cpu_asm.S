/* SPDX-License-Identifier: BSD-2-Clause */

/*  cpu_asm.s
 *
 *  This file contains the basic algorithms for all assembly code used
 *  in an specific CPU port of RTEMS.  These algorithms must be implemented
 *  in assembly language.
 *
 *  COPYRIGHT (c) 1989-2011.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  Copyright (C) 2014, 2017 embedded brains GmbH & Co. KG
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
 *
 *  Ported to ERC32 implementation of the SPARC by On-Line Applications
 *  Research Corporation (OAR) under contract to the European Space
 *  Agency (ESA).
 *
 *  ERC32 modifications of respective RTEMS file: COPYRIGHT (c) 1995.
 *  European Space Agency.
 */

#include <rtems/asm.h>
#include <rtems/score/percpu.h>
#include <libcpu/grlib-tn-0018.h>

/*
 *  void _CPU_Context_switch(
 *    Context_Control  *run,
 *    Context_Control  *heir
 *  )
 *
 *  This routine performs a normal non-FP context switch.
 */

        .align 4
        PUBLIC(_CPU_Context_switch)
        PUBLIC(_CPU_Context_switch_no_return)
SYM(_CPU_Context_switch):
SYM(_CPU_Context_switch_no_return):
        st      %g5, [%o0 + G5_OFFSET]       ! save the global registers

        /*
         * No need to save the thread pointer %g7 since it is a thread
         * invariant.  It is initialized once in _CPU_Context_Initialize().
         */

        std     %l0, [%o0 + L0_OFFSET]       ! save the local registers
        SPARC_LEON3FT_B2BST_NOP
        std     %l2, [%o0 + L2_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l4, [%o0 + L4_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l6, [%o0 + L6_OFFSET]
        SPARC_LEON3FT_B2BST_NOP

        std     %i0, [%o0 + I0_OFFSET]       ! save the input registers
        SPARC_LEON3FT_B2BST_NOP
        std     %i2, [%o0 + I2_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i4, [%o0 + I4_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i6, [%o0 + I6_FP_OFFSET]
        SPARC_LEON3FT_B2BST_NOP

        std     %o6, [%o0 + O6_SP_OFFSET]    ! save the output registers

        ! load the ISR stack nesting prevention flag
        ld      [%g6 + PER_CPU_ISR_DISPATCH_DISABLE], %o4
        ! save it a bit later so we do not waste a couple of cycles

        rd      %psr, %o2
        st      %o2, [%o0 + PSR_OFFSET]      ! save status register

        ! Now actually save ISR stack nesting prevention flag
        st       %o4, [%o0 + ISR_DISPATCH_DISABLE_STACK_OFFSET]

        /*
         *  This is entered from _CPU_Context_restore with:
         *    o1 = context to restore
         *    o2 = psr
         */

        PUBLIC(_CPU_Context_restore_heir)
SYM(_CPU_Context_restore_heir):
        /*
         *  Flush all windows with valid contents except the current one.
         *  In examining the set register windows, one may logically divide
         *  the windows into sets (some of which may be empty) based on their
         *  current status:
         *
         *    + current (i.e. in use),
         *    + used (i.e. a restore would not trap)
         *    + invalid (i.e. 1 in corresponding bit in WIM)
         *    + unused
         *
         *  Either the used or unused set of windows may be empty.
         *
         *  NOTE: We assume only one bit is set in the WIM at a time.
         *
         *  Given a CWP of 5 and a WIM of 0x1, the registers are divided
         *  into sets as follows:
         *
         *    + 0   - invalid
         *    + 1-4 - unused
         *    + 5   - current
         *    + 6-7 - used
         *
         *  In this case, we only would save the used windows -- 6 and 7.
         *
         *   Traps are disabled for the same logical period as in a
         *     flush all windows trap handler.
         *
         *    Register Usage while saving the windows:
         *      g1 = current PSR
         *      g2 = current wim
         *      g3 = CWP
         *      g4 = wim scratch
         *      g5 = scratch
         */

        and     %o2, SPARC_PSR_CWP_MASK, %g3  ! g3 = CWP
        andn    %o2, SPARC_PSR_ET_MASK, %g1   ! g1 = psr with traps disabled
        mov     %g1, %psr                     ! **** DISABLE TRAPS ****
        mov     %wim, %g2                     ! g2 = wim
        mov     1, %g4
        sll     %g4, %g3, %g4                 ! g4 = WIM mask for CW invalid

save_frame_loop:
        sll     %g4, 1, %g5                   ! rotate the "wim" left 1
        srl     %g4, SPARC_NUMBER_OF_REGISTER_WINDOWS - 1, %g4
        or      %g4, %g5, %g4                 ! g4 = wim if we do one restore

        /*
         *  If a restore would not underflow, then continue.
         */

        andcc   %g4, %g2, %g0                 ! Any windows to flush?
        bnz     done_flushing                 ! No, then continue
        nop

        restore                               ! back one window

        /*
         *  Now save the window just as if we overflowed to it.
         */

        std     %l0, [%sp + CPU_STACK_FRAME_L0_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l2, [%sp + CPU_STACK_FRAME_L2_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l4, [%sp + CPU_STACK_FRAME_L4_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l6, [%sp + CPU_STACK_FRAME_L6_OFFSET]
        SPARC_LEON3FT_B2BST_NOP

        std     %i0, [%sp + CPU_STACK_FRAME_I0_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i2, [%sp + CPU_STACK_FRAME_I2_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i4, [%sp + CPU_STACK_FRAME_I4_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i6, [%sp + CPU_STACK_FRAME_I6_FP_OFFSET]
        SPARC_LEON3FT_B2BST_NOP

        ba      save_frame_loop
        nop

done_flushing:

        ! Wait three instructions after the write to PSR before using
        ! non-global registers or instructions affecting the CWP
        mov     %g1, %psr                     ! restore cwp
        add     %g3, 1, %g2                   ! calculate desired WIM
        and     %g2, SPARC_NUMBER_OF_REGISTER_WINDOWS - 1, %g2
        mov     1, %g4
        sll     %g4, %g2, %g4                 ! g4 = new WIM
        mov     %g4, %wim

#if defined(RTEMS_SMP)
        /*
         * The executing thread no longer executes on this processor.  Switch
         * the stack to the temporary interrupt stack of this processor.  Mark
         * the context of the executing thread as not executing.
         */
        add     %g6, PER_CPU_INTERRUPT_FRAME_AREA + CPU_INTERRUPT_FRAME_SIZE, %sp
        st      %g0, [%o0 + SPARC_CONTEXT_CONTROL_IS_EXECUTING_OFFSET]

        ! Try to update the is executing indicator of the heir context
        mov     1, %g1

#if defined(__FIX_LEON3FT_B2BST)
        /*
         * This is a workaround for GRLIB-TN-0011 (Technical Note on LEON3/FT
         * AHB Lock Release During Atomic Operation).  Affected components are
         * the GR712RC, UT699, UT699E, UT700, and LEON3FT-RTAX.  Strictly, the
         * workaround is only necessary if the MMU is enabled.  Using the
         * __FIX_LEON3FT_B2BST is not 100% appropriate, but the best thing we
         * can use to enable the workaround.  An alignment padding is filled
         * with nops.
         */
.align 16
#endif
.Ltry_update_is_executing:

        swap    [%o1 + SPARC_CONTEXT_CONTROL_IS_EXECUTING_OFFSET], %g1
        cmp     %g1, 0
        bne     .Lcheck_is_executing

        ! The next load is in a delay slot, which is all right
#endif

#if defined(SPARC_USE_LAZY_FP_SWITCH)
        ld      [%g6 + SPARC_PER_CPU_FP_OWNER_OFFSET], %g2
#endif
        ld      [%o1 + PSR_OFFSET], %g1       ! g1 = heir psr with traps enabled
#if defined(SPARC_USE_LAZY_FP_SWITCH)
        sethi   %hi(SPARC_PSR_EF_MASK), %g5
        cmp     %g2, %g0
        bne,a   .Lclear_psr_ef_done
         andn   %g1, %g5, %g1                 ! g1 = heir psr w/o PSR[EF]
.Lclear_psr_ef_done:
#endif
        andn    %g1, SPARC_PSR_CWP_MASK, %g1  ! g1 = heir psr w/o cwp
        or      %g1, %g3, %g1                 ! g1 = heir psr with cwp
        mov     %g1, %psr                     ! restore status register and
                                              ! **** ENABLE TRAPS ****

        /*
         * WARNING: This code does not run with the restored stack pointer.  In
         * SMP configurations, it uses a processor-specific stack.  In
         * uniprocessor configurations, it uses the stack of the caller.  In
         * this case, the caller shall ensure that it is not the interrupt
         * stack (which is also the system initialization stack).
         */

        ld      [%o1 + G5_OFFSET], %g5        ! restore the global registers
        ld      [%o1 + G7_OFFSET], %g7

        ! Load thread specific ISR dispatch prevention flag
        ld      [%o1 + ISR_DISPATCH_DISABLE_STACK_OFFSET], %o2
        ! Store it to memory later to use the cycles

        ldd     [%o1 + L0_OFFSET], %l0        ! restore the local registers
        ldd     [%o1 + L2_OFFSET], %l2
        ldd     [%o1 + L4_OFFSET], %l4
        ldd     [%o1 + L6_OFFSET], %l6

        ! Now restore thread specific ISR dispatch prevention flag
        st      %o2, [%g6 + PER_CPU_ISR_DISPATCH_DISABLE]

        ldd     [%o1 + I0_OFFSET], %i0        ! restore the input registers
        ldd     [%o1 + I2_OFFSET], %i2
        ldd     [%o1 + I4_OFFSET], %i4
        ldd     [%o1 + I6_FP_OFFSET], %i6

        ldd     [%o1 + O6_SP_OFFSET], %o6     ! restore the non-volatile output
                                              ! registers (stack pointer,
                                              ! link register)

        jmp     %o7 + 8                       ! return
        nop                                   ! delay slot

#if defined(RTEMS_SMP)
.Lcheck_is_executing:

        ! Check the is executing indicator of the heir context
        ld      [%o1 + SPARC_CONTEXT_CONTROL_IS_EXECUTING_OFFSET], %g1
        cmp     %g1, 0
        beq     .Ltry_update_is_executing
         mov    1, %g1

        ! We may have a new heir

        ! Read the executing and heir
        ld      [%g6 + PER_CPU_OFFSET_EXECUTING], %g2
        ld      [%g6 + PER_CPU_OFFSET_HEIR], %g4

        ! Update the executing only if necessary to avoid cache line
        ! monopolization.
        cmp     %g2, %g4
        beq     .Ltry_update_is_executing
         mov    1, %g1

        ! Calculate the heir context pointer
        sub     %o1, %g2, %g2
        add     %g2, %g4, %o1

        ! Update the executing
        st      %g4, [%g6 + PER_CPU_OFFSET_EXECUTING]

        ba      .Ltry_update_is_executing
         mov    1, %g1
#endif

/*
 *  void _CPU_Context_restore(
 *    Context_Control *new_context
 *  )
 *
 *  This routine is generally used only to perform restart self.
 *
 *  NOTE: It is unnecessary to reload some registers.
 */
        .align 4
        PUBLIC(_CPU_Context_restore)
SYM(_CPU_Context_restore):
        save    %sp, -SPARC_MINIMUM_STACK_FRAME_SIZE, %sp
        rd      %psr, %o2
#if defined(RTEMS_SMP)
        ! On SPARC the restore path needs also a valid executing context on SMP
        ! to update the is executing indicator.
        mov     %i0, %o0
#endif
        ba      SYM(_CPU_Context_restore_heir)
        mov     %i0, %o1                      ! in the delay slot

#if !defined(RTEMS_SMP)
        .align 4
        PUBLIC(_SPARC_Start_multitasking)
SYM(_SPARC_Start_multitasking):
        /*
         * Restore the stack pointer right now, so that the window flushing and
         * interrupts during _CPU_Context_restore_heir() use the stack of the
         * heir thread.  This is crucial for the interrupt handling to prevent
         * a concurrent use of the interrupt stack (which is also the system
         * initialization stack).
         */
        ld      [%o0 + O6_SP_OFFSET], %o6

        ba      SYM(_CPU_Context_restore)
         nop
#endif

/*
 *  void _SPARC_Interrupt_trap()
 *
 *  This routine provides the RTEMS interrupt management.
 *
 *  We enter this handler from the 4 instructions in the trap table with
 *  the following registers assumed to be set as shown:
 *
 *    l0 = PSR
 *    l1 = PC
 *    l2 = nPC
 *    l3 = interrupt vector number (this is not the trap type)
 *
 *  NOTE: This trap handler is intended to service external interrupts.
 */

        .align 4
        PUBLIC(_SPARC_Interrupt_trap)
SYM(_SPARC_Interrupt_trap):
        /*
         *  Save the globals this block uses.
         *
         *  These registers are not restored from the locals.  Their contents
         *  are saved directly from the locals into the ISF below.
         */

        mov     %g4, %l4                 ! save the globals this block uses
        mov     %g5, %l5

        /*
         *  When at a "window overflow" trap, (wim == (1 << cwp)).
         *  If we get here like that, then process a window overflow.
         */

        rd      %wim, %g4
        srl     %g4, %l0, %g5            ! g5 = win >> cwp ; shift count and CWP
                                         !   are LS 5 bits ; how convenient :)
        cmp     %g5, 1                   ! Is this an invalid window?
        bne     dont_do_the_window       ! No, then skip all this stuff
        ! we are using the delay slot

        /*
         *  The following is same as a 1 position right rotate of WIM
         */

        srl     %g4, 1, %g5              ! g5 = WIM >> 1
        sll     %g4, SPARC_NUMBER_OF_REGISTER_WINDOWS-1 , %g4
                                         ! g4 = WIM << (Number Windows - 1)
        or      %g4, %g5, %g4            ! g4 = (WIM >> 1) |
                                         !      (WIM << (Number Windows - 1))

        /*
         *  At this point:
         *
         *    g4 = the new WIM
         *    g5 is free
         */

        /*
         *  Since we are tinkering with the register windows, we need to
         *  make sure that all the required information is in global registers.
         */

        save                          ! Save into the window
        wr      %g4, 0, %wim          ! WIM = new WIM
        nop                           ! delay slots
        nop
        nop

        /*
         *  Now save the window just as if we overflowed to it.
         */

        std     %l0, [%sp + CPU_STACK_FRAME_L0_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l2, [%sp + CPU_STACK_FRAME_L2_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l4, [%sp + CPU_STACK_FRAME_L4_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %l6, [%sp + CPU_STACK_FRAME_L6_OFFSET]
        SPARC_LEON3FT_B2BST_NOP

        std     %i0, [%sp + CPU_STACK_FRAME_I0_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i2, [%sp + CPU_STACK_FRAME_I2_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i4, [%sp + CPU_STACK_FRAME_I4_OFFSET]
        SPARC_LEON3FT_B2BST_NOP
        std     %i6, [%sp + CPU_STACK_FRAME_I6_FP_OFFSET]

        restore
        nop

dont_do_the_window:
        /*
         *  Global registers %g4 and %g5 are saved directly from %l4 and
         *  %l5 directly into the ISF below.
         */

        /*
         *  Save the state of the interrupted task -- especially the global
         *  registers -- in the Interrupt Stack Frame.  Note that the ISF
         *  includes a regular minimum stack frame which will be used if
         *  needed by register window overflow and underflow handlers.
         *
         *  REGISTERS SAME AS AT _SPARC_Interrupt_trap()
         */

        sub     %fp, CPU_INTERRUPT_FRAME_SIZE, %sp
                                               ! make space for ISF

        std     %l0, [%sp + ISF_PSR_OFFSET]    ! save psr, PC
        SPARC_LEON3FT_B2BST_NOP
        st      %l2, [%sp + ISF_NPC_OFFSET]    ! save nPC
        st      %g1, [%sp + ISF_G1_OFFSET]     ! save g1
        std     %g2, [%sp + ISF_G2_OFFSET]     ! save g2, g3
        SPARC_LEON3FT_B2BST_NOP
        std     %l4, [%sp + ISF_G4_OFFSET]     ! save g4, g5 -- see above
        SPARC_LEON3FT_B2BST_NOP
        st      %g7, [%sp + ISF_G7_OFFSET]     ! save g7

        std     %i0, [%sp + ISF_I0_OFFSET]     ! save i0, i1
        SPARC_LEON3FT_B2BST_NOP
        std     %i2, [%sp + ISF_I2_OFFSET]     ! save i2, i3
        SPARC_LEON3FT_B2BST_NOP
        std     %i4, [%sp + ISF_I4_OFFSET]     ! save i4, i5
        SPARC_LEON3FT_B2BST_NOP
        std     %i6, [%sp + ISF_I6_FP_OFFSET]  ! save i6/fp, i7

        rd      %y, %g1
        st      %g1, [%sp + ISF_Y_OFFSET]      ! save y

        /*
         *  Increment ISR nest level and Thread dispatch disable level.
         *
         *  Register usage for this section:
         *
         *    l6 = _Thread_Dispatch_disable_level value
         *    l7 = _ISR_Nest_level value
         *
         *  NOTE: It is assumed that l6 - l7 will be preserved until the ISR
         *        nest and thread dispatch disable levels are unnested.
         */

        ld       [%g6 + PER_CPU_ISR_NEST_LEVEL], %l7
        ld       [%g6 + PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL], %l6

        add      %l7, 1, %l7
        st       %l7, [%g6 + PER_CPU_ISR_NEST_LEVEL]
        SPARC_LEON3FT_B2BST_NOP

        add      %l6, 1, %l6
        st       %l6, [%g6 + PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL]

#if SPARC_HAS_FPU == 1
        /*
         * We cannot use an intermediate value for operations with the PSR[EF]
         * bit since they use a 13-bit sign extension and PSR[EF] is bit 12.
         */
        sethi    %hi(SPARC_PSR_EF_MASK), %l5
#endif

        /*
         *  If ISR nest level was zero (now 1), then switch stack.
         */

        mov      %sp, %fp
        subcc    %l7, 1, %l7             ! outermost interrupt handler?
        bnz      dont_switch_stacks      ! No, then do not switch stacks

#if defined(RTEMS_PROFILING)
        call     SYM(_SPARC_Counter_read_ISR_disabled)
         nop
        mov      %o0, %o5
#else
         nop
#endif

        ld       [%g6 + PER_CPU_INTERRUPT_STACK_HIGH], %sp

#if SPARC_HAS_FPU == 1
        /*
         * Test if the interrupted thread uses the floating point unit
         * (PSR[EF] == 1).  In case it uses the floating point unit, then store
         * the floating point status register.  This has the side-effect that
         * all pending floating point operations complete before the store
         * completes.  The PSR[EF] bit is restored after the call to the
         * interrupt handler.  Thus post-switch actions (e.g. signal handlers)
         * and context switch extensions may still corrupt the floating point
         * context.
         */
        andcc    %l0, %l5, %g0
        beq      dont_switch_stacks
         nop
        st      %fsr, [%g6 + SPARC_PER_CPU_FSR_OFFSET]
#endif

dont_switch_stacks:
        /*
         *  Make sure we have a place on the stack for the window overflow
         *  trap handler to write into.  At this point it is safe to
         *  enable traps again.
         */

        sub      %sp, SPARC_MINIMUM_STACK_FRAME_SIZE, %sp

        /*
         *  Set the PIL in the %psr to mask off interrupts with lower priority.
         *  The original %psr in %l0 is not modified since it will be restored
         *  when the interrupt handler returns.
         */

        mov      %l0, %g5
        sll      %l3, 8, %g4
        and      %g4, SPARC_PSR_PIL_MASK, %g4
        andn     %l0, SPARC_PSR_PIL_MASK, %g5
        or       %g4, %g5, %g5

#if SPARC_HAS_FPU == 1
        /*
         * Clear the PSR[EF] bit of the interrupted context to ensure that
         * interrupt service routines cannot corrupt the floating point context.
         */
        andn     %g5, %l5, %g5
#endif

        wr       %g5, SPARC_PSR_ET_MASK, %psr ! **** ENABLE TRAPS ****

        /*
         *  Call _SPARC_Interrupt_dispatch( %l3 )
         */
        mov      %l3, %o0               ! o0 = 1st arg = vector number
        call     SYM(_SPARC_Interrupt_dispatch)
#if defined(RTEMS_PROFILING)
         mov     %o5, %l3               ! save interrupt entry instant
#else
         nop                            ! delay slot
#endif

#if defined(SPARC_USE_SYNCHRONOUS_FP_SWITCH)
        mov      %l0, %g1               ! PSR[EF] value of interrupted context
        ta       SPARC_SWTRAP_IRQDIS_FP ! **** DISABLE INTERRUPTS ****
#else
        ta       SPARC_SWTRAP_IRQDIS    ! **** DISABLE INTERRUPTS ****
#endif

#if defined(RTEMS_PROFILING)
        cmp      %l7, 0
        bne      profiling_not_outer_most_exit
         nop
        call     SYM(_SPARC_Counter_read_ISR_disabled)
         mov     %g1, %l4               ! Save previous interrupt status
        mov      %o0, %o2               ! o2 = 3rd arg = interrupt exit instant
        mov      %l3, %o1               ! o1 = 2nd arg = interrupt entry instant
        call     SYM(_Profiling_Outer_most_interrupt_entry_and_exit)
         mov     %g6, %o0               ! o0 = 1st arg = per-CPU control
profiling_not_outer_most_exit:
#endif

        /*
         *  Decrement ISR nest level and Thread dispatch disable level.
         *
         *  Register usage for this section:
         *
         *    o2 = g6->dispatch_necessary value
         *    o3 = g6->isr_dispatch_disable value
         *    l6 = g6->thread_dispatch_disable_level value
         *    l7 = g6->isr_nest_level value
         */

        ldub     [%g6 + PER_CPU_DISPATCH_NEEDED], %o2
        ld       [%g6 + PER_CPU_ISR_DISPATCH_DISABLE], %o3
        st       %l7, [%g6 + PER_CPU_ISR_NEST_LEVEL]
        SPARC_LEON3FT_B2BST_NOP
        sub      %l6, 1, %l6
        st       %l6, [%g6 + PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL]

        /*
         * Thread dispatching is necessary and allowed if and only if
         *   g6->dispatch_necessary == 1 and
         *   g6->isr_dispatch_disable == 0 and
         *   g6->thread_dispatch_disable_level == 0.
         *
         * Otherwise, continue with the simple return.
         */
        xor      %o2, 1, %o2
        or       %o2, %l6, %o2
        orcc     %o2, %o3, %o2
        bnz      simple_return

        /*
         * Switch back on the interrupted tasks stack and add enough room to
         * invoke the dispatcher.  Doing this in the delay slot causes no harm,
         * since the stack pointer (%sp) is not used in the simple return path.
         */
         sub     %fp, SPARC_MINIMUM_STACK_FRAME_SIZE, %sp

isr_dispatch:

        /* Set ISR dispatch disable and thread dispatch disable level to one */
        mov      1, %l6
        st       %l6, [%g6 + PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL]
        st       %l6, [%g6 + PER_CPU_ISR_DISPATCH_DISABLE]

        /* Call _Thread_Do_dispatch(), this function will enable interrupts */

        mov      0, %o1                 ! ISR level for _Thread_Do_dispatch()

#if defined(SPARC_USE_LAZY_FP_SWITCH)
        /* Test if we interrupted a floating point thread (PSR[EF] == 1) */
        andcc   %l0, %l5, %g0
        be      .Lnon_fp_thread_dispatch
         ld     [%g6 + PER_CPU_OFFSET_EXECUTING], %l6

        /* Set new floating point unit owner to executing thread */
        st      %l6, [%g6 + SPARC_PER_CPU_FP_OWNER_OFFSET]

        call    SYM(_Thread_Do_dispatch)
         mov    %g6, %o0

        /*
         * If we are still the floating point unit owner, then reset the
         * floating point unit owner to NULL, otherwise clear PSR[EF] in the
         * interrupt frame and let the FP disabled system call do the floating
         * point context save/restore.
         */
        ld      [%g6 + SPARC_PER_CPU_FP_OWNER_OFFSET], %l7
        cmp     %l6, %l7
        bne,a   .Ldisable_fp
         andn   %l0, %l5, %l0
        st      %g0, [%g6 + SPARC_PER_CPU_FP_OWNER_OFFSET]
        ba      .Lthread_dispatch_done
         nop
.Ldisable_fp:
        st       %l0, [%fp + ISF_PSR_OFFSET]
        ba      .Lthread_dispatch_done
         nop
.Lnon_fp_thread_dispatch:
#elif defined(SPARC_USE_SYNCHRONOUS_FP_SWITCH)
        /* Test if we interrupted a floating point thread (PSR[EF] == 1) */
        andcc   %l0, %l5, %g0
        be      .Lnon_fp_thread_dispatch
         nop

        /*
         * Yes, this is a floating point thread, then save the floating point
         * context to a new stack frame.  Then do the thread dispatch.
         * Post-switch actions (e.g. signal handlers) and context switch
         * extensions may safely use the floating point unit.
         */
        sub     %sp, SPARC_FP_FRAME_SIZE, %sp
        std     %f0, [%sp + SPARC_FP_FRAME_OFFSET_FO_F1]
        SPARC_LEON3FT_B2BST_NOP
        std     %f2, [%sp + SPARC_FP_FRAME_OFFSET_F2_F3]
        SPARC_LEON3FT_B2BST_NOP
        std     %f4, [%sp + SPARC_FP_FRAME_OFFSET_F4_F5]
        SPARC_LEON3FT_B2BST_NOP
        std     %f6, [%sp + SPARC_FP_FRAME_OFFSET_F6_F7]
        SPARC_LEON3FT_B2BST_NOP
        std     %f8, [%sp + SPARC_FP_FRAME_OFFSET_F8_F9]
        SPARC_LEON3FT_B2BST_NOP
        std     %f10, [%sp + SPARC_FP_FRAME_OFFSET_F1O_F11]
        SPARC_LEON3FT_B2BST_NOP
        std     %f12, [%sp + SPARC_FP_FRAME_OFFSET_F12_F13]
        SPARC_LEON3FT_B2BST_NOP
        std     %f14, [%sp + SPARC_FP_FRAME_OFFSET_F14_F15]
        SPARC_LEON3FT_B2BST_NOP
        std     %f16, [%sp + SPARC_FP_FRAME_OFFSET_F16_F17]
        SPARC_LEON3FT_B2BST_NOP
        std     %f18, [%sp + SPARC_FP_FRAME_OFFSET_F18_F19]
        SPARC_LEON3FT_B2BST_NOP
        std     %f20, [%sp + SPARC_FP_FRAME_OFFSET_F2O_F21]
        SPARC_LEON3FT_B2BST_NOP
        std     %f22, [%sp + SPARC_FP_FRAME_OFFSET_F22_F23]
        SPARC_LEON3FT_B2BST_NOP
        std     %f24, [%sp + SPARC_FP_FRAME_OFFSET_F24_F25]
        SPARC_LEON3FT_B2BST_NOP
        std     %f26, [%sp + SPARC_FP_FRAME_OFFSET_F26_F27]
        SPARC_LEON3FT_B2BST_NOP
        std     %f28, [%sp + SPARC_FP_FRAME_OFFSET_F28_F29]
        SPARC_LEON3FT_B2BST_NOP
        std     %f30, [%sp + SPARC_FP_FRAME_OFFSET_F3O_F31]
        SPARC_LEON3FT_B2BST_NOP
        st      %fsr, [%sp + SPARC_FP_FRAME_OFFSET_FSR]
        call    SYM(_Thread_Do_dispatch)
         mov    %g6, %o0

        /*
         * Restore the floating point context from stack frame and release the
         * stack frame.
         */
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_FO_F1], %f0
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F2_F3], %f2
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F4_F5], %f4
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F6_F7], %f6
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F8_F9], %f8
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F1O_F11], %f10
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F12_F13], %f12
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F14_F15], %f14
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F16_F17], %f16
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F18_F19], %f18
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F2O_F21], %f20
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F22_F23], %f22
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F24_F25], %f24
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F26_F27], %f26
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F28_F29], %f28
        ldd     [%sp + SPARC_FP_FRAME_OFFSET_F3O_F31], %f30
        ld      [%sp + SPARC_FP_FRAME_OFFSET_FSR], %fsr
        ba      .Lthread_dispatch_done
         add    %sp, SPARC_FP_FRAME_SIZE, %sp

.Lnon_fp_thread_dispatch:
#endif

        call    SYM(_Thread_Do_dispatch)
         mov    %g6, %o0

#if SPARC_HAS_FPU == 1
.Lthread_dispatch_done:
#endif

        ta       SPARC_SWTRAP_IRQDIS ! **** DISABLE INTERRUPTS ****

        /*
         *  While we had ISR dispatching disabled in this thread,
         *  did we miss anything?  If so, then we need to do another
         *  _Thread_Do_dispatch() before leaving this ISR dispatch context.
         */
        ldub    [%g6 + PER_CPU_DISPATCH_NEEDED], %l7

        orcc    %l7, %g0, %g0        ! Is a thread dispatch necessary?
        bne     isr_dispatch         ! Yes, then invoke the dispatcher again.
         mov    0, %o1               ! ISR level for _Thread_Do_dispatch()

        /*
         * No, then set the ISR dispatch disable flag to zero and continue with
         * the simple return.
         */
        st       %g0, [%g6 + PER_CPU_ISR_DISPATCH_DISABLE]

        /*
         *  The CWP in place at this point may be different from
         *  that which was in effect at the beginning of the ISR if we
         *  have been context switched between the beginning of this invocation
         *  of _SPARC_Interrupt_trap() and this point.  Thus the CWP and WIM
         *  should not be changed back to their values at ISR entry time.  Any
         *  changes to the PSR must preserve the CWP.
         */

simple_return:
        ld      [%fp + ISF_Y_OFFSET], %l5      ! restore y
        wr      %l5, 0, %y

        ldd     [%fp + ISF_PSR_OFFSET], %l0    ! restore psr, PC
        ld      [%fp + ISF_NPC_OFFSET], %l2    ! restore nPC
        rd      %psr, %l3
        and     %l3, SPARC_PSR_CWP_MASK, %l3   ! want "current" CWP
        andn    %l0, SPARC_PSR_CWP_MASK, %l0   ! want rest from task
        or      %l3, %l0, %l0                  ! install it later...
        andn    %l0, SPARC_PSR_ET_MASK, %l0

        /*
         *  Restore tasks global and out registers
         */

        mov    %fp, %g1

                                              ! g1 is restored later
        ldd     [%fp + ISF_G2_OFFSET], %g2    ! restore g2, g3
        ldd     [%fp + ISF_G4_OFFSET], %g4    ! restore g4, g5
        ld      [%fp + ISF_G7_OFFSET], %g7    ! restore g7

        ldd     [%fp + ISF_I0_OFFSET], %i0    ! restore i0, i1
        ldd     [%fp + ISF_I2_OFFSET], %i2    ! restore i2, i3
        ldd     [%fp + ISF_I4_OFFSET], %i4    ! restore i4, i5
        ldd     [%fp + ISF_I6_FP_OFFSET], %i6 ! restore i6/fp, i7

        /*
         *  Registers:
         *
         *   ALL global registers EXCEPT G1 and the input registers have
         *   already been restored and thuse off limits.
         *
         *   The following is the contents of the local registers:
         *
         *     l0 = original psr
         *     l1 = return address (i.e. PC)
         *     l2 = nPC
         *     l3 = CWP
         */

        /*
         *  if (CWP + 1) is an invalid window then we need to reload it.
         *
         *  WARNING: Traps should now be disabled
         */

        mov     %l0, %psr                  !  **** DISABLE TRAPS ****
        nop
        nop
        nop
        rd      %wim, %l4
        add     %l0, 1, %l6                ! l6 = cwp + 1
        and     %l6, SPARC_PSR_CWP_MASK, %l6 ! do the modulo on it
        srl     %l4, %l6, %l5              ! l5 = win >> cwp + 1 ; shift count
                                           !  and CWP are conveniently LS 5 bits
        cmp     %l5, 1                     ! Is tasks window invalid?
        bne     good_task_window

        /*
         *  The following code is the same as a 1 position left rotate of WIM.
         */

        sll     %l4, 1, %l5                ! l5 = WIM << 1
        srl     %l4, SPARC_NUMBER_OF_REGISTER_WINDOWS-1 , %l4
                                           ! l4 = WIM >> (Number Windows - 1)
        or      %l4, %l5, %l4              ! l4 = (WIM << 1) |
                                           !      (WIM >> (Number Windows - 1))

        /*
         *  Now restore the window just as if we underflowed to it.
         */

        wr      %l4, 0, %wim               ! WIM = new WIM
        nop                                ! must delay after writing WIM
        nop
        nop
        restore                            ! now into the tasks window

        ldd     [%g1 + CPU_STACK_FRAME_L0_OFFSET], %l0
        ldd     [%g1 + CPU_STACK_FRAME_L2_OFFSET], %l2
        ldd     [%g1 + CPU_STACK_FRAME_L4_OFFSET], %l4
        ldd     [%g1 + CPU_STACK_FRAME_L6_OFFSET], %l6
        ldd     [%g1 + CPU_STACK_FRAME_I0_OFFSET], %i0
        ldd     [%g1 + CPU_STACK_FRAME_I2_OFFSET], %i2
        ldd     [%g1 + CPU_STACK_FRAME_I4_OFFSET], %i4
        ldd     [%g1 + CPU_STACK_FRAME_I6_FP_OFFSET], %i6
                                           ! reload of sp clobbers ISF
        save                               ! Back to ISR dispatch window

good_task_window:
        TN0018_WAIT_IFLUSH %l3,%l4         ! GRLIB-TN-0018 work around macro

        mov     %l0, %psr                  !  **** DISABLE TRAPS ****
        nop; nop; nop
                                           !  and restore condition codes.
        ld      [%g1 + ISF_G1_OFFSET], %g1 ! restore g1
        TN0018_FIX %l3,%l4                 ! GRLIB-TN-0018 work around macro
        jmp     %l1                        ! transfer control and
        rett    %l2                        ! go back to tasks window

/* end of file */
