/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsARMZynq
 *
 * @brief This source file contains the implementation of the polled Zynq UART
 *   support.
 */

/*
 * Copyright (C) 2013, 2017 embedded brains GmbH & Co. KG
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

#include <dev/serial/zynq-uart.h>
#include <dev/serial/zynq-uart-regs.h>

#include <bspopts.h>

/*
 * Make weak and let the user override.
 */
uint32_t zynq_uart_input_clock(void) __attribute__ ((weak));

uint32_t zynq_uart_input_clock(void)
{
  return ZYNQ_CLOCK_UART;
}

int zynq_cal_baud_rate(uint32_t  baudrate,
                              uint32_t* brgr,
                              uint32_t* bauddiv,
                              uint32_t  modereg)
{
  uint32_t brgr_value;    /* Calculated value for baud rate generator */
  uint32_t calcbaudrate;  /* Calculated baud rate */
  uint32_t bauderror;     /* Diff between calculated and requested baud rate */
  uint32_t best_error = 0xFFFFFFFF;
  uint32_t percenterror;
  uint32_t bdiv;
  uint32_t inputclk = zynq_uart_input_clock();

  /*
   * Make sure the baud rate is not impossilby large.
   * Fastest possible baud rate is Input Clock / 2.
   */
  if ((baudrate * 2) > inputclk) {
    return -1;
  }
  /*
   * Check whether the input clock is divided by 8
   */
  if(modereg & ZYNQ_UART_MODE_CLKS) {
    inputclk = inputclk / 8;
  }

  /*
   * Determine the Baud divider. It can be 4to 254.
   * Loop through all possible combinations
   */
  for (bdiv = 4; bdiv < 255; bdiv++) {

    /*
     * Calculate the value for BRGR register
     */
    brgr_value = inputclk / (baudrate * (bdiv + 1));

    /*
     * Calculate the baud rate from the BRGR value
     */
    calcbaudrate = inputclk/ (brgr_value * (bdiv + 1));

    /*
     * Avoid unsigned integer underflow
     */
    if (baudrate > calcbaudrate) {
      bauderror = baudrate - calcbaudrate;
    }
    else {
      bauderror = calcbaudrate - baudrate;
    }

    /*
     * Find the calculated baud rate closest to requested baud rate.
     */
    if (best_error > bauderror) {
      *brgr = brgr_value;
      *bauddiv = bdiv;
      best_error = bauderror;
    }
  }

  /*
   * Make sure the best error is not too large.
   */
  percenterror = (best_error * 100) / baudrate;
#define XUARTPS_MAX_BAUD_ERROR_RATE		 3	/* max % error allowed */
  if (XUARTPS_MAX_BAUD_ERROR_RATE < percenterror) {
    return -1;
  }

  return 0;
}

void zynq_uart_initialize(volatile zynq_uart *regs)
{
  uint32_t brgr = 0x3e;
  uint32_t bauddiv = 0x6;
  uint32_t mode_clks = regs->mode & ZYNQ_UART_MODE_CLKS;

  while ((regs->channel_sts & ZYNQ_UART_CHANNEL_STS_TEMPTY) == 0 ||
         (regs->channel_sts & ZYNQ_UART_CHANNEL_STS_TACTIVE) != 0) {
    /* Wait */
  }

  zynq_cal_baud_rate(ZYNQ_UART_DEFAULT_BAUD, &brgr, &bauddiv, mode_clks);

  regs->control = 0;
  regs->control = ZYNQ_UART_CONTROL_RXDIS | ZYNQ_UART_CONTROL_TXDIS;
  regs->baud_rate_gen = ZYNQ_UART_BAUD_RATE_GEN_CD(brgr);
  regs->baud_rate_div = ZYNQ_UART_BAUD_RATE_DIV_BDIV(bauddiv);
  /* A Tx/Rx logic reset must be issued after baud rate manipulation */
  regs->control = ZYNQ_UART_CONTROL_RXDIS | ZYNQ_UART_CONTROL_TXDIS;
  regs->control = ZYNQ_UART_CONTROL_RXRES | ZYNQ_UART_CONTROL_TXRES;
  regs->rx_fifo_trg_lvl = ZYNQ_UART_RX_FIFO_TRG_LVL_RTRIG(0);
  regs->rx_timeout = ZYNQ_UART_RX_TIMEOUT_RTO(0);
  regs->control = ZYNQ_UART_CONTROL_RXEN | ZYNQ_UART_CONTROL_TXEN;
  regs->mode = ZYNQ_UART_MODE_CHMODE(ZYNQ_UART_MODE_CHMODE_NORMAL)
    | ZYNQ_UART_MODE_PAR(ZYNQ_UART_MODE_PAR_NONE)
    | ZYNQ_UART_MODE_CHRL(ZYNQ_UART_MODE_CHRL_8)
    | mode_clks;

  while (zynq_uart_read_char_polled(regs) >= 0) {
    /* Drop */
  }

  zynq_uart_reset_tx_flush(regs);
}

int zynq_uart_read_char_polled(volatile zynq_uart *regs)
{
  if ((regs->channel_sts & ZYNQ_UART_CHANNEL_STS_REMPTY) != 0) {
    return -1;
  } else {
    return ZYNQ_UART_TX_RX_FIFO_FIFO_GET(regs->tx_rx_fifo);
  }
}

void zynq_uart_write_char_polled(volatile zynq_uart *regs, char c)
{
  while ((regs->channel_sts & ZYNQ_UART_CHANNEL_STS_TNFUL) != 0) {
    /* Wait */
  }

  regs->tx_rx_fifo = ZYNQ_UART_TX_RX_FIFO_FIFO(c);
}

void zynq_uart_reset_tx_flush(volatile zynq_uart *regs)
{
  int c = 4;

  while (c-- > 0)
    zynq_uart_write_char_polled(regs, '\r');

  while ((regs->channel_sts & ZYNQ_UART_CHANNEL_STS_TEMPTY) == 0 ||
         (regs->channel_sts & ZYNQ_UART_CHANNEL_STS_TACTIVE) != 0) {
    /* Wait */
  }
}
