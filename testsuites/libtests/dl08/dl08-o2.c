/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (c) 2014 Chris Johns <chrisj@rtems.org>.  All rights reserved.
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

#include "dl-load.h"
#include "dl-o2.h"
#include "dl-o3.h"

#include <rtems/test-printer.h>

#define printf(...) rtems_printf(&rtems_test_printer, __VA_ARGS__);

int   dl02_bss1;
float dl02_bss2[7];
char  dl02_bss3[21];
int   dl02_data1;
float dl02_data2;

#define DL_NAME       "dlo2"
#define PAINT_VAR(_v) sizeof(_v), &_v, _v

int rtems_main_o2 (void)
{
  printf (DL_NAME ": module: %s\n", dl_localise_file (__FILE__));
  printf (DL_NAME ":         dl02_bss1: %4zu: %p: %d\n",   PAINT_VAR (dl02_bss1));
  printf (DL_NAME ":         dl02_bss2: %4zu: %p: %f\n",   PAINT_VAR (dl02_bss2[0]));
  printf (DL_NAME ":         dl02_bss3: %4zu: %p: %02x\n", PAINT_VAR (dl02_bss3[0]));
  printf (DL_NAME ":        dl02_data1: %4zu: %p: %d\n",   PAINT_VAR (dl02_data1));
  /* no %f in the rtems test printer */
  printf (DL_NAME ":        dl02_data2: %4zu: %p: %f\n",   PAINT_VAR (dl02_data2));

  rtems_main_o3 ();

  return 0;
}
