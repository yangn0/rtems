/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSTestFrameworkImpl
 *
 * @brief This source file contains the implementation of
 *   T_report_hash_sha256_update() and T_report_hash_sha256().
 */

/*
 * Copyright (C) 2019, 2021 embedded brains GmbH & Co. KG
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

#include <rtems/test.h>

#include <limits.h>

#if defined(__rtems__)
#include <sha256.h>
#else
#include <openssl/sha.h>
#endif

#include <rtems/base64.h>

typedef struct {
	SHA256_CTX sha256;
	T_putchar putchar;
	void *putchar_arg;
} T_report_hash_sha256_context;

static T_report_hash_sha256_context T_report_hash_sha256_instance;

void
T_report_hash_sha256_update(char c)
{
	T_report_hash_sha256_context *ctx;

	ctx = &T_report_hash_sha256_instance;
	SHA256_Update(&ctx->sha256, &c, sizeof(c));
}

static void
T_report_hash_sha256_putchar(int c, void *arg)
{
	T_report_hash_sha256_context *ctx;
	char cc;

	ctx = arg;
	cc = (char)c;
	SHA256_Update(&ctx->sha256, &cc, sizeof(cc));
	(*ctx->putchar)(c, ctx->putchar_arg);
}

static void
T_report_hash_sha256_initialize(void)
{
	T_report_hash_sha256_context *ctx;

	ctx = &T_report_hash_sha256_instance;
	SHA256_Init(&ctx->sha256);
	T_set_putchar(T_report_hash_sha256_putchar, ctx, &ctx->putchar,
	    &ctx->putchar_arg);
}

static void
T_report_hash_sha256_finalize(void)
{
	T_report_hash_sha256_context *ctx;
	unsigned char hash[32];

	ctx = &T_report_hash_sha256_instance;
	SHA256_Final(hash, &ctx->sha256);
	T_printf("Y:ReportHash:SHA256:");
	(void)_Base64url_Encode(ctx->putchar, ctx->putchar_arg, hash,
	    sizeof(hash), NULL, INT_MAX);
	T_printf("\n");
}

void
T_report_hash_sha256(T_event event, const char *name)
{
	(void)name;

	switch (event) {
	case T_EVENT_RUN_INITIALIZE_EARLY:
		T_report_hash_sha256_initialize();
		break;
	case T_EVENT_RUN_FINALIZE_LATE:
		T_report_hash_sha256_finalize();
		break;
	default:
		break;
	};
}
