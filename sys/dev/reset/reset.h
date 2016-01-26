/*-
 * Copyright 2016 Michal Meloun <mmel@FreeBSD.org>
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef DEV_RESET_RESET_H
#define DEV_RESET_RESET_H

#include "opt_platform.h"
#include <sys/types.h>

#include "rstnode_if.h"

typedef struct reset *reset_t;


/*
 * Provider interface
 */
#ifdef FDT
int reset_default_ofw_map(device_t provider_dev, phandle_t xref, int ncells,
    pcell_t *cells, intptr_t *id);
void reset_register_ofw_provider(device_t provider_dev);
void reset_unregister_ofw_provider(device_t provider_dev);
#endif

/*
 * Consumer interface
 */
int reset_get_by_id(device_t consumer_dev, device_t provider_dev, intptr_t id,
    reset_t *rst);
void reset_release(reset_t rst);

int reset_assert(reset_t rst);
int reset_clear(reset_t rst);
int reset_is_active(reset_t rst, bool *value);

#ifdef FDT
int reset_get_by_ofw_name(device_t consumer_dev, char *name, reset_t *rst);
int reset_get_by_ofw_idx(device_t consumer_dev, int idx, reset_t *rst);
#endif



#endif /* DEV_RESET_RESET_H */
