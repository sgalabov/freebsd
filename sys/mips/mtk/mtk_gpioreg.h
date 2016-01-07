/*-
 * Copyright (c) 2016 Stanislav Galabov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _MTK_GPIOREG_H_
#define _MTK_GPIOREG_H_

#define GPIO_PIOINT  (_sc)		0x0000
#define GPIO_PIOEDGE (_sc)		0x0004
#define GPIO_PIORENA (_sc)		0x0008
#define GPIO_PIOFENA (_sc)		0x000C
#define GPIO_PIODATA (_sc)		((_sc)->sc_unit == 0 ? 0x0020 : 0x0010)
#define GPIO_PIODIR  (_sc)		((_sc)->sc_unit == 0 ? 0x0024 : 0x0014)
#define GPIO_PIOPOL  (_sc)		((_sc)->sc_unit == 0 ? 0x0028 : 0x0018)
#define GPIO_PIOSET  (_sc)		((_sc)->sc_unit == 0 ? 0x002C : 0x001C)
#define GPIO_PIORESET(_sc)		((_sc)->sc_unit == 0 ? 0x0030 : 0x0020)
#define GPIO_PIOTOG  (_sc)		((_sc)->sc_unit == 0 ? 0x0034 : 0x0024)

#endif /* _MTK_GPIOREG_H_ */
