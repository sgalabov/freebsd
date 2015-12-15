/*-
 * Copyright (c) 2010 Smartcom - Bulgaria AD
 * Copyright (c) 2010 Stanislav Galabov <stanislav_galabov@smartcom.bg>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#define UART_CLOCK_DIVISOR	16
#define RALINK_SYSCTL_BASE	SYSCTL_BASE

#define UART_BASE	RALINK_SYSCTL_BASE
#define UART0		0x0500
#define UART1		0x0c00

#define CONFIG_CONSOLE	UART1

#define RBR_OFFSET	0x00
#define TBR_OFFSET	0x00
#define IER_OFFSET	0x04
#define IIR_OFFSET	0x08
#define FCR_OFFSET	0x08
#define LCR_OFFSET	0x0c
#define MCR_OFFSET	0x10
#define LSR_OFFSET	0x14
#define DLL_OFFSET	0x28

#define __REG(x)                (*((volatile uint32_t *) (x)))

#define RBR(x)		__REG(UART_BASE + (x) + RBR_OFFSET)
#define TBR(x)		__REG(UART_BASE + (x) + TBR_OFFSET)
#define IER(x)		__REG(UART_BASE + (x) + IER_OFFSET)
#define IIR(x)		__REG(UART_BASE + (x) + IIR_OFFSET)
#define FCR(x)		__REG(UART_BASE + (x) + FCR_OFFSET)
#define LCR(x)		__REG(UART_BASE + (x) + LCR_OFFSET)
#define MCR(x)		__REG(UART_BASE + (x) + MCR_OFFSET)
#define LSR(x)		__REG(UART_BASE + (x) + LSR_OFFSET)
#define DLL(x)		__REG(UART_BASE + (x) + DLL_OFFSET)

#define LCR_WLS0        (1 << 0)
#define LCR_WLS1        (1 << 1)
#define LCR_DLAB        (1 << 7)

#define LSR_TEMT        (1 << 6)
#define LSR_DR          (1 << 0)

#endif /* CONSOLE_H_ */
