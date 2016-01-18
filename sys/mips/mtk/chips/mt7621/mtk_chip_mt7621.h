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

#ifndef _MTK_CHIP_MT7621_H_
#define _MTK_CHIP_MT7621_H_

#define SYSCTL_SYSCFG1_USB0_HOST_MODE	(1<<10)

#define SYSCTL_CLKCFG1_UPHY0_CLK_EN	(1<<18)

#define SYSCTL_RSTCTRL_UPHY0		(1<<25)
#define SYSCTL_RSTCTRL_UPHY1		(1<<22)

#define GPIO_PCIE_PORT0			19
#define GPIO_PCIE_PORT1			8
#define GPIO_PCIE_PORT2			7

#define UARTL3_SHARE_PIN_SW		3
#define PCIE_SHARE_PIN_SW		10

#define PCIE0_CLK_EN             	(1 << 24)
#define PCIE1_CLK_EN             	(1 << 25)
#define PCIE2_CLK_EN             	(1 << 26)
#define PCIE0_RST                	(1 << 24)
#define PCIE1_RST                	(1 << 25)
#define PCIE2_RST                	(1 << 26)

#endif /* _MTK_CHIP_MT7621_H_ */
