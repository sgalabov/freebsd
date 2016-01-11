/*-
 * Copyright (c) 2016 Stanislav Galabov.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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
 *
 * $FreeBSD$
 */

#ifndef _MTK_SYSCTLREG_H_
#define _MTK_SYSCTLREG_H_

/* System Control */
#define SYSCTL_CHIPID0_3	0x00
#define SYSCTL_CHIPID4_7	0x04

#define SYSCTL_SYSCFG		0x10
#define SYSCTL_SYSCFG1		0x14
#define SYSCTL_CLKCFG0		0x2C
#define SYSCTL_CLKCFG1		0x30
#define SYSCTL_RSTCTRL		0x34

#define SYSCTL_CUR_CLK_STS	0x44

#define SYSCTL_MT7620_CPLL_CFG0	0x54
#define SYSCTL_MT7620_CPLL_CFG1	0x58

struct mtk_sysctl_softc {
	device_t		dev;
	struct resource		*mem_res;
	int			mem_rid;
	struct resource		*irq_res;
	int			irq_rid;
	int			sysctl_ih;
};

extern uint32_t	mtk_sysctl_get(uint32_t);
extern void	mtk_sysctl_set(uint32_t, uint32_t);

enum mtk_chip_id {
	MTK_CHIP_UNKNOWN,
	MTK_CHIP_RT2880_SHUTTLE,
	MTK_CHIP_RT2880_MP,
	MTK_CHIP_RT2883,
	MTK_CHIP_RT3050,
	MTK_CHIP_RT3052,
	MTK_CHIP_RT3350,
	MTK_CHIP_RT3352,
	MTK_CHIP_RT3662,
	MTK_CHIP_RT3883,
	MTK_CHIP_RT5350,
	MTK_CHIP_RT6855,
	MTK_CHIP_RT6856,
	MTK_CHIP_MT7620,
	MTK_CHIP_MT7621,
	MTK_CHIP_MT7628,
	MTK_CHIP_MT7688,
	MTK_CHIP_MAX
};

extern int	mtk_chip_early_detect(void);
extern uint32_t	mtk_chip_get_cpuclk(void);
extern uint32_t	mtk_chip_get_sysclk(void);
extern uint32_t	mtk_chip_get_uartclk(void);
extern uint32_t	mtk_chip_get_chipid(void);

#endif /* _MTK_SYSCTLREG_H_ */
