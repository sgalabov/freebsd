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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/systm.h>

#include <mips/mtk/mtk_sysctlreg.h>
#include <mips/mtk/mtk_chip.h>
#include <mips/mtk/chips/rt305x/mtk_chip_rt305x.h>

void
mtk_chip_enable_usb_host(void)
{
	uint32_t reg;

	/* Enable USB controller clock */
	reg = mtk_sysctl_get(SYSCTL_CLKCFG1);
	reg |= SYSCTL_CLKCFG1_OTG_CLK_EN;
	mtk_sysctl_set(SYSCTL_CLKCFG1, reg);
	DELAY(100000);

	/* Reset the USB controller */
	reg = mtk_sysctl_get(SYSCTL_RSTCTRL);
	reg |= SYSCTL_RSTCTRL_OTG;
	mtk_sysctl_set(SYSCTL_RSTCTRL, reg);
	DELAY(100000);

	/*
	 * Docs say that RSTCTRL bits for RT305x are W1C, so there should be
	 * no need for the below.
	 */
#if 0
	reg &= ~SYSCTL_RSTCTRL_OTG;
	mtk_sysctl_set(SYSCTL_RSTCTRL, reg);
	DELAY(100000);
#endif
}

void
mtk_chip_disable_usb_host(void)
{
	uint32_t reg;

	/* Put USB controller in reset */
	reg = mtk_sysctl_get(SYSCTL_RSTCTRL);
	reg |= SYSCTL_RSTCTRL_OTG;
	mtk_sysctl_set(SYSCTL_RSTCTRL, reg);

	/* Stop USB controller clock */
	reg = mtk_sysctl_get(SYSCTL_CLKCFG1);
	reg &= ~(SYSCTL_CLKCFG1_OTG_CLK_EN);
	mtk_sysctl_set(SYSCTL_CLKCFG1, reg);
}

uint32_t
mtk_chip_get_cpu_freq(void)
{

	return 384000000;
}

void
mtk_chip_reset(void)
{

	/*
	 * Unfortunately resetting via RSTCTRL doesn't work reliably for
	 * RT305x... so we resort to jumping to the beginning of the mapped
	 * boot media (flash or whatever it was we booted from)
	 */
	__asm __volatile("li	$25, 0xbf000000");
	__asm __volatile("j	$25");
}
