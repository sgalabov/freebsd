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

/* Prototypes */
void mtk_chip_print_basic_identity(void);
void mtk_chip_reset(void) __attribute__((weak));
void mtk_chip_init(void) __attribute__((weak));
void mtk_chip_identify(void) __attribute__((weak));
int  mtk_chip_pci_phy_init(device_t) __attribute__((weak));
int  mtk_chip_pci_init(device_t) __attribute__((weak));

/* Very basic chip identity, suitable for all chips */
void
mtk_chip_print_basic_identity(void)
{
	uint32_t val;

	val = mtk_sysctl_get(SYSCTL_CHIPID0_3);
	printf("Chip ID: %c%c%c%c",
	    (val >>  0) & 0xff,
	    (val >>  8) & 0xff,
	    (val >> 16) & 0xff,
	    (val >> 24) & 0xff);
	val = mtk_sysctl_get(SYSCTL_CHIPID4_7);
	printf("%c%c%c%c\n",
	    (val >>  0) & 0xff,
	    (val >>  8) & 0xff,
	    (val >> 16) & 0xff,
	    (val >> 24) & 0xff);
}

/* Default implementation, suitable for most MTK chips */
void
mtk_chip_reset(void)
{
 
	mtk_sysctl_set(SYSCTL_RSTCTRL, 1);
	while (1);
}
 
/* Empty chip init, suitable for most chips */
void
mtk_chip_init(void)
{
}

/* Generic chip identify, to be overriden by specific chips */
void
mtk_chip_identify(void)
{

	mtk_chip_print_basic_identity();
}

int
mtk_chip_pci_phy_init(device_t dev __unused)
{

	return (-1);
}

int
mtk_chip_pci_init(device_t dev __unused)
{

	return (-1);
}
