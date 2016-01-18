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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr_machdep.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <mips/mtk/mtk_sysctlreg.h>
#include <mips/mtk/mtk_pcireg.h>
#include <mips/mtk/mtk_chip.h>
#include <mips/mtk/chips/mt7620/mtk_chip_mt7620.h>

void
mtk_chip_enable_usb_host(void)
{
	uint32_t reg;

	/* Configure USB controller to host mode */
	reg = mtk_sysctl_get(SYSCTL_SYSCFG1);
	reg |= SYSCTL_SYSCFG1_USB0_HOST_MODE;
	mtk_sysctl_set(SYSCTL_SYSCFG1, reg);

	/* Enable USB controller clock */
	reg = mtk_sysctl_get(SYSCTL_CLKCFG1);
	reg |= SYSCTL_CLKCFG1_UPHY0_CLK_EN | SYSCTL_CLKCFG1_UPHY1_CLK_EN;
	mtk_sysctl_set(SYSCTL_CLKCFG1, reg);

	/* Reset the USB controller */
	reg = mtk_sysctl_get(SYSCTL_RSTCTRL);
	reg |= SYSCTL_RSTCTRL_UPHY0 | SYSCTL_RSTCTRL_UPHY1;
	mtk_sysctl_set(SYSCTL_RSTCTRL, reg);
	DELAY(100000);
	reg &= ~(SYSCTL_RSTCTRL_UPHY0 | SYSCTL_RSTCTRL_UPHY1);
	mtk_sysctl_set(SYSCTL_RSTCTRL, reg);
	DELAY(100000);
}

void
mtk_chip_disable_usb_host(void)
{
	uint32_t reg;

	/* Put USB controller in reset */
	reg = mtk_sysctl_get(SYSCTL_RSTCTRL);
	reg |= SYSCTL_RSTCTRL_UPHY0 | SYSCTL_RSTCTRL_UPHY1;
	mtk_sysctl_set(SYSCTL_RSTCTRL, reg);

	/* Stop USB controller clock */
	reg = mtk_sysctl_get(SYSCTL_CLKCFG1);
	reg &= ~(SYSCTL_CLKCFG1_UPHY0_CLK_EN | SYSCTL_CLKCFG1_UPHY1_CLK_EN);
	mtk_sysctl_set(SYSCTL_CLKCFG1, reg);
}

uint32_t
mtk_chip_get_cpu_freq(void)
{

	return 580000000;
}

#define MAX_RETRIES	10

static int
mtk_chip_wait_pci_phy_busy(struct mtk_pci_softc *sc)
{
	uint32_t reg_value = 0, retry = 0;

	while (retry++ < MAX_RETRIES) {
		reg_value = MT_READ32(sc, MTK_PCI_PHY0_CFG);
		if (reg_value & (0x80000000))
			DELAY(100000);
		else
			break;
	}

	if (retry >= MAX_RETRIES)
		return (-1);

	return (0);
}

static int
mtk_chip_pci_phy(struct mtk_pci_softc *sc, uint32_t addr, uint32_t val)
{
	uint32_t reg_val = 0x0;

	if (mtk_chip_wait_pci_phy_busy(sc))
		return (-1);

	reg_val |= (1<<23);		// Write mode
	reg_val |= (val);		// Data is at bit 0
	reg_val |= (addr) << 8;	// Address is at bit 8

	MT_WRITE32(sc, MTK_PCI_PHY0_CFG, reg_val);
	DELAY(1000);

	if(mtk_chip_wait_pci_phy_busy(sc))
		return (-1);

	return (0);
}

int
mtk_chip_pci_phy_init(device_t dev)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	uint32_t tmp;
	int err = 0;

	err |= mtk_chip_pci_phy(sc, 0x00, 0x80);
	err |= mtk_chip_pci_phy(sc, 0x01, 0x04);
	err |= mtk_chip_pci_phy(sc, 0x68, 0x84);

	if (err)
		return (err);

	mtk_sysctl_set(SYSCTL_RSTCTRL,
	    mtk_sysctl_get(SYSCTL_RSTCTRL) | (1<<26));
	mtk_sysctl_set(SYSCTL_CLKCFG1,
	    mtk_sysctl_get(SYSCTL_CLKCFG1) & ~(1<<26));

	tmp = mtk_sysctl_get(SYSCTL_PPLL_CFG1);
	tmp &= ~(1<<19);
	mtk_sysctl_set(SYSCTL_PPLL_CFG1, tmp);
	tmp |= (1<<31);
	mtk_sysctl_set(SYSCTL_PPLL_CFG1, tmp);

	return (0);
}

int
mtk_chip_pci_init(device_t dev)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	uint32_t tmp;

	mtk_sysctl_set(SYSCTL_SYSCFG1,
		mtk_sysctl_get(SYSCTL_SYSCFG1) | (1 << 8));

	mtk_sysctl_set(SYSCTL_GPIOMODE,
		mtk_sysctl_get(SYSCTL_GPIOMODE) & ~(0x3 << 16));
	mtk_sysctl_set(SYSCTL_RSTCTRL,
		mtk_sysctl_get(SYSCTL_RSTCTRL) & ~(1<<26));
	mtk_sysctl_set(SYSCTL_CLKCFG1,
		mtk_sysctl_get(SYSCTL_CLKCFG1) | (1<<26));

	tmp = mtk_sysctl_get(SYSCTL_PPLL_CFG1);
	if ((tmp & (1<<23)) == 0) {
		device_printf(dev, "PPLL not locked\n");
		return (-1);
	}

	tmp = mtk_sysctl_get(SYSCTL_PPLL_DRV);
	tmp |= (1<<19);
	mtk_sysctl_set(SYSCTL_PPLL_DRV, tmp);
	tmp &= ~(1<<18);
	mtk_sysctl_set(SYSCTL_PPLL_DRV, tmp);
	tmp &= ~(1<<17);
	mtk_sysctl_set(SYSCTL_PPLL_DRV, tmp);
	tmp|= (1<<31);
	mtk_sysctl_set(SYSCTL_PPLL_DRV, tmp);

	// Lift reset
	MT_WRITE32(sc, MTK_PCI_PCICFG, MT_READ32(sc, 0) & ~(1<<1));
	DELAY(500000);
	if ((MT_READ32(sc, MTK_PCI_PCIE0_STATUS) & 0x1) == 1)
		sc->pcie_link_status = 1;
	else
		sc->pcie_link_status = 0;

	// The code below should probably go into the main pci driver
	MT_WRITE32(sc, MTK_PCI_MEMBASE, sc->sc_mem_base);
	MT_WRITE32(sc, MTK_PCI_IOBASE, sc->sc_io_base);

	MT_WRITE32(sc, MTK_PCI_PCIE0_BAR0SETUP, 0x7FFF0001);
	MT_WRITE32(sc, MTK_PCI_PCIE0_BAR1SETUP, 0x00000000);
	MT_WRITE32(sc, MTK_PCI_PCIE0_IMBASEBAR0, 0x00000000);
	MT_WRITE32(sc, MTK_PCI_PCIE0_CLASS, 0x06040001);

	tmp = mtk_pci_read_config(dev, 0, 0, 0, 4, 4);
	mtk_pci_write_config(dev, 0, 0, 0, 4, tmp | 0x7, 4);
	tmp = mtk_pci_read_config(dev, 0, 0, 0, 0x70c, 4);
	tmp &= ~(0xff)<<8;
	tmp |= 0x50<<8;
	mtk_pci_write_config(dev, 0, 0, 0, 0x70c, tmp, 4);
	tmp = mtk_pci_read_config(dev, 0, 0, 0, 0x70c, 4);

	mtk_pci_write_config(dev, 0, 0, 0, PCIR_BAR(0), 0, 4);

	return (0);
}
