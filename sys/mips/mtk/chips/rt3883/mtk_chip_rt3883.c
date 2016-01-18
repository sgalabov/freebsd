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
#include <sys/systm.h>
        
#include <sys/rman.h>
#include <sys/bus.h>
                
#include <machine/bus.h>
         
#include <mips/mtk/mtk_sysctlreg.h>
#include <mips/mtk/mtk_chip.h>
#include <mips/mtk/chips/mt7621/mtk_chip_mt7621.h>
#include <mips/mtk/mtk_pcireg.h>

void
mtk_chip_enable_usb_host(void)
{
}

void
mtk_chip_disable_usb_host(void)
{
}

uint32_t
mtk_chip_get_cpu_freq(void)
{
 
	return 500000000;
}

static inline void
write_config(struct mtk_pci_softc *sc, uint32_t bus, uint32_t dev,
    uint32_t func, uint32_t reg, uint32_t *val)
{
        uint32_t address_reg, data_reg, address;

        address_reg = MTK_PCI_CFGADDR;
        data_reg = MTK_PCI_CFGDATA;

        address = (bus << 16) | (dev << 11) |
                   (func << 8) | (reg & 0xfc) | 0x80000000;

        MT_WRITE32(sc, address_reg, address);
        MT_WRITE32(sc, data_reg, *val);
}

static inline void
read_config(struct mtk_pci_softc *sc, uint32_t bus, uint32_t dev,
    uint32_t func, uint32_t reg, uint32_t *val)
{
        uint32_t address_reg, data_reg, address;
 
        address_reg = MTK_PCI_CFGADDR;
        data_reg = MTK_PCI_CFGDATA;

        address = (bus << 16) | (dev << 11) |
                   (func << 8) | (reg & 0xfc) | 0x80000000;

        MT_WRITE32(sc, address_reg, address);
        *val = MT_READ32(sc, data_reg);
}

int
mtk_chip_pci_phy_init(device_t dev)
{

	return (0);
}

static void
rt3883_pci_setup_port(struct mtk_pci_softc *sc, int port)
{
	uint32_t val;
#if 0
	read_config(sc, 0, port, 0, 0x4, &val);
	val |= 0x4;
	write_config(sc, 0, port, 0, 0x4, &val);
	read_config(sc, 0, port, 0, 
#endif

	MT_WRITE32(sc, MTK_PCI_PCIE0_BAR0SETUP + 0x1000 * port, 0x03FF0001);
	MT_WRITE32(sc, MTK_PCI_PCIE0_BAR0SETUP + 0x1000 * port, 0x03FF0001);

	if (1) {
		val = 0x00000101;
		write_config(sc, 0, 0, 0, 0x1c, &val);
		read_config(sc, 0, 0, 0, 0x1c, &val);
		val = 0x06040001;
	} else {
		val = 0;
		write_config(sc, 0, 0, 0, 0x10, &val);
		read_config(sc, 0, 0, 0, 0x10, &val);
		val = 0x00800001;
	}
		

	MT_WRITE32(sc, MTK_PCI_PCIE0_BAR1SETUP + 0x1000 * port, 0);
	MT_WRITE32(sc, MTK_PCI_PCIE0_IMBASEBAR0 + 0x1000 * port, 0);
	MT_WRITE32(sc, MTK_PCI_PCIE0_ID + 0x1000 * port, 0x08021814);
        MT_WRITE32(sc, MTK_PCI_PCIE0_CLASS + 0x1000 * port, val);
	MT_WRITE32(sc, MTK_PCI_PCIE0_SUBID + 0x1000 * port, 0x28801814);
        MT_WRITE32(sc, MTK_PCI_PCIE0_BAR0SETUP + 0x1000 * port, 0x03FF0001);
        MT_WRITE32(sc, MTK_PCI_PCIENA, 0x00000000);
}

static void
rt3883_pci_dump_port(struct mtk_pci_softc *sc, int port)
{
#if 0
	uint32_t val;

	printf("MTK_PCI_PCIE0_BAR0SETUP : 0x%08x\n",
	    MT_READ32(sc, MTK_PCI_PCIE0_BAR0SETUP + 0x1000 * port));
	printf("MTK_PCI_PCIE0_BAR1SETUP : 0x%08x\n",
	    MT_READ32(sc, MTK_PCI_PCIE0_BAR1SETUP + 0x1000 * port));
	printf("MTK_PCI_PCIE0_IMBASEBAR0: 0x%08x\n",
	    MT_READ32(sc, MTK_PCI_PCIE0_IMBASEBAR0 + 0x1000 * port));
	printf("MTK_PCI_PCIE0_ID        : 0x%08x\n",
	    MT_READ32(sc, MTK_PCI_PCIE0_ID + 0x1000 * port));
	printf("MTK_PCI_PCIE0_CLASS     : 0x%08x\n",
	    MT_READ32(sc, MTK_PCI_PCIE0_CLASS + 0x1000 * port));
	printf("MTK_PCI_PCIE0_SUBID     : 0x%08x\n",
	    MT_READ32(sc, MTK_PCI_PCIE0_SUBID + 0x1000 * port));

	read_config(sc, 0, 0, 0, 0x10, &val);
	printf("VAL0                    : 0x%08x\n", val);
#if 1
	read_config(sc, 0, 0, 0, 0x1c, &val);
	printf("VAL1                    : 0x%08x\n", val);
#endif
#endif
}

int
mtk_chip_pci_init(device_t dev)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	uint32_t val;

	sc->has_pci = 1;

	val = mtk_sysctl_get(SYSCTL_RSTCTRL);
	val |= (1<<23)|(1<<24);
	mtk_sysctl_set(SYSCTL_RSTCTRL, val);

	val = mtk_sysctl_get(SYSCTL_CLKCFG1);
	val &= ~((1<<19)|(1<<21));
	val |= (1<<21);
	mtk_sysctl_set(SYSCTL_CLKCFG1, val);

	val = mtk_sysctl_get(SYSCTL_SYSCFG1);
	val &= ~(0x30);
	val |= (2<<4);
	val |= (3<<7);
	mtk_sysctl_set(SYSCTL_SYSCFG1, val);

	val = mtk_sysctl_get(/*SYSCTL_PCIE_CLK_GEN*/0x7c);
	val &= 0x7fffffff;
	mtk_sysctl_set(/*SYSCTL_PCIE_CLK_GEN*/0x7c, val);

	val = mtk_sysctl_get(/*SYSCTL_PCIE_CLK_GEN1*/0x80);
	val &= 0x80ffffff;
	val |= 0xa << 24;
	mtk_sysctl_set(/*SYSCTL_PCIE_CLK_GEN1*/0x80, val);

	val = mtk_sysctl_get(/*SYSCTL_PCIE_CLK_GEN*/0x7c);
	val |= 0x80000000;
	mtk_sysctl_set(/*SYSCTL_PCIE_CLK_GEN*/0x7c, val);

	val = mtk_sysctl_get(SYSCTL_GPIOMODE);
	val &= ~(0x3800);
	val |= (3 << 11);
	mtk_sysctl_set(SYSCTL_GPIOMODE, val);

	DELAY(50000);

	val = mtk_sysctl_get(SYSCTL_RSTCTRL);
	val &= ~(1<<23);
	mtk_sysctl_set(SYSCTL_RSTCTRL, val);

#if 0
	MT_WRITE32(sc, MTK_PCI_PCICFG, (1<<16)|(1<<1));
#else
	MT_WRITE32(sc, MTK_PCI_PCICFG, (0<<16)|(1<<1));
#endif

	DELAY(500000);

	MT_WRITE32(sc, MTK_PCI_PCICFG, 0);
#if 0
	MT_WRITE32(sc, MTK_PCI_PCICFG, (1<<16));
#endif

	DELAY(500000);

	if ((MT_READ32(sc, 0x2050) & 1) == 0) {
		printf("PCIe%d no card detected\n", 0);
	} else {
		sc->pcie_link_status |= (1 << 0); // was 1<<1
		printf("PCIe%d card detected\n", 0);
	}

//	MT_WRITE32(sc, MTK_PCI_ARBCTL, 0x79);

	MT_WRITE32(sc, MTK_PCI_MEMBASE, sc->sc_mem_base);
	MT_WRITE32(sc, MTK_PCI_IOBASE, sc->sc_io_base);

	rt3883_pci_setup_port(sc, 0);
//	rt3883_pci_setup_port(sc, 1);

	rt3883_pci_dump_port(sc, 0);
//	rt3883_pci_dump_port(sc, 1);

	return (0);
}
