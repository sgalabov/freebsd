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
#include <mips/mtk/mtk_pciereg.h>

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
 
	return 880000000;
}

int
mtk_chip_pci_phy_init(device_t dev __unused)
{

        return (0);
}

#define _REG32(x)	*((volatile uint32_t *)(x))
#define GPIO_CTRL0	_REG32(0xbe000600)
#define GPIO_DATA0	_REG32(0xbe000620)

#if 0
static inline void
mt7621_set_pcie_phy(struct mtk_pci_softc *sc, uint32_t addr, int start_b,
    int bits, int val)
{
	uint32_t reg;

	reg = MT_READ32(sc, addr);
	reg &= ~(((1 << bits) - 1) << start_b);
	reg |= val << start_b;
	MT_WRITE32(sc, addr, reg);
}

static void
mt7621_pci_set_phy_for_ssc(struct mtk_pci_softc *sc, uint32_t offset)
{       
        uint32_t reg = (mtk_sysctl_get(SYSCTL_SYSCFG) >> 6) & 0x7;

        mt7621_set_pcie_phy(sc, offset + 0x400, 8, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x400, 9, 2, 0);
        mt7621_set_pcie_phy(sc, offset + 0x000, 4, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x100, 4, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x000, 5, 1, 0);
        mt7621_set_pcie_phy(sc, offset + 0x100, 5, 1, 0);
        if (reg <= 5 && reg >= 3) {
                mt7621_set_pcie_phy(sc, offset + 0x490,  6,  2, 1);
                mt7621_set_pcie_phy(sc, offset + 0x4a8,  0, 12, 0x1a);
                mt7621_set_pcie_phy(sc, offset + 0x4a8, 16, 12, 0x1a);
        } else {
                mt7621_set_pcie_phy(sc, offset + 0x490, 6, 2, 0);
                if (reg >= 6) {
                        mt7621_set_pcie_phy(sc, offset + 0x4bc,  4,  2, 0x1);
                        mt7621_set_pcie_phy(sc, offset + 0x49c,  0, 31, 0x18000000);
                        mt7621_set_pcie_phy(sc, offset + 0x4a4,  0, 16, 0x18d);
                        mt7621_set_pcie_phy(sc, offset + 0x4a8,  0, 12, 0x4a);
                        mt7621_set_pcie_phy(sc, offset + 0x4a8, 16, 12, 0x4a);
                        mt7621_set_pcie_phy(sc, offset + 0x4a8,  0, 12, 0x11);
                        mt7621_set_pcie_phy(sc, offset + 0x4a8, 16, 12, 0x11);
                } else {
                        mt7621_set_pcie_phy(sc, offset + 0x4a8,  0, 12, 0x1a);
                        mt7621_set_pcie_phy(sc, offset + 0x4a8, 16, 12, 0x1a);
                }
        }
        mt7621_set_pcie_phy(sc, offset + 0x4a0,  5, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x490, 22, 2, 2);
        mt7621_set_pcie_phy(sc, offset + 0x490, 18, 4, 6);
        mt7621_set_pcie_phy(sc, offset + 0x490, 12, 4, 2);
        mt7621_set_pcie_phy(sc, offset + 0x490,  8, 4, 1);
        mt7621_set_pcie_phy(sc, offset + 0x4ac, 16, 3, 0);
        mt7621_set_pcie_phy(sc, offset + 0x490,  1, 3, 2);

        if (reg <= 5 && reg >= 3) {
                mt7621_set_pcie_phy(sc, offset + 0x414, 6, 2, 1);
                mt7621_set_pcie_phy(sc, offset + 0x414, 5, 1, 1);
        }

        mt7621_set_pcie_phy(sc, offset + 0x414, 28, 2, 1);
        mt7621_set_pcie_phy(sc, offset + 0x040, 17, 4, 7);
        mt7621_set_pcie_phy(sc, offset + 0x040, 16, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x140, 17, 4, 7);
        mt7621_set_pcie_phy(sc, offset + 0x140, 16, 1, 1);

        mt7621_set_pcie_phy(sc, offset + 0x000,  5, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x100,  5, 1, 1);
        mt7621_set_pcie_phy(sc, offset + 0x000,  4, 1, 0);
        mt7621_set_pcie_phy(sc, offset + 0x100,  4, 1, 0);
}

static void
mt7621_pci_bypass_pipe_rst(struct mtk_pci_softc *sc)
{

        mt7621_set_pcie_phy(sc, 0x902c, 12, 1, 1);
        mt7621_set_pcie_phy(sc, 0x902c,  4, 1, 1);
        mt7621_set_pcie_phy(sc, 0x912c, 12, 1, 1);
        mt7621_set_pcie_phy(sc, 0x912c,  4, 1, 1);
        mt7621_set_pcie_phy(sc, 0xa02c, 12, 1, 1);
        mt7621_set_pcie_phy(sc, 0xa02c,  4, 1, 1);
}
#endif

static inline void
write_config(struct mtk_pci_softc *sc, uint32_t bus, uint32_t dev,
    uint32_t func, uint32_t reg, uint32_t *val)
{
        uint32_t address_reg, data_reg, address;

        address_reg = MTK_PCI_CFGADDR;
        data_reg = MTK_PCI_CFGDATA;

        address = (((reg & 0xf00) >> 8) << 24) | (bus << 16) | (dev << 11) |
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

        address = (((reg & 0xf00) >> 8) << 24) | (bus << 16) | (dev << 11) |
                   (func << 8) | (reg & 0xfc) | 0x80000000;

        MT_WRITE32(sc, address_reg, address);
        *val = MT_READ32(sc, data_reg);
}

static void
mt7621_pci_setup_port(struct mtk_pci_softc *sc, int port)
{
        uint32_t val;

        read_config(sc, 0, port, 0, 0x4, &val);
        val |= 0x4;
        write_config(sc, 0, port, 0, 0x4, &val);
        read_config(sc, 0, port, 0, 0x70c, &val);
        val &= ~(0xff) << 8;
        val |= 0x50 << 8;
        write_config(sc, 0, port, 0, 0x70c, &val);
        read_config(sc, 0, port, 0, 0x70c, &val);
        printf("Port %d N_FTS = 0x%08x\n", port, val);

	MT_WRITE32(sc, MTK_PCI_PCIE0_BAR0SETUP + 0x1000 * port, 0x7FFF0000);
        MT_WRITE32(sc, MTK_PCI_PCIE0_BAR1SETUP + 0x1000 * port, 0);
        MT_WRITE32(sc, MTK_PCI_PCIE0_IMBASEBAR0 + 0x1000 * port, 0);
        MT_WRITE32(sc, MTK_PCI_PCIE0_CLASS + 0x1000 * port, 0x06040001);
        MT_WRITE32(sc, MTK_PCI_PCIE0_BAR0SETUP + 0x1000 * port, 0x7FFF0001);
        MT_WRITE32(sc, MTK_PCI_PCIENA,
            MT_READ32(sc, MTK_PCI_PCIENA) | (1 << (20 + port)));
}

static inline void
mt7621_pci_set_reset(uint32_t val, int set)
{
	uint32_t res;

	res = mtk_sysctl_get(SYSCTL_RSTCTRL);
	if ((mtk_sysctl_get(SYSCTL_REVID) & 0xffff) == 0x0101) {
		if (set)
			res |= val;
		else
			res &= ~val;
	} else {
		if (set)
			res &= ~val;
		else
			res |= val;
	}
	mtk_sysctl_set(SYSCTL_RSTCTRL, res);
}

int
mtk_chip_pci_init(device_t dev)
{
        struct mtk_pci_softc *sc = device_get_softc(dev);
        uint32_t val;

        sc->pcie_link_status = 0;

#if 0
        val = PCIE0_RST | PCIE1_RST | PCIE2_RST;
	mt7621_pci_set_reset(val, 1);
        DELAY(100);

        val = mtk_sysctl_get(SYSCTL_GPIOMODE);
        val &= ~((0x3 << PCIE_SHARE_PIN_SW) | (0x3 << UARTL3_SHARE_PIN_SW));
        val |= ((0x1 << PCIE_SHARE_PIN_SW) | (0x1 << UARTL3_SHARE_PIN_SW));
        mtk_sysctl_set(SYSCTL_GPIOMODE, val);
#endif
        val = (0x1 << GPIO_PCIE_PORT0) | (0x1 << GPIO_PCIE_PORT1) |
                (0x1 << GPIO_PCIE_PORT2);

//        DELAY(50000);
//	GPIOBUS_PIN_SETFLAGS(busdev, dev, GPIO_PCIE_PORT0, GPIO_PIN_OUTPUT);
        GPIO_CTRL0 |= val;
//        DELAY(50000);
//	GPIOBUS_PIN_SET(busdev, dev, GPIO_PCIE_PORT0, GPIO_PIN_LOW);
        GPIO_DATA0 &= ~val;

        DELAY(100000);
     
#if 0   
        val = PCIE0_RST | PCIE1_RST | PCIE2_RST;
        mt7621_pci_set_reset(val, 0);

        val = mtk_sysctl_get(SYSCTL_CLKCFG1);
        val |= PCIE0_CLK_EN | PCIE1_CLK_EN | PCIE2_CLK_EN;
	mtk_sysctl_set(SYSCTL_CLKCFG1, val);
        DELAY(100000);

        if ((mtk_sysctl_get(SYSCTL_REVID) & 0xffff) == 0x0101)
                mt7621_pci_bypass_pipe_rst(sc);
        mt7621_pci_set_phy_for_ssc(sc, 0x9000);
        mt7621_pci_set_phy_for_ssc(sc, 0xa000);
#endif

        val = (1 << GPIO_PCIE_PORT0) | (1 << GPIO_PCIE_PORT1) |
                (1 << GPIO_PCIE_PORT2);
        GPIO_DATA0 |= val;

        DELAY(500000);

        if ((MT_READ32(sc, 0x2050) & 1) == 0) {
                printf("PCIe%d no card detected\n", 0);
        } else {
                sc->pcie_link_status |= (1 << 0);
                printf("PCIe%d card detected\n", 0);
        }

        if ((MT_READ32(sc, 0x3050) & 1) == 0) {
                printf("PCIe%d no card detected\n", 1);
        } else {
                sc->pcie_link_status |= (1 << 1);
                printf("PCIe%d card detected\n", 1);
        }

        if ((MT_READ32(sc, 0x4050) & 1) == 0) {
                printf("PCIe%d no card detected\n", 2);
        } else {
                sc->pcie_link_status |= (1 << 2);
                printf("PCIe%d card detected\n", 2);
        }

        if (sc->pcie_link_status == 0)
                return (0);

	MT_WRITE32(sc, MTK_PCI_MEMBASE, sc->sc_mem_base);
	MT_WRITE32(sc, MTK_PCI_IOBASE, sc->sc_io_base); // 0x1e160000

        mt7621_pci_setup_port(sc, 0);
        mt7621_pci_setup_port(sc, 1);
        mt7621_pci_setup_port(sc, 2);

	//if (1) return (-1);

	return (0);
}
