/*-
 * Copyright (c) 2016 Stanislav Galabov.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <machine/fdt.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_clock.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mtk/mtk_sysctlreg.h>
#include <mips/mtk/mtk_pciereg.h>
#include <mips/mtk/mtk_pcie_phyreg.h>

#define MEM_RID			0
#define RESET_ASSERT_DELAY	1000
#define RESET_DEASSERT_DELAY	10000

struct mtk_pcie_phy_softc {
	struct resource *	iores;
};

static int  mtk_pcie_phy_detach(device_t);
static int  mtk_pcie_phy_mt7621_init(device_t);
static int  mtk_pcie_phy_mt7628_init(device_t);
static int  mtk_pcie_phy_mt7620_init(device_t);

#define PCIE_PHY_WRITE(_sc, _off, _val)		\
	bus_write_4((_sc)->iores, (_off), (_val))
#define PCIE_PHY_READ(_sc, _off)		\
	bus_read_4((_sc)->iores, (_off))
#define PCIE_PHY_CLR_SET(_sc, _off, _clr, _set)	\
	bus_write_4((_sc)->iores, (_off), ((bus_read_4((_sc)->iores, (_off)) & \
	    ~(_clr)) | (_set)))

static int
mtk_pcie_phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "mtk,pcie-phy"))
		return (ENXIO);

	device_set_desc(dev, "MTK PCIe PHY");

	return (BUS_PROBE_DEFAULT);
}

static int
mtk_pcie_phy_attach(device_t dev)
{
	struct mtk_pcie_phy_softc *sc;
	phandle_t node;
	uint32_t chipid;
	int rid, res;

	/* Get our softc */
	sc = device_get_softc(dev);

	/* Get our FDT node and chip ID */
	node = ofw_bus_get_node(dev);
	chipid = mtk_chip_get_chipid();

	sc->iores = NULL;

	/* Get our resources, if needed */
	if (chipid == MTK_CHIP_MT7621 || chipid == MTK_CHIP_MT7628 ||
	    chipid == MTK_CHIP_MT7688 || chipid == MTK_CHIP_MT7620) {
		rid = MEM_RID;
		sc->iores = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
		    RF_ACTIVE);
		if (!sc->iores) {
			device_printf(dev, "Could not map memory\n");
			return (ENXIO);
		}
	}

	if (!OF_hasprop(node, "mtk,bypass-reset")) {
		/* If we have clocks defined - enable them */
		if (OF_hasprop(node, "clocks"))
			fdt_clock_enable_all(dev);

		/* If we have resets defined - perform a reset sequence */
		if (OF_hasprop(node, "resets")) {
			/*
			 * We need to special-case most MT7621 due to inverted
			 * reset
			 */
			if (chipid == MTK_CHIP_MT7621 &&
			    (mtk_sysctl_get(SYSCTL_REVID) &
			    SYSCTL_REVID_MASK) != SYSCTL_MT7621_REV_E) {
				fdt_reset_deassert_all(dev);
				DELAY(RESET_ASSERT_DELAY);
				fdt_reset_assert_all(dev);
				DELAY(RESET_DEASSERT_DELAY);
			} else {
				fdt_reset_assert_all(dev);
				DELAY(RESET_ASSERT_DELAY);
				fdt_reset_deassert_all(dev);
				DELAY(RESET_DEASSERT_DELAY);
			}
		}
	}

	res = 0;

	/* Some chips require specific PCIe PHY init... handle these */
	switch (chipid) {
	case MTK_CHIP_MT7628: /* Fallthrough */
	case MTK_CHIP_MT7688:
		res = mtk_pcie_phy_mt7628_init(dev);
		break;
	case MTK_CHIP_MT7621:
		res = mtk_pcie_phy_mt7621_init(dev);
		break;
	case MTK_CHIP_MT7620:
		res = mtk_pcie_phy_mt7620_init(dev);
		break;
	}

	/* We don't need the resources any more, release them */
	if (sc->iores) {
		bus_release_resource(dev, SYS_RES_MEMORY, MEM_RID, sc->iores);
		sc->iores = NULL;
	}

	if (res)
		mtk_pcie_phy_detach(dev);

	return (res);
}

static int
mtk_pcie_phy_detach(device_t dev)
{
	struct mtk_pcie_phy_softc *sc = device_get_softc(dev);
	phandle_t node;

	device_printf(dev, "Detaching\n");

	/* Release resources */
	if (sc->iores != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, MEM_RID, sc->iores);

	/* Get our FDT node */
	node = ofw_bus_get_node(dev);

	/* If we have resets defined - assert them */
	if (OF_hasprop(node, "resets"))
		fdt_reset_assert_all(dev);

	/* If we have clocks defined - disable them */
	if (OF_hasprop(node, "clocks"))
		fdt_clock_disable_all(dev);

	return (0);
}

static inline void
mtk_pcie_phy_set(struct mtk_pcie_phy_softc *sc, uint32_t reg,
    int start, int num, int val)
{
	uint32_t reg_val;

	reg_val = PCIE_PHY_READ(sc, reg);
	reg_val &= ~(((1 << num) - 1) << start);
	reg_val |= val << start;
	PCIE_PHY_WRITE(sc, reg, reg_val);
}

static void
mtk_pcie_phy_mt7621_bypass_pipe_rst(struct mtk_pcie_phy_softc *sc, uint32_t off)
{

	mtk_pcie_phy_set(sc, off + 0x002c, 12, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x002c,  4, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x012c, 12, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x012c,  4, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x102c, 12, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x102c,  4, 1, 1);
}

static void
mtk_pcie_phy_mt7621_setup_ssc(struct mtk_pcie_phy_softc *sc, uint32_t off)
{
	uint32_t xtal_sel;

	xtal_sel = mtk_sysctl_get(SYSCTL_SYSCFG) >> 6;
	xtal_sel &= 0x7;

	mtk_pcie_phy_set(sc, off + 0x400, 8, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x400, 9, 2, 0);
	mtk_pcie_phy_set(sc, off + 0x000, 4, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x100, 4, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x000, 5, 1, 0);
	mtk_pcie_phy_set(sc, off + 0x100, 5, 1, 0);

	if (xtal_sel <= 5 && xtal_sel >= 3) {
		mtk_pcie_phy_set(sc, off + 0x490,  6,  2, 1);
		mtk_pcie_phy_set(sc, off + 0x4a8,  0, 12, 0x1a);
		mtk_pcie_phy_set(sc, off + 0x4a8, 16, 12, 0x1a);
	} else {
		mtk_pcie_phy_set(sc, off + 0x490,  6,  2, 0);
		if (xtal_sel >= 6) {
			mtk_pcie_phy_set(sc, off + 0x4bc,  4,  2, 0x01);
			mtk_pcie_phy_set(sc, off + 0x49c,  0, 31, 0x18000000);
			mtk_pcie_phy_set(sc, off + 0x4a4,  0, 16, 0x18d);
			mtk_pcie_phy_set(sc, off + 0x4a8,  0, 12, 0x4a);
			mtk_pcie_phy_set(sc, off + 0x4a8, 16, 12, 0x4a);
			mtk_pcie_phy_set(sc, off + 0x4a8,  0, 12, 0x11);
			mtk_pcie_phy_set(sc, off + 0x4a8, 16, 12, 0x11);
		} else {
			mtk_pcie_phy_set(sc, off + 0x4a8,  0, 12, 0x1a);
			mtk_pcie_phy_set(sc, off + 0x4a8, 16, 12, 0x1a);
		}
	}

	mtk_pcie_phy_set(sc, off + 0x4a0,  5, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x490, 22, 2, 2);
	mtk_pcie_phy_set(sc, off + 0x490, 18, 4, 6);
	mtk_pcie_phy_set(sc, off + 0x490, 12, 4, 2);
	mtk_pcie_phy_set(sc, off + 0x490,  8, 4, 1);
	mtk_pcie_phy_set(sc, off + 0x4ac, 16, 3, 0);
	mtk_pcie_phy_set(sc, off + 0x490,  1, 3, 2);

	if (xtal_sel <= 5 && xtal_sel >= 3) {
		mtk_pcie_phy_set(sc, off + 0x414, 6, 2, 1);
		mtk_pcie_phy_set(sc, off + 0x414, 5, 1, 1);
	}

	mtk_pcie_phy_set(sc, off + 0x414, 28, 2, 1);
	mtk_pcie_phy_set(sc, off + 0x040, 17, 4, 7);
	mtk_pcie_phy_set(sc, off + 0x040, 16, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x140, 17, 4, 7);
	mtk_pcie_phy_set(sc, off + 0x140, 16, 1, 1);

	mtk_pcie_phy_set(sc, off + 0x000,  5, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x100,  5, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x000,  4, 1, 0);
	mtk_pcie_phy_set(sc, off + 0x100,  4, 1, 0);
}

static int
mtk_pcie_phy_mt7621_init(device_t dev)
{
	struct mtk_pcie_phy_softc *sc = device_get_softc(dev);

//	DELAY(100000);

	if ((mtk_sysctl_get(SYSCTL_REVID) & SYSCTL_REVID_MASK) == 
	    SYSCTL_MT7621_REV_E)
		mtk_pcie_phy_mt7621_bypass_pipe_rst(sc, 0x9000);

	mtk_pcie_phy_mt7621_setup_ssc(sc, 0x9000);
	mtk_pcie_phy_mt7621_setup_ssc(sc, 0xa000);

	return (0);
}

static void
mtk_pcie_phy_mt7628_setup(struct mtk_pcie_phy_softc *sc, uint32_t off)
{
	uint32_t xtal_sel;

	xtal_sel = mtk_sysctl_get(SYSCTL_SYSCFG) >> 6;
	xtal_sel &= 0x1;

	mtk_pcie_phy_set(sc, off + 0x400,  8, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x400,  9, 2, 0);
	mtk_pcie_phy_set(sc, off + 0x000,  4, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x000,  5, 1, 0);
	mtk_pcie_phy_set(sc, off + 0x4ac, 16, 3, 3);

	if (xtal_sel == 1) {
		mtk_pcie_phy_set(sc, off + 0x4bc, 24,  8, 0x7d);
		mtk_pcie_phy_set(sc, off + 0x490, 12,  4, 0x08);
		mtk_pcie_phy_set(sc, off + 0x490,  6,  2, 0x01);
		mtk_pcie_phy_set(sc, off + 0x4c0,  0, 32, 0x1f400000);
		mtk_pcie_phy_set(sc, off + 0x4a4,  0, 16, 0x013d);
		mtk_pcie_phy_set(sc, off + 0x4a8, 16, 16, 0x74);
		mtk_pcie_phy_set(sc, off + 0x4a8,  0, 16, 0x74);
	} else {
		mtk_pcie_phy_set(sc, off + 0x4bc, 24,  8, 0x64);
		mtk_pcie_phy_set(sc, off + 0x490, 12,  4, 0x0a);
		mtk_pcie_phy_set(sc, off + 0x490,  6,  2, 0x00);
		mtk_pcie_phy_set(sc, off + 0x4c0,  0, 32, 0x19000000);
		mtk_pcie_phy_set(sc, off + 0x4a4,  0, 16, 0x018d);
		mtk_pcie_phy_set(sc, off + 0x4a8, 16, 16, 0x4a);
		mtk_pcie_phy_set(sc, off + 0x4a8,  0, 16, 0x4a);
	}

	mtk_pcie_phy_set(sc, off + 0x498, 0, 8, 5);
	mtk_pcie_phy_set(sc, off + 0x000, 5, 1, 1);
	mtk_pcie_phy_set(sc, off + 0x000, 4, 1, 0);
}

static int
mtk_pcie_phy_mt7628_init(device_t dev)
{
	struct mtk_pcie_phy_softc *sc = device_get_softc(dev);

	mtk_pcie_phy_mt7628_setup(sc, 0x9000);

	return (0);
}

static int
mtk_pcie_phy_mt7620_wait_busy(struct mtk_pcie_phy_softc *sc)
{
	uint32_t reg_value, retry;

	reg_value = retry = 0;

	while (retry++ < MT7620_MAX_RETRIES) {
		reg_value = PCIE_PHY_READ(sc, MT7620_PCIE_PHY_CFG);
		if (reg_value & PHY_BUSY)
			DELAY(100000);
		else
			break;
	}

	if (retry >= MT7620_MAX_RETRIES)
		return (ENXIO);

	return (0);
}

static int
mtk_pcie_phy_mt7620_set(struct mtk_pcie_phy_softc *sc, uint32_t reg,
    uint32_t val)
{
	uint32_t reg_val;

	if (mtk_pcie_phy_mt7620_wait_busy(sc))
		return (ENXIO);

	reg_val = PHY_MODE_WRITE | ((reg & 0xff) << PHY_ADDR_OFFSET) |
	    (val & 0xff);
	PCIE_PHY_WRITE(sc, MT7620_PCIE_PHY_CFG, reg_val);
	DELAY(1000);

	if (mtk_pcie_phy_mt7620_wait_busy(sc))
		return (ENXIO);

	return (0);
}

static int
mtk_pcie_phy_mt7620_init(device_t dev)
{
	struct mtk_pcie_phy_softc *sc = device_get_softc(dev);
	int err;

	err = 0;
	err |= mtk_pcie_phy_mt7620_set(sc, 0x00, 0x80);
	err |= mtk_pcie_phy_mt7620_set(sc, 0x01, 0x04);
	err |= mtk_pcie_phy_mt7620_set(sc, 0x68, 0x84);

	if (err)
		return (err);

	PCIE_PHY_CLR_SET(sc, MTK_PCI_PCICFG, 0, (1<<1));

	fdt_reset_assert_all(dev);
	fdt_clock_disable_all(dev);

	mtk_sysctl_clr_set(MT7620_PPLL_DRV, LC_CKDRVPD, PDRV_SW_SET);

	fdt_clock_enable_all(dev);
	fdt_reset_deassert_all(dev);

	DELAY(100000);

	if (!(mtk_sysctl_get(MT7620_PPLL_CFG1) & PPLL_LOCKED)) {
		device_printf(dev, "PPLL not locked\n");
		fdt_reset_assert_all(dev);
		fdt_clock_disable_all(dev);
		return (ENXIO);
	}

	mtk_sysctl_clr_set(MT7620_PPLL_DRV, LC_CKDRVOHZ | LC_CKDRVHZ,
	    LC_CKDRVPD | PDRV_SW_SET);

	DELAY(100000);

	return (0);
}

static device_method_t mtk_pcie_phy_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_pcie_phy_probe),
	DEVMETHOD(device_attach,	mtk_pcie_phy_attach),
	DEVMETHOD(device_detach,	mtk_pcie_phy_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	DEVMETHOD_END
};

static driver_t mtk_pcie_phy_driver = {
	.name = "pciephy",
	.methods = mtk_pcie_phy_methods,
	.size = sizeof(struct mtk_pcie_phy_softc),
};

static devclass_t mtk_pcie_phy_devclass;

DRIVER_MODULE(pciephy, simplebus, mtk_pcie_phy_driver, mtk_pcie_phy_devclass, 0,
    0);
