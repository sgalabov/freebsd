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

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_clock.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mtk/mtk_sysctlreg.h>

#define RESET_ASSERT_DELAY	1000
#define RESET_DEASSERT_DELAY	10000

struct mtk_usb_phy_softc {
	/* Empty for now */
};

static void mtk_usb_phy_mt7621_init(device_t);
static void mtk_usb_phy_mt7628_init(device_t);

static int
mtk_usb_phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "mtk,usb-phy"))
		return (ENXIO);

	device_set_desc(dev, "MTK USB PHY");

	return (BUS_PROBE_DEFAULT);
}

static int
mtk_usb_phy_attach(device_t dev)
{
	phandle_t node;
	uint32_t chipid, val;

	/* Get our FDT node */
	node = ofw_bus_get_node(dev);
	chipid = mtk_chip_get_chipid();

	/* Now let's see about setting USB to host or device mode */
	val = mtk_sysctl_get(SYSCTL_SYSCFG1);
	if (OF_hasprop(node, "mtk,usb-host"))
		val |= SYSCFG1_USB_HOST_MODE;
	else
		val &= ~SYSCFG1_USB_HOST_MODE;
	mtk_sysctl_set(SYSCTL_SYSCFG1, val);

	/* If we have clocks defined - enable them */
	if (OF_hasprop(node, "clocks"))
		fdt_clock_enable_all(dev);

	/* If we have resets defined - perform a reset sequence */
	if (OF_hasprop(node, "resets")) {
		fdt_reset_assert_all(dev);
		DELAY(RESET_ASSERT_DELAY);
		fdt_reset_deassert_all(dev);
		DELAY(RESET_DEASSERT_DELAY);
	}

	/* Some chips require specific USB PHY init... handle these */
	switch (chipid) {
	case MTK_CHIP_MT7628: /* Fallthrough */
	case MTK_CHIP_MT7688:
		mtk_usb_phy_mt7628_init(dev);
		break;
	case MTK_CHIP_MT7621:
		mtk_usb_phy_mt7621_init(dev);
		break;
	}

	return (0);
}

static int
mtk_usb_phy_detach(device_t dev)
{
	phandle_t node;

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

static void
mtk_usb_phy_mt7621_init(device_t dev)
{

	device_printf(dev, "PHY Init not implemented yet\n");
}

static void
mtk_usb_phy_mt7628_init(device_t dev)
{

	device_printf(dev, "PHY Init not implemented yet\n");
}

static device_method_t mtk_usb_phy_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_usb_phy_probe),
	DEVMETHOD(device_attach,	mtk_usb_phy_attach),
	DEVMETHOD(device_detach,	mtk_usb_phy_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	DEVMETHOD_END
};

static driver_t mtk_usb_phy_driver = {
	.name = "usbphy",
	.methods = mtk_usb_phy_methods,
	.size = sizeof(struct mtk_usb_phy_softc),
};

static devclass_t mtk_usb_phy_devclass;

DRIVER_MODULE(usbphy, simplebus, mtk_usb_phy_driver, mtk_usb_phy_devclass, 0,
    0);
