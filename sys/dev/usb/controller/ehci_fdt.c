/*-
 * Copyright (c) 2016 Stanislav Galabov. 
 * Copyright (c) 2006 M. Warner Losh.
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

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_util.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/ehci.h>
#include <dev/usb/controller/ehcireg.h>

#include <sys/rman.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_clock.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define	MEM_RID			0
#define IRQ_RID			0

#define RESET_ASSERT_DELAY	1000	/* XXX: better value? */
#define RESET_DEASSERT_DELAY	10000	/* XXX: better value? */

#ifndef EHCI_HC_VENDORSTR
#define EHCI_HC_VENDORSTR	"Generic"
#endif

#ifndef EHCI_HC_DEVSTR
#define EHCI_HC_DEVSTR		"USB EHCI controller"
#endif

static device_probe_t  ehci_fdt_probe;
static device_attach_t ehci_fdt_attach;
static device_detach_t ehci_fdt_detach;

static int
ehci_fdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "usb-ehci"))
		return (ENXIO);
	device_set_desc(dev, EHCI_HC_VENDORSTR " " EHCI_HC_DEVSTR);

	return (BUS_PROBE_DEFAULT);
}

static int
ehci_fdt_attach(device_t dev)
{
	ehci_softc_t *sc = device_get_softc(dev);
	phandle_t node;
	int err;
	int rid;

	/* get our FDT node */
	node = ofw_bus_get_node(dev);

	/* initialise some bus fields */
	sc->sc_bus.parent = dev;
	sc->sc_bus.devices = sc->sc_devices;
	sc->sc_bus.devices_max = EHCI_MAX_DEVICES;
	sc->sc_bus.dma_bits = 32;

	/* get all DMA memory */
	if (usb_bus_mem_alloc_all(&sc->sc_bus,
	    USB_GET_DMA_TAG(dev), &ehci_iterate_hw_softc)) {
		return (ENOMEM);
	}

	rid = MEM_RID;
	sc->sc_io_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);

	if (!(sc->sc_io_res)) {
		goto error;
	}
	sc->sc_io_tag = rman_get_bustag(sc->sc_io_res);
	sc->sc_io_hdl = rman_get_bushandle(sc->sc_io_res);
	sc->sc_io_size = rman_get_size(sc->sc_io_res);

	rid = IRQ_RID;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (!(sc->sc_irq_res)) {
		goto error;
	}
	sc->sc_bus.bdev = device_add_child(dev, "usbus", -1);
	if (!(sc->sc_bus.bdev)) {
		goto error;
	}
	device_set_ivars(sc->sc_bus.bdev, &sc->sc_bus);

	strlcpy(sc->sc_vendor, EHCI_HC_VENDORSTR, sizeof(sc->sc_vendor));

	err = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, (driver_intr_t *)ehci_interrupt, sc, &sc->sc_intr_hdl);
	if (err) {
		sc->sc_intr_hdl = NULL;
		goto error;
	}

	/*
	 * If the device has 'clocks=' property - enable its clock(s).
	 */
	if (OF_hasprop(node, "clocks"))
		fdt_clock_enable_all(dev);

	/*
	 * If the device has 'resets=' property - assert its reset(s),
	 * wait a while, then deassert them and wait a while again in order
	 * to give it time to reset properly.
	 */
	if (OF_hasprop(node, "resets")) {
		fdt_reset_assert_all(dev);
		DELAY(RESET_ASSERT_DELAY);
		fdt_reset_deassert_all(dev);
		DELAY(RESET_DEASSERT_DELAY);
	}

	bus_space_write_4(sc->sc_io_tag, sc->sc_io_hdl, EHCI_USBCMD, 0);

	err = ehci_init(sc);
	if (!err) {
		err = device_probe_and_attach(sc->sc_bus.bdev);
	}
	if (err) {
		goto error;
	}
	return (0);

error:
	ehci_fdt_detach(dev);
	return (ENXIO);
}

static int
ehci_fdt_detach(device_t dev)
{
	ehci_softc_t *sc = device_get_softc(dev);
	device_t bdev;
	phandle_t node;
	int err;

	/* get our FDT node */
	node = ofw_bus_get_node(dev);

	if (sc->sc_bus.bdev) {
		bdev = sc->sc_bus.bdev;
		device_detach(bdev);
		device_delete_child(dev, bdev);
	}
	/* during module unload there are lots of children leftover */
	device_delete_children(dev);

	if (sc->sc_io_res != NULL) {
		/*
		 * Put the controller into reset, then disable clocks and do
		 * the MI tear down.  We have to disable the clocks/hardware
		 * after we do the rest of the teardown.
		 */
		bus_space_write_4(sc->sc_io_tag, sc->sc_io_hdl,
		    EHCI_USBCMD, 0);

		/*
		 * If the controller has 'resets=' property - assert all
		 * resets.
		 */
		if (OF_hasprop(node, "resets"))
			fdt_reset_assert_all(dev);

		/*
		 * If the controller has 'clocks=' property - stop all clocks.
		 */
		if (OF_hasprop(node, "clocks"))
			fdt_clock_disable_all(dev);

		if (sc->sc_irq_res && sc->sc_intr_hdl) {
			/*
			 * only call ehci_detach() after ehci_init()
			 */
			ehci_detach(sc);

			err = bus_teardown_intr(dev, sc->sc_irq_res,
			    sc->sc_intr_hdl);
			sc->sc_intr_hdl = NULL;
		}
		if (sc->sc_irq_res) {
			bus_release_resource(dev, SYS_RES_IRQ, IRQ_RID,
			    sc->sc_irq_res);
			sc->sc_irq_res = NULL;
		}
		if (sc->sc_io_res) {
			bus_release_resource(dev, SYS_RES_MEMORY, MEM_RID,
			    sc->sc_io_res);
			sc->sc_io_res = NULL;
		}
	}
	usb_bus_mem_free_all(&sc->sc_bus, &ehci_iterate_hw_softc);

	return (0);
}

static device_method_t ehci_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, ehci_fdt_probe),
	DEVMETHOD(device_attach, ehci_fdt_attach),
	DEVMETHOD(device_detach, ehci_fdt_detach),
	DEVMETHOD(device_suspend, bus_generic_suspend),
	DEVMETHOD(device_resume, bus_generic_resume),
	DEVMETHOD(device_shutdown, bus_generic_shutdown),

	DEVMETHOD_END
};

static driver_t ehci_fdt_driver = {
	.name = "ehci",
	.methods = ehci_fdt_methods,
	.size = sizeof(ehci_softc_t),
};

static devclass_t ehci_fdt_devclass;

DRIVER_MODULE(ehci, simplebus, ehci_fdt_driver, ehci_fdt_devclass, 0, 0);
MODULE_DEPEND(ehci, usb, 1, 1, 1);
