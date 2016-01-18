/*-
 * Copyright (c) 2016 Stanislav Galabov.
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
 * This is based on the pci allocator code from sys/dev/arm/mv/:
 *
 * Copyright (c) 2008 MARVELL INTERNATIONAL LTD.
 * Copyright (c) 2010 The FreeBSD Foundation
 * Copyright (c) 2010-2012 Semihalf
 * All rights reserved.
 *
 * Developed by Semihalf.
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/endian.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>
#include <machine/pmap.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <dev/pci/pcib_private.h>
#include "pcib_if.h"

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mtk/mtk_pcireg.h>
#include <mips/mtk/mtk_chip.h>

#include "pic_if.h"

struct mtx mtk_pci_mtx;
MTX_SYSINIT(mtk_pci_mtx, &mtk_pci_mtx, "MTK PCI/PCIe mutex", MTX_SPIN);

static int mtk_pcib_init(device_t, int, int);
static int mtk_pci_intr(void *);

static struct mtk_pci_softc *mt_sc = NULL;

static struct resource_spec mtk_pci_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE | RF_OPTIONAL },
	{ SYS_RES_IRQ,		2,	RF_ACTIVE | RF_OPTIONAL },
	{ -1, 0, 0 }
};

struct mtk_pci_range {
	u_long	base;
	u_long	len;
};

#if 0
#define FDT_RANGES_CELLS	((3 + 3 + 2) * 2)
#else
#define FDT_RANGES_CELLS	(3 * 2)
#endif

static void
mtk_pci_range_dump(struct mtk_pci_range *range)
{
#ifdef DEBUG
	printf("\n");
	printf("  base = 0x%08lx\n", range->base);
	printf("  len  = 0x%08lx\n", range->len);
#endif
}

static int
mtk_pci_ranges_decode(phandle_t node, struct mtk_pci_range *io_space,
    struct mtk_pci_range *mem_space)
{
	struct mtk_pci_range *pci_space;
	pcell_t ranges[FDT_RANGES_CELLS];
	pcell_t *rangesptr;
	pcell_t cell0, cell1, cell2;
	int tuples, i, rv, len;

	/*
	 * Retrieve 'ranges' property.
	 */
	if (!OF_hasprop(node, "ranges"))
		return (EINVAL);

	len = OF_getproplen(node, "ranges");
	if (len > sizeof(ranges))
		return (ENOMEM);

	if (OF_getprop(node, "ranges", ranges, sizeof(ranges)) <= 0)
		return (EINVAL);

	tuples = len / (sizeof(pcell_t) * 3);

	/*
	 * Initialize the ranges so that we don't have to worry about
	 * having them all defined in the FDT. In particular, it is
	 * perfectly fine not to want I/O space on PCI busses.
	 */
	bzero(io_space, sizeof(*io_space));
	bzero(mem_space, sizeof(*mem_space));

	rangesptr = &ranges[0];
	for (i = 0; i < tuples; i++) {
		cell0 = fdt_data_get((void *)rangesptr, 1);
		rangesptr++;
		cell1 = fdt_data_get((void *)rangesptr, 1);
		rangesptr++;
		cell2 = fdt_data_get((void *)rangesptr, 1);
		rangesptr++;

		if (cell0 == 2) {
			pci_space = mem_space;
		} else if (cell0 == 1) {
			pci_space = io_space;
		} else {
			rv = ERANGE;
			goto out;
		}

		pci_space->base = cell1;
		pci_space->len = cell2;
	}

	rv = 0;
out:
	return (rv);
}

static int
mtk_pci_ranges(phandle_t node, struct mtk_pci_range *io_space,
    struct mtk_pci_range *mem_space)
{
	int err;

	if ((err = mtk_pci_ranges_decode(node, io_space, mem_space)) != 0) {
		return (err);
	}

	mtk_pci_range_dump(io_space);
	mtk_pci_range_dump(mem_space);

	return (0);
}

static int
mtk_pci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "mtk,mtk-pci"))
		return (ENXIO);

	device_set_desc(dev, "MTK PCI bridge");

	return (BUS_PROBE_DEFAULT);
}

static int
mtk_pci_attach(device_t dev)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	struct mtk_pci_range io_space, mem_space;
	phandle_t node;
	intptr_t xref;
	int i;

	sc->has_pci = 0;

	if (bus_alloc_resources(dev, mtk_pci_spec, sc->pci_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* See how many interrupts we were given */
	sc->sc_num_irq = 0;
	for (i = 1; i < 4; i++)
		if (sc->pci_res[i] != NULL)
			sc->sc_num_irq++;
	sc->sc_irq_start = MTK_PCIE0_IRQ;
	sc->sc_irq_end = sc->sc_irq_start + sc->sc_num_irq - 1;

	sc->sc_dev = dev;
	mt_sc = sc;
	node = ofw_bus_get_node(dev);
	xref = OF_xref_from_node(node);

	if (mtk_pci_ranges(node, &io_space, &mem_space)) {
		device_printf(dev, "could not retrieve 'ranges' data\n");
		goto cleanup_res;
	}

	sc->sc_io_base = io_space.base;
	sc->sc_io_size = io_space.len;
	sc->sc_mem_base = mem_space.base;
	sc->sc_mem_size = mem_space.len;

	sc->sc_mem_rman.rm_type = RMAN_ARRAY;
	sc->sc_mem_rman.rm_descr = "mtk pci memory window";
	if (rman_init(&sc->sc_mem_rman) != 0 ||
	    rman_manage_region(&sc->sc_mem_rman, sc->sc_mem_base,
	    sc->sc_mem_base + sc->sc_mem_size - 1) != 0) {
		device_printf(dev, "failed to setup memory rman\n");
		goto cleanup_res;
	}

	sc->sc_io_rman.rm_type = RMAN_ARRAY;
	sc->sc_io_rman.rm_descr = "mtk pci io window";
	if (rman_init(&sc->sc_io_rman) != 0 ||
	    rman_manage_region(&sc->sc_io_rman, sc->sc_io_base,
	    sc->sc_io_base + sc->sc_io_size - 1) != 0) {
		device_printf(dev, "failed to setup io rman\n");
		goto cleanup_res;
	}

	sc->sc_irq_rman.rm_type = RMAN_ARRAY;
	sc->sc_irq_rman.rm_descr = "mtk pci irqs";
	if (rman_init(&sc->sc_irq_rman) != 0 ||
	    rman_manage_region(&sc->sc_irq_rman, sc->sc_irq_start,
	    sc->sc_irq_end) != 0) {
		device_printf(dev, "failed to setup irq rman\n");
		goto cleanup_res;
	}

	if(mtk_chip_pci_phy_init(dev) || mtk_chip_pci_init(dev)) {
		device_printf(dev, "pci phy init failed\n");
		goto cleanup_rman;
	}

	if (intr_pic_register(dev, xref) != 0) {
		device_printf(dev, "could not register PIC\n");
		goto cleanup_rman;
	}

	for (i = 1; i < 1 + sc->sc_num_irq; i++) {
		if (bus_setup_intr(dev, sc->pci_res[i], INTR_TYPE_MISC,
		    mtk_pci_intr, NULL, sc, &sc->pci_intrhand[i - 1])) {
			device_printf(dev, "could not setup intr handler %d\n",
			    i);
			goto cleanup;
		}
	}

	device_printf(dev, "interrupts registered\n");

	mtk_pcib_init(dev, 0, PCI_SLOTMAX);

	if (device_add_child(dev, "pci", -1) == NULL) {
		device_printf(dev, "could not attach pci bus\n");
		goto cleanup;
	}

	if (bus_generic_attach(dev)) {
		device_printf(dev, "could not attach to bus\n");
		goto cleanup;
	}

	return (0);

cleanup:
#ifdef notyet
	intr_pic_unregister(dev, xref);
#endif
	for (i = 1; i < 4; i++) {
		if (sc->pci_intrhand[i - 1] != NULL)
			bus_teardown_intr(dev, sc->pci_res[i],
			    sc->pci_intrhand[i - 1]);
	}
cleanup_rman:
	rman_fini(&sc->sc_irq_rman);
	rman_fini(&sc->sc_io_rman);
	rman_fini(&sc->sc_mem_rman);
cleanup_res:
	mt_sc = NULL;
	bus_release_resources(dev, mtk_pci_spec, sc->pci_res);
	return (ENXIO);
}

static int
mtk_pci_read_ivar(device_t dev, device_t child, int which,
	uintptr_t *result)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_DOMAIN:
		*result = device_get_unit(dev);
		return (0);
	case PCIB_IVAR_BUS:
		*result = sc->sc_busno;
		return (0);
	}

	return (ENOENT);
}

static int
mtk_pci_write_ivar(device_t dev, device_t child, int which,
	uintptr_t result)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_BUS:
		sc->sc_busno = result;
		return (0);
	}

	return (ENOENT);
}

static struct resource *
mtk_pci_alloc_resource(device_t bus, device_t child, int type, int *rid,
	u_long start, u_long end, u_long count, u_int flags)
{
	struct mtk_pci_softc *sc = device_get_softc(bus);
	struct resource *rv;
	struct rman *rm;
#if 0
	vm_offset_t va;
#endif

	printf("%s: entered\n", __FUNCTION__);

	switch (type) {
	case SYS_RES_IRQ:
		rm = &sc->sc_irq_rman;
		break;
	case SYS_RES_IOPORT:
		rm = &sc->sc_io_rman;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->sc_mem_rman;
		break;
	default:
		printf("%s: %d\n", __FUNCTION__, 1);
		return (NULL);
	}

	rv = rman_reserve_resource(rm, start, end, count, flags, child);

	if (rv == NULL) {
		printf("%s: %d (%s)\n", __FUNCTION__, 2, type == SYS_RES_IRQ ? "IRQ" : type == SYS_RES_MEMORY ? "MEM" : "IO" );
		return (NULL);
	}

	rman_set_rid(rv, *rid);

#if 0 /* mips bus_space code should now take care of this */
	if (type != SYS_RES_IRQ) {
		if (type == SYS_RES_MEMORY) {
			va = (vm_offset_t)pmap_mapdev(start, count);
		} else if (type == SYS_RES_IOPORT){
			va = (vm_offset_t)MIPS_PHYS_TO_KSEG1(start);
		}
		rman_set_bushandle(rv, va);
		rman_set_virtual(rv, (void *)va);
		rman_set_bustag(rv, mips_bus_space_generic);
	}
#endif

	if ((flags & RF_ACTIVE) && type != SYS_RES_IRQ) {
		if (bus_activate_resource(child, type, *rid, rv)) {
			rman_release_resource(rv);
			return (NULL);
		}
	}

	printf("%s: %d (%s)\n", __FUNCTION__, 3, rv == NULL ? "NULL" : "NONNULL");

	return (rv);
}

#if 0
static int
mtk_pci_activate_resource(device_t bus, device_t child, int type, int rid,
	struct resource *r)
{

	return rman_activate_resource(r);
}
#endif

static inline int
mtk_idx_to_irq(int idx)
{

	return ((idx == 0) ? MTK_PCIE0_IRQ :
		(idx == 1) ? MTK_PCIE1_IRQ :
		(idx == 2) ? MTK_PCIE2_IRQ : -1);
}

static inline int
mtk_irq_to_idx(int irq)
{

	return ((irq == MTK_PCIE0_IRQ) ? 0 :
		(irq == MTK_PCIE1_IRQ) ? 1 :
		(irq == MTK_PCIE2_IRQ) ? 2 : -1);
}

static void
mtk_pci_mask_irq(void *source)
{
	MT_WRITE32(mt_sc, MTK_PCI_PCIENA,
		MT_READ32(mt_sc, MTK_PCI_PCIENA) & ~(1<<((int)source)));
}

static void
mtk_pci_unmask_irq(void *source)
{

	MT_WRITE32(mt_sc, MTK_PCI_PCIENA,
		MT_READ32(mt_sc, MTK_PCI_PCIENA) | (1<<((int)source)));
}

static int
mtk_pci_setup_intr(device_t bus, device_t child, struct resource *ires,
	int flags, driver_filter_t *filt, driver_intr_t *handler,
	void *arg, void **cookiep)
{
	struct mtk_pci_softc *sc = device_get_softc(bus);
	struct intr_event *event;
	int irq, error, irqidx;

	printf("%s: %d\n", __FUNCTION__, 1);

	irq = rman_get_start(ires);

	if (irq < sc->sc_irq_start || irq > sc->sc_irq_end) {
		printf("%s: %d\n", __FUNCTION__, 2);
		return (EINVAL);
	}

	irqidx = irq - sc->sc_irq_start;

	event = sc->sc_eventstab[irqidx];
	if (event == NULL) {
		error = intr_event_create(&event, (void *)irq, 0, irq,
		    mtk_pci_mask_irq, mtk_pci_unmask_irq, NULL, NULL,
		    "pci intr%d:", irq);

		if (error == 0) {
			sc->sc_eventstab[irqidx] = event;
	//		sc->sc_intr_counter[irqidx] =
	//		    mips_intrcnt_create(event->ie_name);
		}
		else {
			printf("%s: %d\n", __FUNCTION__, 3);
			return (error);
		}
	}

	intr_event_add_handler(event, device_get_nameunit(child), filt,
		handler, arg, intr_priority(flags), flags, cookiep);

	//mips_intrcnt_setname(sc->sc_intr_counter[irqidx], event->ie_fullname);

	mtk_pci_unmask_irq((void*)irq);

	printf("%s: %d\n", __FUNCTION__, 4);

	return (0);
}

static int
mtk_pci_teardown_intr(device_t dev, device_t child, struct resource *ires,
	void *cookie)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	int irq, result, irqidx;

	irq = rman_get_start(ires);
	if ((irqidx = mtk_irq_to_idx(irq)) == -1)
		panic("%s: bad irq %d", __FUNCTION__, irq);

	if (sc->sc_eventstab[irqidx] == NULL)
		panic("Trying to teardown unoccupied IRQ");

	mtk_pci_mask_irq((void*)irq);

	result = intr_event_remove_handler(cookie);
	if (!result)
		sc->sc_eventstab[irqidx] = NULL;

	return (result);
}

static inline uint32_t
mtk_pci_make_addr(int bus, int slot, int func, int reg)
{
	uint32_t addr;

	//reg &= 0xff;

	addr = (((reg & 0xf00) >> 8) << 24) | (bus << 16) | (slot << 11) |
		(func << 8) | (reg & 0xfc) | (1 << 31);

	return (addr);
}

static int
mtk_pci_maxslots(device_t dev)
{

	return (PCI_SLOTMAX);
}

static inline int
mtk_pci_slot_has_link(device_t dev, int slot)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);

	if (sc->has_pci && slot >= 0x10) return 1;
                
	return !!(sc->pcie_link_status & (1<<slot));
}

uint32_t
mtk_pci_read_config(device_t dev, u_int bus, u_int slot, u_int func,
	u_int reg, int bytes)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	uint32_t addr = 0, data = 0;

	//printf("%s: bus %d link %d... ", __FUNCTION__, bus,
	//    sc->pcie_link_status & (1<<slot));

	if (bus == 0 && mtk_pci_slot_has_link(dev, slot) == 0) {
		//printf("fail, return 0x%08x\n", ~0U);
		return (~0U);
	}

	//printf("pass, ");

	mtx_lock_spin(&mtk_pci_mtx);
	addr = mtk_pci_make_addr(bus, slot, func, (reg & ~3));
	MT_WRITE32(sc, MTK_PCI_CFGADDR, addr);
	switch (bytes % 4) {
	case 0:
		data = MT_READ32(sc, MTK_PCI_CFGDATA);
		break;
	case 1:
		data = MT_READ8(sc, MTK_PCI_CFGDATA + (reg & 0x3));
		break;
	case 2:
		data = MT_READ16(sc, MTK_PCI_CFGDATA + (reg & 0x3));
		break;
	default:
		panic("%s(): Wrong number of bytes (%d) requested!\n",
			__FUNCTION__, bytes % 4);
	}
	mtx_unlock_spin(&mtk_pci_mtx);

	//printf("return 0x%08x\n", data);

	return (data);
}

void
mtk_pci_write_config(device_t dev, u_int bus, u_int slot, u_int func,
	u_int reg, uint32_t val, int bytes)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	uint32_t addr = 0, data = val;

	//printf("%s: bus %d link %d... ", __FUNCTION__, bus,
	    //sc->pcie_link_status & (1<<slot));

	if (bus == 0 && mtk_pci_slot_has_link(dev, slot) == 0) {
		//printf("fail, no write\n");
		return;
	}

	mtx_lock_spin(&mtk_pci_mtx);
	addr = mtk_pci_make_addr(bus, slot, func, (reg & ~3));
	MT_WRITE32(sc, MTK_PCI_CFGADDR, addr);
	switch (bytes % 4) {
	case 0:
		MT_WRITE32(sc, MTK_PCI_CFGDATA, data);
		break;
	case 1:
		MT_WRITE8(sc, MTK_PCI_CFGDATA + (reg & 0x3), data);
		break;
	case 2:
		MT_WRITE16(sc, MTK_PCI_CFGDATA + (reg & 0x3), data);
		break;
	default:
		panic("%s(): Wrong number of bytes (%d) requested!\n",
			__FUNCTION__, bytes % 4);
	}
	mtx_unlock_spin(&mtk_pci_mtx);
	//printf("pass, written\n");
}

#if 0
static int
mtk_pci_route_interrupt(device_t pcib, device_t device, int pin)
{
	//struct mtk_pci_softc *sc = device_get_softc(pcib);
	int bus, sl, dev;

	if (1) return PCI_INVALID_IRQ;

	bus = pci_get_bus(device);
	sl = pci_get_slot(device);
	dev = pci_get_device(device);

	printf("%s: for %d:%d:%d, int = %d\n", __FUNCTION__, bus, sl, dev, pin);

	if (bus != 0)
		panic("Unexpected bus number %d\n", bus);

	/* PCIe only */
	switch (sl) {
	case 0: return MTK_PCIE0_IRQ;
	case 1: return MTK_PCIE0_IRQ + 1;
	case 2: return MTK_PCIE0_IRQ + 2;
	default: return (-1);
	}

	return (-1);
}
#endif

static device_method_t mtk_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_pci_probe),
	DEVMETHOD(device_attach,	mtk_pci_attach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	mtk_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,	mtk_pci_write_ivar),
	DEVMETHOD(bus_alloc_resource,	mtk_pci_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
#if 0
	DEVMETHOD(bus_activate_resource,   mtk_pci_activate_resource),
#else
	DEVMETHOD(bus_activate_resource,   bus_generic_activate_resource),
#endif
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,	mtk_pci_setup_intr),
	DEVMETHOD(bus_teardown_intr,	mtk_pci_teardown_intr),

	/* pcib interface */
	DEVMETHOD(pcib_maxslots,	mtk_pci_maxslots),
	DEVMETHOD(pcib_read_config,	mtk_pci_read_config),
	DEVMETHOD(pcib_write_config,	mtk_pci_write_config),
#if 0
	DEVMETHOD(pcib_route_interrupt,	mtk_pci_route_interrupt),
#endif

	DEVMETHOD_END
};

static driver_t mtk_pci_driver = {
	"pcib",
	mtk_pci_methods,
	sizeof(struct mtk_pci_softc),
};

static devclass_t mtk_pci_devclass;

DRIVER_MODULE(mtk_pci, simplebus, mtk_pci_driver, mtk_pci_devclass, 0, 0);

static inline uint32_t
pcib_bit_get(uint32_t *map, uint32_t bit)
{
	uint32_t n = bit / BITS_PER_UINT32;

	bit = bit % BITS_PER_UINT32;
	return (map[n] & (1 << bit));
}

static inline void
pcib_bit_set(uint32_t *map, uint32_t bit)
{
	uint32_t n = bit / BITS_PER_UINT32;

	bit = bit % BITS_PER_UINT32;
	map[n] |= (1 << bit);
}

static inline uint32_t
pcib_map_check(uint32_t *map, uint32_t start, uint32_t bits)
{
	uint32_t i;

	for (i = start; i < start + bits; i++)
		if (pcib_bit_get(map, i))
			return (0);

	return (1);
}

static inline void
pcib_map_set(uint32_t *map, uint32_t start, uint32_t bits)
{
	uint32_t i;

	for (i = start; i < start + bits; i++)
		pcib_bit_set(map, i);
}

static bus_addr_t
pcib_alloc(device_t dev, uint32_t smask)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	uint32_t bits, bits_limit, i, *map, min_alloc, size;
	bus_addr_t addr = 0;
	bus_addr_t base;

	if (smask & 1) {
		base = sc->sc_io_base;
		min_alloc = PCI_MIN_IO_ALLOC;
		bits_limit = sc->sc_io_size / min_alloc;
		map = sc->sc_io_map;
		smask &= ~0x3;
	} else {
		base = sc->sc_mem_base;
		min_alloc = PCI_MIN_MEM_ALLOC;
		bits_limit = sc->sc_mem_size / min_alloc;
		map = sc->sc_mem_map;
		smask &= ~0xF;
	}

	size = ~smask + 1;
	bits = size / min_alloc;

	for (i = 0; i + bits <= bits_limit; i+= bits)
		if (pcib_map_check(map, i, bits)) {
			pcib_map_set(map, i, bits);
			addr = base + (i * min_alloc);
			return (addr);
		}

	return (addr);
}

static int
mtk_pcib_init_bar(device_t dev, int bus, int slot, int func, int barno)
{
	uint32_t addr, bar;
	int reg, width;

	reg = PCIR_BAR(barno);

	mtk_pci_write_config(dev, bus, slot, func, reg, ~0, 4);
	bar = mtk_pci_read_config(dev, bus, slot, func, reg, 4);
	if (bar == 0)
		return (1);

	/* Calculate BAR size: 64 or 32 bit (in 32-bit units) */
	width = ((bar & 7) == 4) ? 2 : 1;

	addr = pcib_alloc(dev, bar);
	if (!addr)
		return (-1);

	if (bootverbose)
		printf("PCI %u:%u:%u: reg %x: smask=%08x: addr=%08x\n",
		    bus, slot, func, reg, bar, addr);

	mtk_pci_write_config(dev, bus, slot, func, reg, addr, 4);
	if (width == 2)
		mtk_pci_write_config(dev, bus, slot, func, reg + 4, 0, 4);

	return (width);
}

static int
mtk_pcib_init_all_bars(device_t dev, int bus, int slot, int func,
	int hdrtype)
{
	int maxbar, bar, i;

	maxbar = (hdrtype & PCIM_HDRTYPE) ? 0 : 6;
	bar = 0;

	while (bar < maxbar) {
		i = mtk_pcib_init_bar(dev, bus, slot, func, bar);
		bar += i;
		if (i < 0) {
			device_printf(dev, "PCI IO/Memory space exhausted\n");
			return (ENOMEM);
		}
	}

	return (0);
}

static int cur_secbus = 0;

static void
mtk_pcib_init_bridge(device_t dev, int bus, int slot, int func)
{
	struct mtk_pci_softc *sc = device_get_softc(dev);
	bus_addr_t io_base, mem_base;
	uint32_t io_limit, mem_limit;
	int secbus;

	if (bus == 0 && !mtk_pci_slot_has_link(dev, slot)) {
		device_printf(dev, "Skip bus %d due to no link\n",++cur_secbus);
		return;
	}

	io_base = sc->sc_io_base;
	io_limit = io_base + sc->sc_io_size - 1;
	mem_base = sc->sc_mem_base;
	mem_limit = mem_base + sc->sc_mem_size - 1;

	mtk_pci_write_config(dev, bus, slot, func, PCIR_IOBASEL_1,
		io_base >> 8, 1);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_IOBASEH_1,
		io_base >> 16, 2);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_IOLIMITL_1,
		io_limit >> 8, 1);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_IOLIMITH_1,
		io_limit >> 16, 2);

	mtk_pci_write_config(dev, bus, slot, func, PCIR_MEMBASE_1,
		mem_base >> 16, 2);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_MEMLIMIT_1,
		mem_limit >> 16, 2);

	mtk_pci_write_config(dev, bus, slot, func, PCIR_PMBASEL_1,
		0x10, 2);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_PMBASEH_1,
		0x0, 4);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_PMLIMITL_1,
		0xF, 2);
	mtk_pci_write_config(dev, bus, slot, func, PCIR_PMLIMITH_1,
		0x0, 4);

	mtk_pci_write_config(dev, bus, slot, func, PCIR_INTLINE, 0xff, 1);

	secbus = mtk_pci_read_config(dev, bus, slot, func, PCIR_SECBUS_1, 1);

	if (secbus == 0) {
		mtk_pci_write_config(dev, bus, slot, func, PCIR_SECBUS_1,
			++cur_secbus, 1);
		mtk_pci_write_config(dev, bus, slot, func, PCIR_SUBBUS_1,
			cur_secbus, 1);
		secbus = cur_secbus;
	}

	mtk_pcib_init(dev, secbus, PCI_SLOTMAX);
}

static uint8_t
mtk_pci_get_int(device_t dev, int bus, int slot)
{

	if (slot != 0)
		return (PCI_INVALID_IRQ);

	switch (bus) {
	case 1:
		return (MTK_PCIE0_IRQ);
	case 2:
		return (MTK_PCIE1_IRQ);
	case 3:
		return (MTK_PCIE2_IRQ);
	default:
		device_printf(dev, "Bus %d out of range\n", slot);
		return (PCI_INVALID_IRQ);
	}

	/* Unreachable */
	return (PCI_INVALID_IRQ);
}

static int
mtk_pcib_init(device_t dev, int bus, int maxslot)
{
	int slot, func, maxfunc, error;
	uint8_t hdrtype, command, class, subclass;

	for (slot = 0; slot <= maxslot; slot++) {
		maxfunc = 0;
		for (func = 0; func <= maxfunc; func++) {
			hdrtype = mtk_pci_read_config(dev, bus, slot, func,
				PCIR_HDRTYPE, 1);

			if ((hdrtype & PCIM_HDRTYPE) > PCI_MAXHDRTYPE)
				continue;

			if (func == 0 && (hdrtype & PCIM_MFDEV))
				maxfunc = PCI_FUNCMAX;

			command = mtk_pci_read_config(dev, bus, slot, func,
				PCIR_COMMAND, 1);
			command &= ~(PCIM_CMD_MEMEN | PCIM_CMD_PORTEN);
			mtk_pci_write_config(dev, bus, slot, func,
				PCIR_COMMAND, command, 1);

			error = mtk_pcib_init_all_bars(dev, bus, slot, func,
				hdrtype);

			if (error)
				return (error);

			command |= PCIM_CMD_BUSMASTEREN | PCIM_CMD_MEMEN |
				PCIM_CMD_PORTEN;
			mtk_pci_write_config(dev, bus, slot, func,
				PCIR_COMMAND, command, 1);

			mtk_pci_write_config(dev, bus, slot, func,
				PCIR_CACHELNSZ, 16, 1);

			class = mtk_pci_read_config(dev, bus, slot, func,
				PCIR_CLASS, 1);
			subclass = mtk_pci_read_config(dev, bus, slot, func,
				PCIR_SUBCLASS, 1);

			if (class != PCIC_BRIDGE ||
			    subclass != PCIS_BRIDGE_PCI) {
				uint8_t val;

				val = mtk_pci_get_int(dev, bus, slot);

				mtk_pci_write_config(dev, bus, slot, func,
				    PCIR_INTLINE, val, 1); /* XXX */
				continue;
			}

			mtk_pcib_init_bridge(dev, bus, slot, func);
		}
	}

	return (0);
}

static int
mtk_pci_intr(void *arg)
{
	struct mtk_pci_softc *sc = arg;
	struct intr_event *event;
	uint32_t reg, irq, irqidx;

	reg = MT_READ32(sc, MTK_PCI_PCIINT);

	for (irq = sc->sc_irq_start; irq <= sc->sc_irq_end; irq++) {
		if (reg & (1u<<irq)) {
			irqidx = irq - sc->sc_irq_start;
			event = sc->sc_eventstab[irqidx];
			if (!event || TAILQ_EMPTY(&event->ie_handlers)) {
				if (irq != 0)
					printf("Stray PCI IRQ %d\n", irq);
				continue;
			}

			intr_event_handle(event, NULL);
			//mips_intrcnt_inc(sc->sc_intr_counter[irqidx]);
		}
	}

	return (FILTER_HANDLED);
}
