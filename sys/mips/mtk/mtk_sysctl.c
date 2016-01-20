/*-
 * Copyright (c) 2016 Stanislav Galabov.
 * Copyright (c) 2010 Aleksandr Rybalko.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>

#include <machine/fdt.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mtk/mtk_sysctlreg.h>
#include <mips/mtk/mtk_chip.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/fdt_clock.h>
#include <dev/fdt/fdt_reset.h>

#include "fdt_pinctrl_if.h"
#include "fdt_clock_if.h"
#include "fdt_reset_if.h"

static int	mtk_sysctl_probe(device_t);
static int	mtk_sysctl_attach(device_t);
static int	mtk_sysctl_detach(device_t);


static struct mtk_sysctl_softc *mtk_sysctl_softc = NULL;

static const struct ofw_compat_data mtk_compat_data[] = {
	{ "mtk,rt2880sh-soc",	(uintptr_t)MTK_CHIP_RT2880_SHUTTLE },
	{ "mtk,rt2880mp-soc",	(uintptr_t)MTK_CHIP_RT2880_MP },
	{ "mtk,rt2883-soc",	(uintptr_t)MTK_CHIP_RT2883 },
	{ "mtk,rt3050-soc",	(uintptr_t)MTK_CHIP_RT3050 },
	{ "mtk,rt3052-soc",	(uintptr_t)MTK_CHIP_RT3052 },
	{ "mtk,rt3350-soc",	(uintptr_t)MTK_CHIP_RT3350 },
	{ "mtk,rt3352-soc",	(uintptr_t)MTK_CHIP_RT3352 },
	{ "mtk,rt3662-soc",	(uintptr_t)MTK_CHIP_RT3662 },
	{ "mtk,rt3883-soc",	(uintptr_t)MTK_CHIP_RT3883 },
	{ "mtk,rt5350-soc",	(uintptr_t)MTK_CHIP_RT5350 },
	{ "mtk,rt6855-soc",	(uintptr_t)MTK_CHIP_RT6855 },
	{ "mtk,rt6856-soc",	(uintptr_t)MTK_CHIP_RT6856 },
	{ "mtk,mt7620-soc",	(uintptr_t)MTK_CHIP_MT7620 },
	{ "mtk,mt7621-soc",	(uintptr_t)MTK_CHIP_MT7621 },
	{ "mtk,mt7628-soc",	(uintptr_t)MTK_CHIP_MT7628 },
	{ "mtk,mt7688-soc",	(uintptr_t)MTK_CHIP_MT7688 },

	/* Sentinel */
	{ NULL,			(uintptr_t)MTK_CHIP_UNKNOWN }
};

static uint32_t mtk_chip_chipid = MTK_CHIP_UNKNOWN;
static uint32_t mtk_chip_cpuclk = 880000000;
static uint32_t mtk_chip_uartclk = 40000000;
static uint32_t mtk_chip_sysclk = 440000000;

#if 0
static void
mtk_sysctl_dump_config(device_t dev)
{
	uint32_t val;
#define DUMPREG(r) 							\
	val = mtk_sysctl_get(r); printf("    " #r "=%#08x\n", val)

	val = mtk_sysctl_get(SYSCTL_CHIPID0_3);
	printf("\tChip ID: \"%c%c%c%c", 
	    (val >> 0 ) & 0xff, 
	    (val >> 8 ) & 0xff, 
	    (val >> 16) & 0xff, 
	    (val >> 24) & 0xff);
	val = mtk_sysctl_get(SYSCTL_CHIPID4_7);
	printf("%c%c%c%c\"\n", 
	    (val >> 0 ) & 0xff, 
	    (val >> 8 ) & 0xff, 
	    (val >> 16) & 0xff, 
	    (val >> 24) & 0xff);

	DUMPREG(SYSCTL_SYSCFG);
#if !defined(RT5350) && !defined(MT7620) && 0
	if ( val & SYSCTL_SYSCFG_INIC_EE_SDRAM)
		printf("\tGet SDRAM config from EEPROM\n");
	if ( val & SYSCTL_SYSCFG_INIC_8MB_SDRAM)
		printf("\tBootstrap flag is set\n");
	printf("\tGE0 mode %u\n",
	    ((val & SYSCTL_SYSCFG_GE0_MODE_MASK) >> 
		SYSCTL_SYSCFG_GE0_MODE_SHIFT));
	if ( val & SYSCTL_SYSCFG_BOOT_ADDR_1F00)
		printf("\tBoot from 0x1f000000\n");
	if ( val & SYSCTL_SYSCFG_BYPASS_PLL)
		printf("\tBypass PLL\n");
	if ( val & SYSCTL_SYSCFG_BIG_ENDIAN)
		printf("\tBig Endian\n");
	if ( val & SYSCTL_SYSCFG_CPU_CLK_SEL_384MHZ)
		printf("\tClock is 384MHz\n");
	printf("\tBoot from %u\n",
	    ((val & SYSCTL_SYSCFG_BOOT_FROM_MASK) >> 
		SYSCTL_SYSCFG_BOOT_FROM_SHIFT));
	printf("\tBootstrap test code %u\n",
	    ((val & SYSCTL_SYSCFG_TEST_CODE_MASK) >> 
		SYSCTL_SYSCFG_TEST_CODE_SHIFT));
	printf("\tSRAM_CS mode %u\n",
	    ((val & SYSCTL_SYSCFG_SRAM_CS_MODE_MASK) >> 
		SYSCTL_SYSCFG_SRAM_CS_MODE_SHIFT));
	printf("\t%umA SDRAM_CLK driving\n",
	    (val & SYSCTL_SYSCFG_SDRAM_CLK_DRV)?12:8);

	DUMPREG(SYSCTL_CLKCFG0);
	printf("\tSDRAM_CLK_SKEW %uns\n", (val >> 30) & 0x03);

	DUMPREG(SYSCTL_CLKCFG1);
	if ( val & SYSCTL_CLKCFG1_PBUS_DIV_CLK_BY2)
		printf("\tPbus clock is 1/2 of System clock\n");
	if ( val & SYSCTL_CLKCFG1_OTG_CLK_EN)
		printf("\tUSB OTG clock is enabled\n");
	if ( val & SYSCTL_CLKCFG1_I2S_CLK_EN)
		printf("\tI2S clock is enabled\n");
	printf("\tI2S clock is %s\n", 
	    (val & SYSCTL_CLKCFG1_I2S_CLK_SEL_EXT)?
		"external":"internal 15.625MHz");
	printf("\tI2S clock divider %u\n",
	    ((val & SYSCTL_CLKCFG1_I2S_CLK_DIV_MASK) >> 
		SYSCTL_CLKCFG1_I2S_CLK_DIV_SHIFT));
	if ( val & SYSCTL_CLKCFG1_PCM_CLK_EN)
		printf("\tPCM clock is enabled\n");

	printf("\tPCM clock is %s\n", 
	    (val & SYSCTL_CLKCFG1_PCM_CLK_SEL_EXT)?
		"external":"internal 15.625MHz");
	printf("\tPCM clock divider %u\n",
	    ((val & SYSCTL_CLKCFG1_PCM_CLK_DIV_MASK) >> 
		SYSCTL_CLKCFG1_PCM_CLK_DIV_SHIFT));
	DUMPREG(SYSCTL_GPIOMODE);
#endif
#undef DUMPREG

	return;
}
#endif

static int
mtk_sysctl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "mtk,mtk-sysc"))
		return (ENXIO);

	device_set_desc(dev, "MTK System Control driver");

	return (0);
}

static int
mtk_sysctl_attach(device_t dev)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	int error = 0;

	KASSERT((device_get_unit(dev) == 0),
	    ("mtk_sysctl: Only one sysctl module supported"));

	if (mtk_sysctl_softc != NULL)
		return (ENXIO);

	mtk_sysctl_softc = sc;

	/* Map control/status registers. */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->mem_rid, RF_ACTIVE);

	if (sc->mem_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		error = ENXIO;
		mtk_sysctl_detach(dev);
		return(error);
	}

	mtk_chip_identify(dev);

	if ((error = bus_generic_attach(dev)))
		return (error);

	fdt_clock_register_provider(dev);
	fdt_reset_register_provider(dev);
	fdt_pinctrl_register(dev, "pinctrl-single,bits");
	fdt_pinctrl_configure_tree(dev);

	if (bootverbose)
		device_printf(dev, "GPIOMODE: 0x%08x\n",
		    bus_read_4(sc->mem_res, 0x60));

	{
		uint32_t val;

		val = *((volatile uint32_t *)0xb0181000);
		device_printf(dev, "asic: 0x%08x\n", val);
	}

	return (0);
}

static int
mtk_sysctl_detach(device_t dev)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);

	fdt_clock_unregister_provider(dev);
	fdt_reset_unregister_provider(dev);

	bus_generic_detach(dev);

	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);

	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);

	return(0);
}

uint32_t
mtk_sysctl_get(uint32_t reg)
{
	struct mtk_sysctl_softc *sc = mtk_sysctl_softc;
	return (bus_read_4(sc->mem_res, reg));
}

void
mtk_sysctl_set(uint32_t reg, uint32_t val)
{
	struct mtk_sysctl_softc *sc = mtk_sysctl_softc;
	bus_write_4(sc->mem_res, reg, val);
	return;
}

void
mtk_sysctl_clr_set(uint32_t reg, uint32_t clr, uint32_t set)
{
	uint32_t val;

	val = mtk_sysctl_get(reg);
	val &= ~clr;
	val |= set;
	mtk_sysctl_set(reg, val);
}

static int
mtk_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	phandle_t node;
	ssize_t i, len;
	uint32_t *value, *pconf;

	node = OF_node_from_xref(cfgxref);

	len = OF_getencprop_alloc(node, "pinctrl-single,bits",
	    sizeof(uint32_t) * 3, (void**)&value);
	if (len < 0) {
		device_printf(dev, "missing pinctrl-single,bits attribute\n");
		return (EINVAL);
	}

	pconf = value;
	for (i = 0; i < len; i++, pconf += 3) {
		if (pconf[0] != 0x60) {
			device_printf(dev, "wrong GPIOMODE register 0x%x\n",
			    pconf[0]);
			return (EINVAL);
		}
		bus_write_4(sc->mem_res, 0x60,
		    (bus_read_4(sc->mem_res, 0x60) & ~pconf[2]) | pconf[1]);
	}

	return (0);
}

static int
mtk_clock_enable(device_t dev, int index)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	uint32_t mask = (1u << index);

	if (index < 0 || index > 31)
		return (EINVAL);

	bus_write_4(sc->mem_res, 0x30,
	    bus_read_4(sc->mem_res, 0x30) | mask);

	return (0);
}

static int
mtk_clock_disable(device_t dev, int index)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	uint32_t mask = ~(1u << index);

	if (index < 0 || index > 31)
		return (EINVAL);

	bus_write_4(sc->mem_res, 0x30,
	    bus_read_4(sc->mem_res, 0x30) & mask);

	return (0);
}

static int
mtk_clock_get_info(device_t dev, int index, struct fdt_clock_info *info)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	uint32_t mask = (1u << index);

	if (index < 0 || index > 31 || info == NULL)
		return (EINVAL);

	if (bus_read_4(sc->mem_res, 0x30) & mask)
		info->flags = FDT_CIFLAG_RUNNING;
	else
		info->flags = 0;

	return (0);
}

static int
mtk_reset_assert(device_t dev, int index)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	uint32_t mask = (1u << index);

	if (index < 1 || index > 31)
		return (EINVAL);

	bus_write_4(sc->mem_res, 0x34,
	    bus_read_4(sc->mem_res, 0x34) | mask);

	return (0);
}

static int
mtk_reset_deassert(device_t dev, int index)
{
	struct mtk_sysctl_softc *sc = device_get_softc(dev);
	uint32_t mask = ~(1u << index);

	if (index < 1 || index > 31)
		return (EINVAL);

	bus_write_4(sc->mem_res, 0x34,
	    bus_read_4(sc->mem_res, 0x34) & mask);

	return (0);
}

static device_method_t mtk_sysctl_methods[] = {
	DEVMETHOD(device_probe,			mtk_sysctl_probe),
	DEVMETHOD(device_attach,		mtk_sysctl_attach),
	DEVMETHOD(device_detach,		mtk_sysctl_detach),

	/* pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure,	mtk_pinctrl_configure),

	/* fdt_clock interface */
	DEVMETHOD(fdt_clock_enable,		mtk_clock_enable),
	DEVMETHOD(fdt_clock_disable,		mtk_clock_disable),
	DEVMETHOD(fdt_clock_get_info,		mtk_clock_get_info),

	/* fdt_reset interface */
	DEVMETHOD(fdt_reset_assert,		mtk_reset_assert),
	DEVMETHOD(fdt_reset_deassert,		mtk_reset_deassert),

	{0, 0},
};

static driver_t mtk_sysctl_driver = {
	"sysc",
	mtk_sysctl_methods,
	sizeof(struct mtk_sysctl_softc),
};
static devclass_t mtk_sysctl_devclass;

EARLY_DRIVER_MODULE(mtk_sysctl, simplebus, mtk_sysctl_driver,
    mtk_sysctl_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_EARLY);


/* Early clock detection functions */
uint32_t
mtk_chip_get_uartclk(void)
{

	return mtk_chip_uartclk;
}

uint32_t
mtk_chip_get_sysclk(void)
{

	return mtk_chip_sysclk;
}

uint32_t
mtk_chip_get_cpuclk(void)
{

	return mtk_chip_cpuclk;
}

uint32_t
mtk_chip_get_chipid(void)
{

	return mtk_chip_chipid;
}

static inline uint32_t
mtk_detect_cpuclk_rt2880sh(uint32_t val)
{
	uint32_t res;

	switch (val) {
	case 0:
		res = 233333333;
		break;
	case 1:
		res = 250000000;
		break;
	case 2:
		res = 266666666;
		break;
	case 3:
		res = 280000000;
		break;
	}

	return (res);
}

static inline uint32_t
mtk_detect_cpuclk_rt2880mp(uint32_t val)
{
	uint32_t res;

	switch (val) {
	case 0:
		res = 250000000;
		break;
	case 1:
		res = 266666666;
		break;
	case 2:
		res = 280000000;
		break;
	case 3:
		res = 300000000;
		break;
	}

	return (res);
}

static inline uint32_t
mtk_detect_cpuclk_rt2883(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 20;
	val &= 0x03;

	if (mtk_chip_chipid == MTK_CHIP_RT2880_SHUTTLE)
		res = mtk_detect_cpuclk_rt2880sh(val);
	else if (mtk_chip_chipid == MTK_CHIP_RT2880_MP)
		res = mtk_detect_cpuclk_rt2880sh(val);
	else {
		switch (val) {
		case 0:
			res = 380000000;
			break;
		case 1:
			res = 390000000;
			break;
		case 2:
			res = 400000000;
			break;
		case 3:
			res = 420000000;
			break;
		}
	}

	return (res);
}

static uint32_t
mtk_detect_cpuclk_rt305x(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 18;
	val &= 0x01;

	if (mtk_chip_chipid == MTK_CHIP_RT3350 || !val) {
		res = 320000000;
	} else {
		res = 384000000;
	}

	return (res);
}

static uint32_t
mtk_detect_cpuclk_rt3352(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 8;
	val &= 0x01;

	if (val) {
		res = 400000000;
	} else {
		res = 384000000;
	}

	return (res);
}

static uint32_t
mtk_detect_cpuclk_rt3883(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 8;
	val &= 0x03;

	switch (val) {
	case 0:
		res = 250000000;
		break;
	case 1:
		res = 384000000;
		break;
	case 2:
		res = 480000000;
		break;
	case 3:
		res = 500000000;
		break;
	}

	return (res);
}

static uint32_t
mtk_detect_cpuclk_rt5350(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, val1, res;

	val = val1 = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 8;
	val &= 0x01;
	val1 >>= 10;
	val1 &= 0x1;
	val |= (val1 << 1);

	res = 0;

	switch (val) {
	case 0:
		res = 360000000;
		break;
	case 1:
		/* reserved value - panic? */
		break;
	case 2:
		res = 320000000;
		break;
	case 3:
		res = 300000000;
		break;
	}

	return (res);
}

static uint32_t
mtk_detect_cpuclk_rt6855(bus_space_tag_t bst, bus_space_handle_t bsh)
{

	return (400000000);
}

static uint32_t
mtk_detect_cpuclk_mt7620(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	if (((1u<<24) & bus_space_read_4(bst, bsh, SYSCTL_MT7620_CPLL_CFG1))) {
		res = 480000000;
	} else {
		val = bus_space_read_4(bst, bsh, SYSCTL_MT7620_CPLL_CFG0);
		if (!((1u<<31) & val)) {
			res = 600000000;
		} else {
			uint32_t mul, div;
			mul = 24 + ((val & 0x70000) >> 16);
			div = (val & 0xc00) >> 10;
			div = (div < 3) ? div + 2 : 8;

			res = (40 * mul) / div;
			res *= (1000 * 1000);
		}
	}
	
	return (res);
}

static uint32_t
mtk_detect_cpuclk_mt7621(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	bus_space_handle_t memh;
	uint32_t val, res;

	res = 0;

	val = bus_space_read_4(bst, bsh, SYSCTL_CLKCFG0);
	if (val & (1u<<30)) {
		uint32_t div;
		/*
		 * We need to temporarily map the MEMCTRL space, which is 0x5000
		 * away from the SYSCTL space
		 */
		if (bus_space_map(bst, 0x1e005000, 0x1000, 0, &memh))
			goto out_err;
		div = bus_space_read_4(bst, memh, 0x648);
		bus_space_unmap(bst, memh, 0x1000);
		div >>= 4;
		div &= 0x7f;
		div += 1;
		
		val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
		val >>= 6;
		val &= 0x07;

		if (val >= 6) {
			res = 25 * div;
		} else if (val >= 3) {
			res = 20 * div;
		} else {
			res = 0 * div;
		}
	} else {
		uint32_t div;
		val = bus_space_read_4(bst, bsh, SYSCTL_CUR_CLK_STS);
		div = (val >> 8) & 0x1f;
		val &= 0x1f;

		res = 500 * val / div;
	}

out_err:
	return (res * 1000 * 1000);
}

static uint32_t
mtk_detect_cpuclk_mt7628(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	if (val & 0x80) {
		res = 580000000;
	} else {
		res = 575000000;
	}

	return res;
}

static uint32_t
mtk_detect_sysclk_rt3883(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, val1, res;

	val1 = val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 17;
	val &= 0x01;

	val1 >>= 8;
	val1 &= 0x03;

	if (val) {
		switch (val1) {
		case 0:
			res = 125000000;
			break;
		case 1:
			res = 128000000;
			break;
		case 2:
			res = 160000000;
			break;
		case 3:
			res = 166000000;
			break; 
		}
	} else {
		switch (val1) {
		case 0:
			res = 83000000;
			break;
		case 1:
			res = 96000000;
			break;
		case 2:
			res = 120000000;
			break;
		case 4:
			res = 125000000;
			break;
		}
	}

	return (res);
}

static uint32_t
mtk_detect_sysclk_rt5350(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, val1, res;

	val = val1 = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 8;
	val &= 0x1;
	val1 >>= 10;
	val1 &= 0x1;
	val |= (val1 << 1);

	res = 0;

	switch (val) {
	case 0:
		res = 120000000;
		break;
	case 1:
		/* reserved value - panic? */
		break;
	case 2:
		res = 80000000;
		break;
	case 3:
		res = 100000000;
		break;
	}

	return (res);
}

static uint32_t
mtk_detect_sysclk_mt7620(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	uint32_t val, res;

	val = bus_space_read_4(bst, bsh, SYSCTL_SYSCFG);
	val >>= 4;
	val &= 0x03;

	if (val == 0)
		res = mtk_chip_cpuclk / 4;
	else
		res = mtk_chip_cpuclk / 3;

	return (res);
}

int
mtk_chip_early_detect(void)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;
	phandle_t node;
	u_long base, size;
	int i, res;

	if ((node = OF_finddevice("/")) == -1)
		return (-2); //(EINVAL);
	
	for (i = 0; mtk_compat_data[i].ocd_str != NULL; i++) {
		if (fdt_is_compatible(node, mtk_compat_data[i].ocd_str)) {
			mtk_chip_chipid = (uint32_t)mtk_compat_data[i].ocd_data;
			break;
		}
	}

	if (mtk_chip_chipid == MTK_CHIP_UNKNOWN)
		return (-3); //(EINVAL);

	/* Try to find the bus address to map to... */
	if((node = OF_finddevice("/soc")) == -1)
		return (-4); //(EINVAL);

	if (fdt_regsize(node, &base, &size) != 0)
		return (-5); //(EINVAL);

	bst = fdtbus_bs_tag;
	if (bus_space_map(bst, base, size, 0, &bsh))
		return (-6); //(EINVAL);

	res = 0;

	/* determine CPU clock */
	switch (mtk_chip_chipid) {
	case MTK_CHIP_RT2880_SHUTTLE: /* fallthrough */
	case MTK_CHIP_RT2880_MP: /* fallthrough */
	case MTK_CHIP_RT2883:
		mtk_chip_cpuclk = mtk_detect_cpuclk_rt2883(bst, bsh);
		break;
	case MTK_CHIP_RT3050: /* fallthrough */
	case MTK_CHIP_RT3052: /* fallthrough */
	case MTK_CHIP_RT3350:
		mtk_chip_cpuclk = mtk_detect_cpuclk_rt305x(bst, bsh);
		break;
	case MTK_CHIP_RT3352:
		mtk_chip_cpuclk = mtk_detect_cpuclk_rt3352(bst, bsh);
		break;
	case MTK_CHIP_RT3662: /* fallthrough */
	case MTK_CHIP_RT3883:
		mtk_chip_cpuclk = mtk_detect_cpuclk_rt3883(bst, bsh);
		break;
	case MTK_CHIP_RT5350:
		mtk_chip_cpuclk = mtk_detect_cpuclk_rt5350(bst, bsh);
		break;
	case MTK_CHIP_RT6855: /* fallthrough */
	case MTK_CHIP_RT6856:
		mtk_chip_cpuclk = mtk_detect_cpuclk_rt6855(bst, bsh);
		break;
	case MTK_CHIP_MT7620:
		mtk_chip_cpuclk = mtk_detect_cpuclk_mt7620(bst, bsh);
		break;
	case MTK_CHIP_MT7621:
		mtk_chip_cpuclk = mtk_detect_cpuclk_mt7621(bst, bsh);
		break;
	case MTK_CHIP_MT7628:
	case MTK_CHIP_MT7688:
		mtk_chip_cpuclk = mtk_detect_cpuclk_mt7628(bst, bsh);
		break;

	case MTK_CHIP_UNKNOWN: /* fallthrough */
	default:
		res = -7; //EINVAL;
		break;
	}

	if (res)
		goto out_err;

	/* determine system clock */
	switch (mtk_chip_chipid) {
	case MTK_CHIP_RT2880_SHUTTLE: /* fallthrough */
	case MTK_CHIP_RT2880_MP: /* fallthrough */
	case MTK_CHIP_RT2883:
		mtk_chip_sysclk = mtk_chip_cpuclk / 2;
		break;
	case MTK_CHIP_RT3662: /* fallthrough */
	case MTK_CHIP_RT3883:
		mtk_chip_sysclk = mtk_detect_sysclk_rt3883(bst, bsh);
		break;
	case MTK_CHIP_RT5350:
		mtk_chip_sysclk = mtk_detect_sysclk_rt5350(bst, bsh);
		break;
	case MTK_CHIP_RT6855: /* fallthrough */
	case MTK_CHIP_RT6856: /* fallthrough */
	case MTK_CHIP_MT7621:
		mtk_chip_sysclk = mtk_chip_cpuclk / 4;
		break;
	case MTK_CHIP_MT7620:
		mtk_chip_sysclk = mtk_detect_sysclk_mt7620(bst, bsh);
		break;

	default:
		mtk_chip_sysclk = mtk_chip_cpuclk / 3;
		break;
	}

	/* determine UART clock */
	switch (mtk_chip_chipid) {
	case MTK_CHIP_RT3352: /* fallthrough */
	case MTK_CHIP_RT3662: /* fallthrough */
	case MTK_CHIP_RT3883: /* fallthrough */
	case MTK_CHIP_RT5350: /* fallthrough */
	case MTK_CHIP_RT6855: /* fallthrough */
	case MTK_CHIP_RT6856: /* fallthrough */
	case MTK_CHIP_MT7620: /* fallthrough */
	case MTK_CHIP_MT7628: /* fallthrough */
	case MTK_CHIP_MT7688:
		mtk_chip_uartclk = 40000000;
		break;
	case MTK_CHIP_MT7621:
		mtk_chip_uartclk = 50000000;
		break;

	default:
		mtk_chip_uartclk = mtk_chip_sysclk;
		break;
	}

out_err:
	bus_space_unmap(bst, bsh, size);
	return (res);
}
