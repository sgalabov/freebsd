#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>

#include "opt_global.h"

#include <mips/mt762x/interrupt.h>
#include <mips/mt762x/gic.h>

uint32_t __gic_base, __cm_gcr_base;
static uint32_t __gic_base_phys;

void
gic_irq_ack(int irq)
{

	gic_write(gic_reg(0x280), irq);
}

static int
gic_detect_address(void)
{
	uint32_t reg_val;

	/* See if we have a config2 register */
	reg_val = mips_rd_config1();
	if ((reg_val & (1 << 31)) == 0) {
		printf("No config2 register\n");
		return (0);
	}

	/* Now check if we have a config3 register */
	reg_val = mips_rd_config2();
	if ((reg_val & (1 << 31)) == 0) {
		printf("No config3 register\n");
		return (0);
	}

	/* Now check if we should expect to have a CM GCR */
	reg_val = mips_rd_config3();
	if (!(reg_val & MIPS_CONFIG3_CMGCR_MASK)) {
		printf("No CM GCR\n");
		return (0);
	}

	/* Now see whether we actually have a CM GCR base defined... */
	reg_val = mips_rd_cmgcrbase();
	reg_val &= MIPS_CMGCRF_BASE;
	reg_val <<= (36 - 32);
	if (reg_val == 0) {
		printf("No CM GCR Base in cmgcrbase\n");
		return (0);
	}

	/* Now see whether the last value actually matches in memory... */
	__cm_gcr_base = MIPS_PHYS_TO_KSEG1(reg_val);
	printf("cm_gcr_base: 0x%08x\n", reg_val);
	if ((*cm_reg(0x8) & 0xffff8000) != reg_val) {
		printf("No match in memory (0x%08x)...\n", *cm_reg(0x8));
		return (0);
	}

	/* See if a GIC is supposed to exist... */
	if (!(*cm_reg(0xd0) & 0x1)) {
		printf("GIC doesn't exist 0x%08x\n", *cm_reg(0xd0));
		return (0);
	}

	/* Finally, read the GIC_BASE register... */
	reg_val = *cm_reg(0x80) & ~(0x1);
	printf("gic_base: 0x%08x\n", reg_val);

	return (reg_val);
}

static int
gic_find_address(void)
{

	__gic_base = __gic_base_phys = 0;

	if ((__gic_base_phys = gic_detect_address()) != 0) {
		__gic_base = MIPS_PHYS_TO_KSEG1(__gic_base_phys);
		return (0);
	}

	return (-1);
}

#define POLARITY	0x0100
#define TRIGGER		0x0180
#define SMASK		0x0380
#define RMASK		0x0300
#define MAP_PIN		0x0500

#define GIC_INTR_OFS(i)	(((i) / 32) * 4)
#define GIC_INTR_BIT(i)	((i) % 32)
#define GIC_INTR_VAL(i)	(1 << GIC_INTR_BIT(i))

void
gic_irq_map_to_pin(int irq, int pin)
{

	gic_write(gic_reg(MAP_PIN + (4 * irq)), (1 << 31) | pin);
}

#if defined(SMP) && defined(CPU_MIPS1004KC)
static void
gic_route_ipis(void)
{
	uint32_t i, vpe;

	gic_write(gic_reg(0x304), 0xf);

	// Route IPIs to PIN 4.
	for (i = 32; i < 36; i++) {
		gic_write(gic_reg(MAP_PIN + (4 * i)), (1 << 31) | 4);
	}

//	gic_write(gic_reg(0x280), 32);
//	gic_write(gic_reg(0x280), 33);
//	gic_write(gic_reg(0x280), 34);
//	gic_write(gic_reg(0x280), 35);
//	gic_write(gic_reg(0x104), 0xf);
	gic_write(gic_reg(0x184), 0xf);
	gic_write(gic_reg(0x204), 0x0);
	gic_write(gic_reg(0x384), 0xf);

	// Route IPIs to respective VPEs...
	for (i = 32, vpe = 0; vpe < 4; vpe++, i++) {
		gic_write(gic_reg(0x2000 + (32 * i) + ((vpe / 32) * 4)),
			(1 << (vpe % 32)));
		printf("%s(): irq%d, reg = 0x%08x\n", __FUNCTION__, i,
                        gic_read(gic_reg(0x2000 + (32 * i) + ((vpe/32) * 4))));
	}
}
#endif

static void
gic_route_interrupts(void)
{
#if 1
	gic_write(gic_reg(0x300), 0xffffffff);
	gic_write(gic_reg(0x100), 0xffffffff);
	gic_write(gic_reg(0x200), 0x0);
//	gic_write(gic_reg(0x400), 0xffffffff);
#endif
}

void
gic_irq_mask(int irq)
{

	gic_write(gic_reg(0x300), gic_read(gic_reg(0x300)) | (1 << irq));
}

void
gic_irq_unmask(int irq)
{

	gic_write(gic_reg(0x380), gic_read(gic_reg(0x380)) | (1 << irq));
}

static void
gic_dump_irq(void)
{
	 
	printf("0x00000180 -> 0x%08x, %d\n", gic_read(gic_reg(0x180)),
		(gic_read(gic_reg(0x180)) >> 26) & 1);
	printf("0x00000200 -> 0x%08x, %d\n", gic_read(gic_reg(0x200)),
		(gic_read(gic_reg(0x200)) >> 26) & 1);
	printf("0x00000380 -> 0x%08x, %d\n", gic_read(gic_reg(0x380)),
		(gic_read(gic_reg(0x380)) >> 26) & 1);
	printf("0x%08x -> 0x%08x\n", MAP_PIN + (4 * 26),
		gic_read(gic_reg(MAP_PIN + (4 * 26))));


	printf("0x00000400 -> 0x%08x\n", gic_read(gic_reg(0x400)));
	printf("0x00000480 -> 0x%08x\n", gic_read(gic_reg(0x480)));
}

uint32_t
gic_irq_get(void)
{
	uint32_t res = gic_read(gic_reg(0x480));

	res &= gic_read(gic_reg(0x400));

//	printf("%s(): 0x%08x\n", __FUNCTION__, res);

	return (res);
}

static void
gic_enable(void)
{

	gic_write(GCR_GIC_BASE, __gic_base_phys | 1);
}

void
gic_init(void)
{

	if (gic_find_address()) {
		printf("GIC not found\n");
#if defined(SMP) && defined(CPU_MIPS1004KC)
		panic("GIC not found, can't use IPIs");
#endif
		return;
	}

	gic_route_interrupts();

#if defined(SMP) && defined(CPU_MIPS1004KC)
	gic_route_ipis();
#endif

	gic_dump_irq();

	gic_enable();
}
