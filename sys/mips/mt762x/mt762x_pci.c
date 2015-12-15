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
#include <machine/intr_machdep.h>
#include <machine/pmap.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <dev/pci/pcib_private.h>
#include "pcib_if.h"

#include <mips/mt762x/rt305xreg.h>
#include <mips/mt762x/mt762x_pcivar.h>
#include <mips/mt762x/mt762x_pci_bus_space.h>
#include <mips/mt762x/gic.h>

#define PCIE_PHY_SSC

#define DEBUG
#ifdef DEBUG
#define NOT_IMPL()	printf("%s(): not implemented yet.\n", __FUNCTION__)
#define dprintf		printf
#else
#define NOT_IMPL()
#define dprintf(x, arg...)
#endif

struct mtx mt762x_pci_mtx;
MTX_SYSINIT(mt762x_pci_mtx, &mt762x_pci_mtx, "mt762x PCI space mutex",
    MTX_SPIN);

#define PCI_MIN_IO_ALLOC	4
#define PCI_MIN_MEM_ALLOC	16

#define BITS_PER_UINT32		(NBBY * sizeof(uint32_t))

struct mt762x_pci_softc {
        device_t                sc_dev;

        int                     sc_busno;
        int                     sc_baseslot;
        struct rman             sc_mem_rman;
        struct rman             sc_irq_rman;

	bus_addr_t		sc_mem_base;
	bus_addr_t		sc_mem_size;
	uint32_t		sc_mem_map[(512*1024*1024)/
				(PCI_MIN_MEM_ALLOC * BITS_PER_UINT32)];

	bus_addr_t		sc_io_base;
	bus_addr_t		sc_io_size;
	uint32_t		sc_io_map[(16*1024*1024)/
				(PCI_MIN_IO_ALLOC * BITS_PER_UINT32)];

        struct intr_event       *sc_eventstab[MT762X_PCI_NIRQS];
        mips_intrcnt_t          sc_intr_counter[MT762X_PCI_NIRQS];
        struct resource         *sc_irq;
        void                    *sc_ih;

	int			pcie_link_status;
};

static int pci_irqs[MT762X_PCI_NIRQS] = {
	MT762X_PCIE0_IRQ,
	MT762X_PCIE1_IRQ,
	MT762X_PCIE2_IRQ
};

static int mt762x_pci_setup_intr(device_t, device_t, struct resource *, int,
                    driver_filter_t *, driver_intr_t *, void *, void **);
static int mt762x_pci_teardown_intr(device_t, device_t, struct resource *,
                    void *);
static int mt762x_pci_intr(void *);

static inline void
write_config(uint32_t bus, uint32_t dev, uint32_t func, uint32_t reg,
	uint32_t *val)
{
	uint32_t address_reg, data_reg, address;

	address_reg = MT762X_PCI_CONF_REG;
	data_reg = MT762X_PCI_DATA_REG;

	address = (((reg & 0xf00) >> 8) << 24) | (bus << 16) | (dev << 11) |
		   (func << 8) | (reg & 0xfc) | 0x80000000;

	MT_WRITE32(address_reg, &address);
	MT_WRITE32(data_reg, val);
}

static inline void
read_config(uint32_t bus, uint32_t dev, uint32_t func, uint32_t reg,
	uint32_t *val)
{
	uint32_t address_reg, data_reg, address;

	address_reg = MT762X_PCI_CONF_REG;
	data_reg = MT762X_PCI_DATA_REG;

	address = (((reg & 0xf00) >> 8) << 24) | (bus << 16) | (dev << 11) |
		   (func << 8) | (reg & 0xfc) | 0x80000000;

	MT_WRITE32(address_reg, &address);
	MT_READ32(data_reg, val);
}

static void
mt762x_map_pci_irqs(void)
{
	int i;

	for (i = 0; i < MT762X_PCI_NIRQS; i++)
		gic_irq_map_to_pin(pci_irqs[i], PCI_INTR_PIN);
}

static void
mt762x_pci_mask_irq(void *source)
{

	gic_irq_mask((int)source);
}

static void
mt762x_pci_unmask_irq(void *source)
{

	gic_irq_unmask((int)source);
}

static void
mt762x_clear_pci_irqs(void)
{
	int i;

	for (i = 0; i < MT762X_PCI_NIRQS; i++)
		gic_irq_ack(pci_irqs[i]);
}

static void
mt762x_mask_pci_irqs(void)
{
	int i;

	for (i = 0; i < MT762X_PCI_NIRQS; i++)
		gic_irq_mask(pci_irqs[i]);
}

static inline int
mt762x_irq_to_idx(int irq)
{

	return ((irq == MT762X_PCIE0_IRQ) ? 0 :
		(irq == MT762X_PCIE1_IRQ) ? 1 :
		(irq == MT762X_PCIE2_IRQ) ? 2 : -1);
}

#ifdef notyet
static int
mt762x_pci_check_bus_error(void)
{

	NOT_IMPL();

	return (0);
}
#endif

static inline uint32_t
mt762x_pci_make_addr(int bus, int slot, int func, int reg)
{
	uint32_t addr;

	addr = (((reg & 0xf00) >> 8) << 24) | (bus << 16) | (slot << 11) |
		(func << 8) | (reg & 0xfc) | (1 << 31);

	return (addr);
}

#ifdef notyet
static int
mt762x_pci_conf_setup(int bus, int slot, int func, int reg, int bytes,
    uint32_t cmd)
{
        uint32_t addr = mt762x_pci_make_addr(bus, slot, func, (reg & ~3));

        mtx_assert(&mt762x_pci_mtx, MA_OWNED);

        MT_WRITE_REG(MT762X_PCI_CONF_ADDR, addr);
        MT_WRITE_REG(MT762X_PCI_CONF_CMD + (btr & 0x3), cmd);

        dprintf("%s: tag (%x, %x, %x) %d/%d addr=%08x, cmd=%08x\n", __func__,
            bus, slot, func, reg, bytes, addr, cmd);

        return mt762x_pci_check_bus_error();
}
#endif

static uint32_t
mt762x_pci_read_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, int bytes)
{
	uint32_t addr = 0, data = 0;

	mtx_lock_spin(&mt762x_pci_mtx);
	addr = mt762x_pci_make_addr(bus, slot, func, (reg & ~3));
	MT_WRITE32(MT762X_PCI_CONF_REG, &addr);
	switch (bytes % 4) {
	case 0:
		MT_READ32(MT762X_PCI_DATA_REG, &data);
		break;
	case 1:
		MT_READ8(MT762X_PCI_DATA_REG + (reg & 0x3), &data);
		break;
	case 2:
		MT_READ16(MT762X_PCI_DATA_REG + (reg & 0x3), &data);
		break;
	default:
		panic("%s(): Wrong number of bytes (%d) requested!\n",
			__FUNCTION__, bytes % 4);
	}
	mtx_unlock_spin(&mt762x_pci_mtx);

	return (data);
}

static void
mt762x_pci_write_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, uint32_t val, int bytes)
{
	uint32_t addr = 0, data = val;

	mtx_lock_spin(&mt762x_pci_mtx);
	addr = mt762x_pci_make_addr(bus, slot, func, (reg & ~3));
	MT_WRITE32(MT762X_PCI_CONF_REG, &addr);
	switch (bytes % 4) {
	case 0:
		MT_WRITE32(MT762X_PCI_DATA_REG, &data);
		break;
	case 1:
		MT_WRITE8(MT762X_PCI_DATA_REG + (reg & 0x3), &data);
		break;
	case 2:
		MT_WRITE16(MT762X_PCI_DATA_REG + (reg & 0x3), &data);
		break;
	default:
		panic("%s(): Wrong number of bytes (%d) requested!\n",
			__FUNCTION__, bytes % 4);
	}
	mtx_unlock_spin(&mt762x_pci_mtx);
}

static inline void
mt762x_set_pcie_phy(uint32_t addr, int start_b, int bits, int val)
{

	_REG32(addr) &= ~(((1<< bits) - 1) << start_b);
	_REG32(addr) |= val << start_b;
}

static void
mt762x_pci_set_phy_for_ssc(uint32_t offset)
{
	uint32_t reg = (_REG32(RALINK_SYSTEM_CONTROL_BASE + 0x10) >> 6) & 0x7;

	mt762x_set_pcie_phy(offset + 0x400, 8, 1, 1);
	mt762x_set_pcie_phy(offset + 0x400, 9, 2, 0);
	mt762x_set_pcie_phy(offset + 0x000, 4, 1, 1);
	mt762x_set_pcie_phy(offset + 0x100, 4, 1, 1);
	mt762x_set_pcie_phy(offset + 0x000, 5, 1, 0);
	mt762x_set_pcie_phy(offset + 0x100, 5, 1, 0);
	if (reg <= 5 && reg >= 3) {
		mt762x_set_pcie_phy(offset + 0x490, 6, 2, 1);
#ifdef PCIE_PHY_SSC
		mt762x_set_pcie_phy(offset + 0x4a8,  0, 12, 0x1a);
		mt762x_set_pcie_phy(offset + 0x4a8, 16, 12, 0x1a);
#endif
	} else {
		mt762x_set_pcie_phy(offset + 0x490, 6, 2, 0);
		if (reg >= 6) {
			mt762x_set_pcie_phy(offset + 0x4bc,  4,  2, 0x1);
			mt762x_set_pcie_phy(offset + 0x49c,  0, 31, 0x18000000);
			mt762x_set_pcie_phy(offset + 0x4a4,  0, 16, 0x18d);
			mt762x_set_pcie_phy(offset + 0x4a8,  0, 12, 0x4a);
			mt762x_set_pcie_phy(offset + 0x4a8, 16, 12, 0x4a);
#ifdef PCIE_PHY_SSC
			mt762x_set_pcie_phy(offset + 0x4a8,  0, 12, 0x11);
			mt762x_set_pcie_phy(offset + 0x4a8, 16, 12, 0x11);
#endif
		} else {
#ifdef PCIE_PHY_SSC
			mt762x_set_pcie_phy(offset + 0x4a8,  0, 12, 0x1a);
			mt762x_set_pcie_phy(offset + 0x4a8, 16, 12, 0x1a);
#endif
		}
	}
	mt762x_set_pcie_phy(offset + 0x4a0,  5, 1, 1);
	mt762x_set_pcie_phy(offset + 0x490, 22, 2, 2);
	mt762x_set_pcie_phy(offset + 0x490, 18, 4, 6);
	mt762x_set_pcie_phy(offset + 0x490, 12, 4, 2);
	mt762x_set_pcie_phy(offset + 0x490,  8, 4, 1);
	mt762x_set_pcie_phy(offset + 0x4ac, 16, 3, 0);
	mt762x_set_pcie_phy(offset + 0x490,  1, 3, 2);

	if (reg <= 5 && reg >= 3) {
		mt762x_set_pcie_phy(offset + 0x414, 6, 2, 1);
		mt762x_set_pcie_phy(offset + 0x414, 5, 1, 1);
	}

#ifdef PCIE_PHY_SSC
	mt762x_set_pcie_phy(offset + 0x414, 28, 2, 1);
#else
	mt762x_set_pcie_phy(offset + 0x414, 28, 2, 0);
#endif
	mt762x_set_pcie_phy(offset + 0x040, 17, 4, 7);
	mt762x_set_pcie_phy(offset + 0x040, 16, 1, 1);
	mt762x_set_pcie_phy(offset + 0x140, 17, 4, 7);
	mt762x_set_pcie_phy(offset + 0x140, 16, 1, 1);

	mt762x_set_pcie_phy(offset + 0x000,  5, 1, 1);
	mt762x_set_pcie_phy(offset + 0x100,  5, 1, 1);
	mt762x_set_pcie_phy(offset + 0x000,  4, 1, 0);
	mt762x_set_pcie_phy(offset + 0x100,  4, 1, 0);
}

static void
mt762x_pci_bypass_pipe_rst(void)
{

	mt762x_set_pcie_phy(RALINK_PCIEPHY_P0P1_CTL_OFFSET + 0x02c, 12, 1, 1);
	mt762x_set_pcie_phy(RALINK_PCIEPHY_P0P1_CTL_OFFSET + 0x02c,  4, 1, 1);
	mt762x_set_pcie_phy(RALINK_PCIEPHY_P0P1_CTL_OFFSET + 0x12c, 12, 1, 1);
        mt762x_set_pcie_phy(RALINK_PCIEPHY_P0P1_CTL_OFFSET + 0x12c,  4, 1, 1);
	mt762x_set_pcie_phy(RALINK_PCIEPHY_P2_CTL_OFFSET   + 0x02c, 12, 1, 1);
        mt762x_set_pcie_phy(RALINK_PCIEPHY_P2_CTL_OFFSET   + 0x02c,  4, 1, 1);
}

static void
mt762x_pci_setup_port(int port)
{
	uint32_t val;

	read_config(0, port, 0, 0x4, &val);
	val |= 0x4;
	write_config(0, port, 0, 0x4, &val);
	read_config(0, port, 0, 0x70c, &val);
	val &= ~(0xff) << 8;
	val |= 0x50 << 8;
	write_config(0, port, 0, 0x70c, &val);
	read_config(0, port, 0, 0x70c, &val);
	printf("Port %d N_FTS = 0x%08x\n", port, val);

	//PCI_CACHE_LINE_SIZE
	//pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, (L1_CACHE_BYTES >> 2));
	mt762x_pci_write_config(NULL, 0, port, 0, PCIR_CACHELNSZ, 16, 1);
}

static void
mt762x_pci_init(device_t dev)
{
	struct mt762x_pci_softc *sc = device_get_softc(dev);
	uint32_t val;

	sc->pcie_link_status = 0;

	val = RALINK_PCIE0_RST | RALINK_PCIE1_RST | RALINK_PCIE2_RST;
	ASSERT_SYSRST_PCIE(val);
	DELAY(100);

#if defined (GPIO_PERST)
	val = RALINK_GPIOMODE;
	val &= ~((0x3 << PCIE_SHARE_PIN_SW) | (0x3 << UARTL3_SHARE_PIN_SW));
	val |= ((0x1 << PCIE_SHARE_PIN_SW) | (0x1 << UARTL3_SHARE_PIN_SW));
	RALINK_GPIOMODE = val;
	val = (0x1 << GPIO_PCIE_PORT0) | (0x1 << GPIO_PCIE_PORT1) |
		(0x1 << GPIO_PCIE_PORT2);
	DELAY(50000);
	RALINK_GPIO_CTRL0 |= val;
	DELAY(50000);
	RALINK_GPIO_DATA0 &= ~val;
#else
	RALINK_GPIOMODE &= ~(0x3 << PCIE_SHARE_PIN_SW);
#endif

	DELAY(100000);

	val = RALINK_PCIE0_RST | RALINK_PCIE1_RST | RALINK_PCIE2_RST;
	DEASSERT_SYSRST_PCIE(val);

	val = RALINK_CLKCFG1;
	val |= RALINK_PCIE0_CLK_EN | RALINK_PCIE1_CLK_EN | RALINK_PCIE2_CLK_EN;
	RALINK_CLKCFG1 = val;
	DELAY(100000);

	if ((_REG32(0xbe00000c) & 0xffff) == 0x0101)
		mt762x_pci_bypass_pipe_rst();
	mt762x_pci_set_phy_for_ssc(RALINK_PCIEPHY_P0P1_CTL_OFFSET);
	mt762x_pci_set_phy_for_ssc(RALINK_PCIEPHY_P2_CTL_OFFSET);

#if defined(GPIO_PERST)
	val = (1 << GPIO_PCIE_PORT0) | (1 << GPIO_PCIE_PORT1) |
		(1 << GPIO_PCIE_PORT2);
	RALINK_GPIO_DATA0 |= val;
#else
	RALINK_PCI_PCICFG_ADDR &= ~(1 << 1);
#endif

	DELAY(500000);

	if ((RALINK_PCI0_STATUS & 1) == 0) {
		printf("PCIe%d no card detected\n", 0);
	} else {
		sc->pcie_link_status |= (1 << 0);
		printf("PCIe%d card detected\n", 0);
	}

	if ((RALINK_PCI1_STATUS & 1) == 0) {
		printf("PCIe%d no card detected\n", 1);
	} else {
		sc->pcie_link_status |= (1 << 1);
		printf("PCIe%d card detected\n", 1);
	}

	if ((RALINK_PCI2_STATUS & 1) == 0) {
		printf("PCIe%d no card detected\n", 2);
	} else {
		sc->pcie_link_status |= (1 << 2);
		printf("PCIe%d card detected\n", 2);
	}

	if (sc->pcie_link_status == 0)
		return;

#if 0
	switch (sc->pcie_link_status) {
	case 0:
		return;

	case 2:
		/* PCIe1 only */
		RALINK_PCI_PCICFG_ADDR &= ~0x00ff0000;
		RALINK_PCI_PCICFG_ADDR |= (1 << 16) | (0 << 20);
		break;

	case 4:
		/* PCIe2 only */
		RALINK_PCI_PCICFG_ADDR &= ~0x0fff0000;
		RALINK_PCI_PCICFG_ADDR |= (1 << 16) | (2 << 20) | (0 << 24);
		break;

	case 5:
		/* PCIe0 + PCIe2 */
		RALINK_PCI_PCICFG_ADDR &= ~0x0fff0000;
		RALINK_PCI_PCICFG_ADDR |= (0 << 16) | (2 << 20) | (1 << 24);
		break;

	case 6:
		/* PCIe1 + PCIe2 */
		RALINK_PCI_PCICFG_ADDR &= ~0x0fff0000;
		RALINK_PCI_PCICFG_ADDR |= (2 << 16) | (0 << 20) | (1 << 24);
		break;
	}
#endif

	RALINK_PCI_MEMBASE = 0;//0x60000000;
	RALINK_PCI_IOBASE = 0x1e160000;

	RALINK_PCI0_BAR0SETUP_ADDR = 0x7FFF0000;
	RALINK_PCI0_BAR1SETUP_ADDR = 0;
	RALINK_PCI0_IMBASEBAR0_ADDR = 0;
	RALINK_PCI0_CLASS = 0x06040001;
	RALINK_PCI0_BAR0SETUP_ADDR = 0x7FFF0001;
	RALINK_PCI_PCIMSK_ADDR |= (1 << 20);

	RALINK_PCI1_BAR0SETUP_ADDR = 0x7FFF0000;
	RALINK_PCI1_BAR1SETUP_ADDR = 0;
	RALINK_PCI1_IMBASEBAR0_ADDR = 0;
	RALINK_PCI1_CLASS = 0x06040001;
	RALINK_PCI1_BAR0SETUP_ADDR = 0x7FFF0001;
	RALINK_PCI_PCIMSK_ADDR |= (1 << 21);

	RALINK_PCI2_BAR0SETUP_ADDR = 0x7FFF0000;
	RALINK_PCI2_BAR1SETUP_ADDR = 0;
	RALINK_PCI2_IMBASEBAR0_ADDR = 0;
	RALINK_PCI2_CLASS = 0x06040001;
	RALINK_PCI2_BAR0SETUP_ADDR = 0x7FFF0001;
	RALINK_PCI_PCIMSK_ADDR |= (1 << 22);

	mt762x_pci_setup_port(0);
	mt762x_pci_setup_port(1);
	mt762x_pci_setup_port(2);
}

static int
mt762x_pci_probe(device_t dev)
{

        return (BUS_PROBE_NOWILDCARD);
}

static int mt762x_pci_route_interrupt(device_t pcib, device_t device, int pin);
#define mt_pcib_read_config	mt762x_pci_read_config
#define mt_pcib_write_config	mt762x_pci_write_config

static int mt_pcib_init(struct mt762x_pci_softc *sc, int bus, int maxslot);

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
pcib_alloc(struct mt762x_pci_softc *sc, uint32_t smask)
{
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

	//printf("%s(): requested %d bytes\n", __FUNCTION__, size);

        for (i = 0; i + bits <= bits_limit; i += bits)
                if (pcib_map_check(map, i, bits)) {
                        pcib_map_set(map, i, bits);
                        addr = base + (i * min_alloc);
			//printf("return 0x%08x\n", addr);
                        return (addr);
                }

	//printf("return 0x%08x\n", addr);
        return (addr);
}

static int
mt_pcib_init_bar(struct mt762x_pci_softc *sc, int bus, int slot, int func,
	int barno)
{
	uint32_t addr, bar;
	int reg, width;

	reg = PCIR_BAR(barno);

	mt_pcib_write_config(sc->sc_dev, bus, slot, func, reg, ~0, 4);
	bar = mt_pcib_read_config(sc->sc_dev, bus, slot, func, reg, 4);
        if (bar == 0)
                return (1);

	/* Calculate BAR size: 64 or 32 bit (in 32-bit units) */
        width = ((bar & 7) == 4) ? 2 : 1;

        addr = pcib_alloc(sc, bar);
        if (!addr)
                return (-1);

        if (bootverbose)
                printf("PCI %u:%u:%u: reg %x: smask=%08x: addr=%08x\n",
                    bus, slot, func, reg, bar, addr);

        mt_pcib_write_config(sc->sc_dev, bus, slot, func, reg, addr, 4);
        if (width == 2)
                mt_pcib_write_config(sc->sc_dev, bus, slot, func, reg + 4,
                    0, 4);

        return (width);
}

static int
mt_pcib_init_all_bars(struct mt762x_pci_softc *sc, int bus, int slot, int func,
	int hdrtype)
{
	int maxbar, bar, i;

	maxbar = (hdrtype & PCIM_HDRTYPE) ? 0 : 6;
	bar = 0;

	while (bar < maxbar) {
		i = mt_pcib_init_bar(sc, bus, slot, func, bar);
		bar += i;
		if (i < 0) {
			device_printf(sc->sc_dev,
				"PCI IO/Memory space exhausted\n");
			return (ENOMEM);
		}
	}

	return (0);
}

static int cur_secbus = 0;

static int
slot_has_link(struct mt762x_pci_softc *sc, int slot)
{
	switch (slot) {
	case 0: return  sc->pcie_link_status & 1;
	case 1: return  sc->pcie_link_status & 2;
	case 2: return  sc->pcie_link_status & 4;
	default: return 0;
	}
}

static void
mt_pcib_init_bridge(struct mt762x_pci_softc *sc, int bus, int slot, int func)
{
	bus_addr_t io_base, mem_base;
	uint32_t io_limit, mem_limit;
	int secbus;

	//printf("%s(): bus %d, slot %d, func %d\n", __FUNCTION__,
	//	bus, slot, func);

	if (bus == 0 && !slot_has_link(sc, slot)) {
		printf("Skip bus %d due to no link\n", ++cur_secbus);
		return;
	}

	io_base = sc->sc_io_base;
	io_limit = io_base + sc->sc_io_size - 1;
	mem_base = sc->sc_mem_base;
	mem_limit = mem_base + sc->sc_mem_size - 1;

	mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_IOBASEL_1,
		io_base >> 8, 1);
	mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_IOBASEH_1,
            io_base >> 16, 2);
        mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_IOLIMITL_1,
            io_limit >> 8, 1);
        mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_IOLIMITH_1,
            io_limit >> 16, 2);

#if 1
	mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_MEMBASE_1,
            mem_base >> 16, 2);
        mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_MEMLIMIT_1,
            mem_limit >> 16, 2);
#endif

	mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_PMBASEL_1,
            0x10, 2);
        mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_PMBASEH_1,
            0x0, 4);
        mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_PMLIMITL_1,
            0xF, 2);
        mt_pcib_write_config(sc->sc_dev, bus, slot, func, PCIR_PMLIMITH_1,
            0x0, 4);

	secbus = mt_pcib_read_config(sc->sc_dev, bus, slot, func,
            PCIR_SECBUS_1, 1);

	if (secbus == 0) {
		//printf("SECBUS = 0?!\n");
		mt_pcib_write_config(sc->sc_dev, bus, slot, func,
			PCIR_SECBUS_1, ++cur_secbus, 1);
		secbus = cur_secbus;
		mt_pcib_write_config(sc->sc_dev, bus, slot, func,
			PCIR_SUBBUS_1, cur_secbus, 1);
	}

        /* Configure buses behind the bridge */
        mt_pcib_init(sc, secbus, PCI_SLOTMAX);
}

static int
mt_pcib_init(struct mt762x_pci_softc *sc, int bus, int maxslot)
{
	int slot, func, maxfunc, error;
	uint8_t hdrtype, command, class, subclass;

	//printf("%s(): bus %d, maxslot %d\n", __FUNCTION__, bus, maxslot);

	for (slot = 0; slot <= maxslot; slot++) {
		maxfunc = 0;
		for (func = 0; func <= maxfunc; func++) {
			hdrtype = mt_pcib_read_config(sc->sc_dev, bus, slot,
                            func, PCIR_HDRTYPE, 1);

			//printf("%s(): bus %d, slot %d, func %d, hdrtype %d\n",
			//	__FUNCTION__, bus, slot, func, hdrtype);

			if ((hdrtype & PCIM_HDRTYPE) > PCI_MAXHDRTYPE)
				continue;

			if (func == 0  && (hdrtype & PCIM_MFDEV))
				maxfunc = PCI_FUNCMAX;

			command = mt_pcib_read_config(sc->sc_dev, bus, slot,
				func, PCIR_COMMAND, 1);
			command &= ~(PCIM_CMD_MEMEN | PCIM_CMD_PORTEN);
			mt_pcib_write_config(sc->sc_dev, bus, slot, func,
				PCIR_COMMAND, command, 1);

			error = mt_pcib_init_all_bars(sc, bus, slot, func,
				hdrtype);

			if (error)
				return (error);

			command |= PCIM_CMD_BUSMASTEREN | PCIM_CMD_MEMEN |
				PCIM_CMD_PORTEN;
			mt_pcib_write_config(sc->sc_dev, bus, slot, func,
				PCIR_COMMAND, command, 1);

			mt_pcib_write_config(sc->sc_dev, bus, slot, func,
				PCIR_CACHELNSZ, 16, 1);

			class = mt_pcib_read_config(sc->sc_dev, bus, slot,
				func, PCIR_CLASS, 1);
			subclass = mt_pcib_read_config(sc->sc_dev, bus, slot,
				func, PCIR_SUBCLASS, 1);

			if (1 || class != PCIC_BRIDGE ||
				subclass != PCIS_BRIDGE_PCI)
				continue;

			mt_pcib_init_bridge(sc, bus, slot, func);
		}	
	}

//	pcib_write_irq_mask(sc, (0xf << 24));

	return (0);
}


static int
mt762x_pci_attach(device_t dev)
{
        //int rid = 0;
        struct mt762x_pci_softc *sc = device_get_softc(dev);

	sc->sc_dev = dev;
	sc->sc_mem_base = 0x60000000;
	sc->sc_mem_size = 0x08000000;
	sc->sc_io_base = 0x1e160000;
	sc->sc_io_size = 0x10000;

	//sc->sc_mem_base = (bus_addr_t)pmap_mapdev(sc->sc_mem_base, sc->sc_mem_size);
        sc->sc_mem_rman.rm_type = RMAN_ARRAY;
        sc->sc_mem_rman.rm_descr = "mt762x PCI memory window";
        if (rman_init(&sc->sc_mem_rman) != 0 ||
            rman_manage_region(&sc->sc_mem_rman, sc->sc_mem_base,
                sc->sc_mem_base + sc->sc_mem_size - 1) != 0) {
                panic("mt762x_pci_attach: failed to set up I/O rman");
        }

        sc->sc_irq_rman.rm_type = RMAN_ARRAY;
        sc->sc_irq_rman.rm_descr = "mt762x PCI IRQs";
        if (rman_init(&sc->sc_irq_rman) != 0 ||
            rman_manage_region(&sc->sc_irq_rman, MT762X_PCIE0_IRQ,
                MT762X_PCIE0_IRQ) != 0 ||
	    rman_manage_region(&sc->sc_irq_rman, MT762X_PCIE1_IRQ,
		MT762X_PCIE2_IRQ) != 0)
                panic("mt762x_pci_attach: failed to set up IRQ rman");

#ifdef notyet
        /*
         * Check if there is a base slot hint. Otherwise use default value.
         */
        if (resource_int_value(device_get_name(dev),
            device_get_unit(dev), "baseslot", &sc->sc_baseslot) != 0) {
                device_printf(dev,
                    "%s: missing hint '%s', default to MT762X_PCI_BASE_SLOT\n",
                    __func__, "baseslot");
                sc->sc_baseslot = MT762X_PCI_BASE_SLOT;
        }
#else
	sc->sc_baseslot = MT762X_PCI_BASE_SLOT;
#endif

	mt762x_map_pci_irqs();
	mt762x_clear_pci_irqs();
	mt762x_mask_pci_irqs();

        /* Hook up our interrupt handler. */
	cpu_establish_hardintr("pci", mt762x_pci_intr, NULL, sc,
		PCI_INTR_PIN, INTR_TYPE_MISC | INTR_EXCL, NULL);
#if 0
        if ((sc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
            RF_SHAREABLE | RF_ACTIVE)) == NULL) {
                device_printf(dev, "unable to allocate IRQ resource\n");
                return ENXIO;
        }

        if ((bus_setup_intr(dev, sc->sc_irq, INTR_TYPE_MISC,
                            mt762x_pci_intr, NULL, sc, &sc->sc_ih))) {
                device_printf(dev,
                    "WARNING: unable to register interrupt handler\n");
                return ENXIO;
        }
#endif

#ifdef notyet
        /* reset PCI core and PCI bus */
        mt762x_device_stop(RST_RESET_PCI_CORE | RST_RESET_PCI_BUS);
        DELAY(100000);

        mt762x_device_start(RST_RESET_PCI_CORE | RST_RESET_PCI_BUS);
        DELAY(100000);
#endif

#ifdef notyet
        /* Init PCI windows */
        MT_WRITE_REG(MT762X_PCI_WINDOW0, PCI_WINDOW0_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW1, PCI_WINDOW1_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW2, PCI_WINDOW2_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW3, PCI_WINDOW3_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW4, PCI_WINDOW4_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW5, PCI_WINDOW5_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW6, PCI_WINDOW6_ADDR);
        MT_WRITE_REG(MT762X_PCI_WINDOW7, PCI_WINDOW7_CONF_ADDR);
        DELAY(100000);
#endif

        mtx_lock_spin(&mt762x_pci_mtx);
//        mt762x_pci_check_bus_error();
        mtx_unlock_spin(&mt762x_pci_mtx);

	mt762x_pci_init(dev);
//	mt762x_pci_route_interrupt(dev, NULL, 0);
//	mt762x_pci_route_interrupt(dev, NULL, 0);
//	mt762x_pci_route_interrupt(dev, NULL, 0);

#ifdef notyet
        /* Fixup internal PCI bridge */
        mt762x_pci_local_write(dev, PCIR_COMMAND,
            PCIM_CMD_BUSMASTEREN | PCIM_CMD_MEMEN
            | PCIM_CMD_SERRESPEN | PCIM_CMD_BACKTOBACK
            | PCIM_CMD_PERRESPEN | PCIM_CMD_MWRICEN, 4);
#endif

	mt_pcib_init(sc, 0, PCI_SLOTMAX);
	mt_pcib_init_bridge(sc, 0, 0, 0);
	mt_pcib_init_bridge(sc, 0, 1, 0);
	mt_pcib_init_bridge(sc, 0, 2, 0);

        device_add_child(dev, "pci", -1);
        return (bus_generic_attach(dev));
}

static int
mt762x_pci_read_ivar(device_t dev, device_t child, int which,
    uintptr_t *result)
{
        struct mt762x_pci_softc *sc = device_get_softc(dev);

	//printf("%s(): entered\n", __FUNCTION__);

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
mt762x_pci_write_ivar(device_t dev, device_t child, int which,
    uintptr_t result)
{
        struct mt762x_pci_softc * sc = device_get_softc(dev);

	//printf("%s(): entered\n", __FUNCTION__);

        switch (which) {
        case PCIB_IVAR_BUS:
                sc->sc_busno = result;
                return (0);
        }

        return (ENOENT);
}

static struct resource *
mt762x_pci_alloc_resource(device_t bus, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{

        struct mt762x_pci_softc *sc = device_get_softc(bus);
        struct resource *rv;
        struct rman *rm;
	vm_offset_t va;

	//printf("%s(): ", __FUNCTION__);

        switch (type) {
        case SYS_RES_IRQ:
		//printf("irq ");
                rm = &sc->sc_irq_rman;
                break;
        case SYS_RES_MEMORY:
		//printf("mem ");
                rm = &sc->sc_mem_rman;
                break;
        default:
	//	printf("0x%08x ", type);
                return (NULL);
        }

        rv = rman_reserve_resource(rm, start, end, count, flags, child);

	//printf("%s(): start 0x%08x, end 0x%08x, count %d, %s\n", __FUNCTION__,
	//	(uint32_t)start, (uint32_t)end, (uint32_t)count,
	//	type == SYS_RES_MEMORY ? "mem" : "irq");

        if (rv == NULL) {
		//printf("NULL\n");
                return (NULL);
	}

	rman_set_rid(rv, *rid);

	if (type == SYS_RES_MEMORY) {
		va = (vm_offset_t)pmap_mapdev(start, count);
		rman_set_bushandle(rv, va);
		rman_set_virtual(rv, (void *)va);
		rman_set_bustag(rv, mips_bus_space_generic);
		printf("%s(): mapped to 0x%08x\n", __FUNCTION__, va);
	} else {
		rman_set_bustag(rv, mips_bus_space_generic);
		rv->r_bushandle = start;
	}

	//printf("%s(): ... \n", __FUNCTION__);

        if (flags & RF_ACTIVE) {
		//printf("activate ");
                if (bus_activate_resource(child, type, *rid, rv)) {
                        rman_release_resource(rv);
			//printf("NULL\n");
                        return (NULL);
                }
        }
	//printf("return\n");
        return (rv);
}

#if 1
static int
mt762x_pci_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
#if 1
	return (rman_activate_resource(r));
#else
        int res = (BUS_ACTIVATE_RESOURCE(device_get_parent(bus),
            child, type, rid, r));

        if (!res) {
                switch(type) {
                case SYS_RES_MEMORY:
                case SYS_RES_IOPORT:
                        //rman_set_bustag(r, mt762x_bus_space_pcimem);
                        break;
                }
        }
        return (res);
#endif
}
#endif

static int
mt762x_pci_setup_intr(device_t bus, device_t child, struct resource *ires,
            int flags, driver_filter_t *filt, driver_intr_t *handler,
            void *arg, void **cookiep)
{
        struct mt762x_pci_softc *sc = device_get_softc(bus);
        struct intr_event *event;
        int irq, error, irqidx;

        irq = rman_get_start(ires);

	if ((irqidx = mt762x_irq_to_idx(irq)) == -1)
                panic("%s: bad irq %d", __func__, irq);

        event = sc->sc_eventstab[irqidx];
        if (event == NULL) {
                error = intr_event_create(&event, (void *)irq, 0, irq,
                    mt762x_pci_mask_irq, mt762x_pci_unmask_irq, NULL, NULL,
                    "pci intr%d:", irq);

                if (error == 0) {
                        sc->sc_eventstab[irqidx] = event;
                        sc->sc_intr_counter[irqidx] =
                            mips_intrcnt_create(event->ie_name);
                }
                else
                        return (error);
        }

        intr_event_add_handler(event, device_get_nameunit(child), filt,
            handler, arg, intr_priority(flags), flags, cookiep);
        mips_intrcnt_setname(sc->sc_intr_counter[irqidx], event->ie_fullname);

        mt762x_pci_unmask_irq((void*)irq);

        return (0);
}

static int
mt762x_pci_teardown_intr(device_t dev, device_t child, struct resource *ires,
    void *cookie)
{
        struct mt762x_pci_softc *sc = device_get_softc(dev);
        int irq, result, irqidx;

        irq = rman_get_start(ires);
	if ((irqidx = mt762x_irq_to_idx(irq)) == -1)
                panic("%s: bad irq %d", __func__, irq);

        if (sc->sc_eventstab[irqidx] == NULL)
                panic("Trying to teardown unoccupied IRQ");

        mt762x_pci_mask_irq((void*)irq);

        result = intr_event_remove_handler(cookie);
        if (!result)
                sc->sc_eventstab[irqidx] = NULL;

        return (result);
}

static int
mt762x_pci_intr(void *arg)
{
        struct mt762x_pci_softc *sc = arg;
        struct intr_event *event;
        uint32_t reg, irq, irqidx;

        /*
         * Handle only unmasked interrupts
         */
        reg = gic_irq_get();
        for (irqidx = 0; irqidx < MT762X_PCI_NIRQS; irqidx++) {
		irq = pci_irqs[irqidx];
		//printf("%s(): IRQ %d\n", __FUNCTION__, irq);
                if (reg & (1 << irq)) {
			//printf("%s(): irq %d\n", __FUNCTION__, irq);
                        event = sc->sc_eventstab[irqidx];
                        if (!event || TAILQ_EMPTY(&event->ie_handlers)) {
                                /* Ignore timer interrupts */
                                if (irq != 0)
                                        printf("Stray IRQ %d\n", irq);
                                continue;
                        }

                        /* Flush DDR FIFO for PCI/PCIe */
                        //mt762x_device_flush_ddr(MT762X_CPU_DDR_FLUSH_PCIE);

                        /* TODO: frame instead of NULL? */
                        intr_event_handle(event, NULL);
                        mips_intrcnt_inc(sc->sc_intr_counter[irqidx]);
                }
        }

        return (FILTER_HANDLED);
}

static int
mt762x_pci_maxslots(device_t dev)
{

        return (PCI_SLOTMAX);
}

static int
mt762x_pci_route_interrupt(device_t pcib, device_t device, int pin)
{
	//struct mt762x_pci_softc *sc = device_get_softc(pcib);
	int bus, sl;

	bus = pci_get_bus(device);
	sl = pci_get_slot(device);

	//printf("%s(): bus %d, slot %d\n", __FUNCTION__, bus, sl);

	if (bus != 0) panic("Unexpected bus number %d\n", bus);
	switch (sl) {
	case 0: return MT762X_PCIE0_IRQ;
	case 1: return MT762X_PCIE1_IRQ;
	case 2: return MT762X_PCIE2_IRQ;
	}

        return (-1);
}

static device_method_t mt762x_pci_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,         mt762x_pci_probe),
        DEVMETHOD(device_attach,        mt762x_pci_attach),
        DEVMETHOD(device_shutdown,      bus_generic_shutdown),
        DEVMETHOD(device_suspend,       bus_generic_suspend),
        DEVMETHOD(device_resume,        bus_generic_resume),

        /* Bus interface */
        DEVMETHOD(bus_read_ivar,        mt762x_pci_read_ivar),
        DEVMETHOD(bus_write_ivar,       mt762x_pci_write_ivar),
        DEVMETHOD(bus_alloc_resource,   mt762x_pci_alloc_resource),
        DEVMETHOD(bus_release_resource, bus_generic_release_resource),
        DEVMETHOD(bus_activate_resource, mt762x_pci_activate_resource),
        DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
        DEVMETHOD(bus_setup_intr,       mt762x_pci_setup_intr),
        DEVMETHOD(bus_teardown_intr,    mt762x_pci_teardown_intr),

        /* pcib interface */
        DEVMETHOD(pcib_maxslots,        mt762x_pci_maxslots),
        DEVMETHOD(pcib_read_config,     mt762x_pci_read_config),
        DEVMETHOD(pcib_write_config,    mt762x_pci_write_config),
        DEVMETHOD(pcib_route_interrupt, mt762x_pci_route_interrupt),

        DEVMETHOD_END
};

static driver_t mt762x_pci_driver = {
        "pcib",
        mt762x_pci_methods,
        sizeof(struct mt762x_pci_softc),
};

static devclass_t mt762x_pci_devclass;

DRIVER_MODULE(mt762x_pci, nexus, mt762x_pci_driver, mt762x_pci_devclass, 0, 0);
DRIVER_MODULE(mt762x_pci, obio, mt762x_pci_driver, mt762x_pci_devclass, 0, 0);
