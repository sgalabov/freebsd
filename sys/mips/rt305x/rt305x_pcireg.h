#ifndef __RT305X_PCIREG_H__
#define __RT305X_PCIREG_H__

#define RT305X_PCI_NIRQS	1
#define RT305X_PCI_BASESLOT	0

#define RT305X_PCI_PCICFG		0x0000
#define RT305X_PCI_PCIINT		0x0008
#define RT305X_PCI_PCIENA		0x000C
#define RT305X_PCI_CFGADDR		0x0020
#define RT305X_PCI_CFGDATA		0x0024
#define RT305X_PCI_MEMBASE		0x0028
#define RT305X_PCI_IOBASE		0x002C
#define RT305X_PCI_PHY0_CFG		0x0090

#define RT305X_PCI_PCIE0_BAR0SETUP	0x2010
#define RT305X_PCI_PCIE0_BAR1SETUP	0x2014
#define RT305X_PCI_PCIE0_IMBASEBAR0	0x2018
#define RT305X_PCI_PCIE0_ID		0x2030
#define RT305X_PCI_PCIE0_CLASS		0x2034
#define RT305X_PCI_PCIE0_SUBID		0x2038
#define RT305X_PCI_PCIE0_STATUS		0x2050
#define RT305X_PCI_PCIE0_DLECR		0x2060
#define RT305X_PCI_PCIE0_ECRC		0x2064

#define RT305X_PCIE0_IRQ	20
#define RT305X_PCIE1_IRQ	21
#define RT305X_PCIE2_IRQ	22

#define RT305X_PCI_INTR_PIN	2

#define PCI_MIN_IO_ALLOC	4
#define PCI_MIN_MEM_ALLOC	16
#define BITS_PER_UINT32		(NBBY * sizeof(uint32_t))

#define RT_WRITE32(sc, off, val) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (off), (val))
#define RT_WRITE16(sc, off, val) \
	bus_space_write_2((sc)->sc_bst, (sc)->sc_bsh, (off), (val))
#define RT_WRITE8(sc, off, val) \
	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (off), (val))
#define RT_READ32(sc, off) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (off))
#define RT_READ16(sc, off) \
	bus_space_read_2((sc)->sc_bst, (sc)->sc_bsh, (off))
#define RT_READ8(sc, off) \
	bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (off))

#endif /* __RT305X_PCIREG_H__ */
