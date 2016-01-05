/*-
 * Copyright (c) 2015 Stanislav Galabov.
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
 *
 * $FreeBSD$
 */
#ifndef __MTK_PCIREG_H__
#define __MTK_PCIREG_H__

#define PCI_MIN_IO_ALLOC		4
#define PCI_MIN_MEM_ALLOC		16
#define BITS_PER_UINT32			(NBBY * sizeof(uint32_t))

#define MTK_PCI_NIRQS			1
#define MTK_PCI_BASESLOT		0

struct mtk_pci_softc {
	device_t		sc_dev;

	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;

	int			sc_busno;

	struct rman		sc_mem_rman;
	struct rman		sc_io_rman;
	struct rman		sc_irq_rman;

	bus_addr_t		sc_mem_base;
	bus_addr_t		sc_mem_size;
	uint32_t		sc_mem_map[(256*1024*1024) /
				(PCI_MIN_MEM_ALLOC * BITS_PER_UINT32)];

	bus_addr_t		sc_io_base;
	bus_addr_t		sc_io_size;
	uint32_t		sc_io_map[(16*1024*1024) /
				(PCI_MIN_IO_ALLOC * BITS_PER_UINT32)];

	struct intr_event	*sc_eventstab[MTK_PCI_NIRQS];
	mips_intrcnt_t		sc_intr_counter[MTK_PCI_NIRQS];

	int			pcie_link_status;
};

#define MTK_PCI_PCICFG			0x0000
#define MTK_PCI_PCIINT			0x0008
#define MTK_PCI_PCIENA			0x000C
#define MTK_PCI_CFGADDR			0x0020
#define MTK_PCI_CFGDATA			0x0024
#define MTK_PCI_MEMBASE			0x0028
#define MTK_PCI_IOBASE			0x002C
#define MTK_PCI_PHY0_CFG		0x0090

#define MTK_PCI_PCIE0_BAR0SETUP		0x2010
#define MTK_PCI_PCIE0_BAR1SETUP		0x2014
#define MTK_PCI_PCIE0_IMBASEBAR0	0x2018
#define MTK_PCI_PCIE0_ID		0x2030
#define MTK_PCI_PCIE0_CLASS		0x2034
#define MTK_PCI_PCIE0_SUBID		0x2038
#define MTK_PCI_PCIE0_STATUS		0x2050
#define MTK_PCI_PCIE0_DLECR		0x2060
#define MTK_PCI_PCIE0_ECRC		0x2064

#define MTK_PCIE0_IRQ			20
#define MTK_PCIE1_IRQ			21
#define MTK_PCIE2_IRQ			22

#define MTK_PCI_INTR_PIN		2

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

uint32_t mtk_pci_read_config(device_t, u_int, u_int, u_int, u_int, int);
void mtk_pci_write_config(device_t, u_int, u_int, u_int, u_int, uint32_t, int);

#endif /* __MTK_PCIREG_H__ */
