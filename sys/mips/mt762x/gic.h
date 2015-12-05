/*-
 * Copyright (c) 2015 Stanislav Galabov <sgalabov@gmail.com>
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
#ifndef _GIC_H_
#define _GIC_H_

#define GIC_STATIC_ADDR		0x1fbc0000

#define GIC_CPU_INT0		0
#define GIC_CPU_INT1		1
#define GIC_CPU_INT2		2
#define GIC_CPU_INT3		3
#define GIC_CPU_INT4		4
#define GIC_CPU_INT5		5

#define GIC_INT_TMR		(GIC_CPU_INT5)

typedef volatile __uint32_t gic_reg_t;
extern uint32_t __gic_base, __cm_gcr_base;

#define gic_reg(offset) \
	((gic_reg_t *)(__gic_base + (offset)))

#define cm_reg(offset) \
	((gic_reg_t *)(__cm_gcr_base + (offset)))

#define gic_read(base)		(*(base))
#define gic_write(base, val)	(*(base) = (val))

#define GCR_GIC_BASE	cm_reg(0x0080)

#define GIC_SH_WEDGE	gic_reg(0x0280)

extern void gic_init(void);
extern uint32_t gic_irq_get(void);
extern void gic_irq_ack(int);
extern void gic_irq_mask(int);
extern void gic_irq_unmask(int);
extern void gic_irq_map_to_pin(int, int);

#endif			/* _GIC_H_ */
