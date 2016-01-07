/*-
 * Copyright 2016 Stanislav Galabov
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
        
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/resource.h>
#include <sys/gpio.h>
                
#include <machine/bus.h>
#include <machine/intr.h>
                
#include <dev/gpio/gpiobusvar.h>
                
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <gnu/dts/include/dt-bindings/interrupt-controller/irq.h>

#include "gpio_if.h"
#include "pic_if.h"

#define MTK_GPIO_PINS 32

struct mtk_gpio_pin {
	uint32_t pin_caps;
	uint32_t pin_flags;
	enum intr_trigger intr_trigger;
	enum intr_polarity intr_polarity;
	char pin_name[GPIOMAXNAME];
	struct intr_irqsrc *pin_irqsrc;
};

struct mtk_gpio_softc {
	device_t		dev;
	device_t		busdev;
	struct resource		*res[2];
	struct mtx		mtx;
	struct mtk_gpio_pin	pins[MTK_GPIO_PINS];
	void			*intrhand;

	uint32_t		num_pins;
	uint8_t			do_remap;
};

static struct resource_spec mtk_gpio_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ,    0, RF_ACTIVE },
	{ -1, 0 }
};

static int mtk_gpio_probe(device_t dev);
static int mtk_gpio_attach(device_t dev);
static int mtk_gpio_detach(device_t dev);
static int mtk_gpio_intr(void *arg);

#define MTK_GPIO_LOCK(sc)		mtx_lock_spin(&(sc)->mtx)
#define MTK_GPIO_UNLOCK(sc)		mtx_unlock_spin(&(sc)->mtx)
#define MTK_GPIO_LOCK_INIT(sc)		\
    mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev),	\
    "mtk_gpio", MTX_SPIN)
#define MTK_GPIO_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->mtx)

#define MTK_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))
#define MTK_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))

/* Register definitions */
#define GPIO_PIOINT(_sc)		0x0000
#define GPIO_PIOEDGE(_sc)		0x0004
#define GPIO_PIORENA(_sc)		0x0008
#define GPIO_PIOFENA(_sc)		0x000C
#define GPIO_PIODATA(_sc)		((_sc)->do_remap ? 0x0020 : 0x0010)
#define GPIO_PIODIR(_sc)		((_sc)->do_remap ? 0x0024 : 0x0014)
#define GPIO_PIOPOL(_sc)		((_sc)->do_remap ? 0x0028 : 0x0018)
#define GPIO_PIOSET(_sc)		((_sc)->do_remap ? 0x002C : 0x001C)
#define GPIO_PIORESET(_sc)		((_sc)->do_remap ? 0x0030 : 0x0020)
#define GPIO_PIOTOG(_sc)		((_sc)->do_remap ? 0x0034 : 0x0024)

static int
mtk_gpio_probe(device_t dev)
{
	phandle_t node;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "mtk,mtk-gpio"))
		return (ENXIO);

	node = ofw_bus_get_node(dev);
	if (!OF_hasprop(node, "gpio-controller"))
		return (ENXIO);

	device_set_desc(dev, "MTK GPIO Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
mtk_gpio_pin_set_direction(struct mtk_gpio_softc *sc, uint32_t pin,
    uint32_t dir)
{
	uint32_t regval, mask = (1u << pin);

	if (!(sc->pins[pin].pin_caps & dir))
		return (EINVAL);

	regval = MTK_READ_4(sc, GPIO_PIODIR(sc));
	if (dir == GPIO_PIN_INPUT)
		regval &= ~mask;
	else
		regval |= mask;
	MTK_WRITE_4(sc, GPIO_PIODIR(sc), regval);

	sc->pins[pin].pin_flags &= ~(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT);
	sc->pins[pin].pin_flags |= dir;

	return (0);
}

static int
mtk_gpio_pin_set_invert(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t val)
{
	uint32_t regval, mask = (1u << pin);

	regval = MTK_READ_4(sc, GPIO_PIOPOL(sc));
	if (val)
		regval |= mask;
	else
		regval &= ~mask;
	MTK_WRITE_4(sc, GPIO_PIOPOL(sc), regval);
	sc->pins[pin].pin_flags &= ~(GPIO_PIN_INVIN | GPIO_PIN_INVOUT);
	sc->pins[pin].pin_flags |= val;

	return (0);
}

static void
mtk_gpio_pin_probe(struct mtk_gpio_softc *sc, uint32_t pin)
{
	uint32_t mask = (1u << pin);
	uint32_t val;

	/* Clear cached gpio config */
	sc->pins[pin].pin_flags = 0;

	val = MTK_READ_4(sc, GPIO_PIORENA(sc)) |
	    MTK_READ_4(sc, GPIO_PIOFENA(sc));
	if (val & mask) {
		/* Pin is in interrupt mode */
		sc->pins[pin].intr_trigger = INTR_TRIGGER_EDGE;
		val = MTK_READ_4(sc, GPIO_PIORENA(sc));
		if (val & mask)
			sc->pins[pin].intr_polarity = INTR_POLARITY_HIGH;
		else
			sc->pins[pin].intr_polarity = INTR_POLARITY_LOW;
	}

	val = MTK_READ_4(sc, GPIO_PIODIR(sc));
	if (val & mask)
		sc->pins[pin].pin_flags |= GPIO_PIN_OUTPUT;
	else
		sc->pins[pin].pin_flags |= GPIO_PIN_INPUT;

	val = MTK_READ_4(sc, GPIO_PIOPOL(sc));
	if (val & mask) {
		if (sc->pins[pin].pin_flags & GPIO_PIN_INPUT) {
			sc->pins[pin].pin_flags |= GPIO_PIN_INVIN;
		} else {
			sc->pins[pin].pin_flags |= GPIO_PIN_INVOUT;
		}
	}
}

static int
mtk_gpio_attach(device_t dev)
{
	struct mtk_gpio_softc *sc;
	phandle_t node;
	uint32_t i, num_pins;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, mtk_gpio_spec, sc->res)) {
		device_printf(dev, "could not allocate resources for device\n");
		return (ENXIO);
	}

	MTK_GPIO_LOCK_INIT(sc);

	node = ofw_bus_get_node(dev);
	if (OF_hasprop(node, "mtk,register-gap")) {
		device_printf(dev, "<register gap>\n");
		sc->do_remap = 1;
	} else {
		device_printf(dev, "<no register gap>\n");
		sc->do_remap = 0;
	}

	OF_getencprop(node, "mtk,num-pins", &num_pins, sizeof(num_pins));

	sc->num_pins = num_pins;

	for (i = 0; i < num_pins; i++) {
		sc->pins[i].pin_caps |= GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
		    GPIO_PIN_INVIN | GPIO_PIN_INVOUT;
		sc->pins[i].intr_polarity = INTR_POLARITY_HIGH;
		sc->pins[i].intr_trigger = INTR_TRIGGER_EDGE;

		snprintf(sc->pins[i].pin_name, GPIOMAXNAME - 1, "gpio%c%d",
		    device_get_unit(dev) + 'a', i);
		sc->pins[i].pin_name[GPIOMAXNAME - 1] = '\0';

		mtk_gpio_pin_probe(sc, i);
	}

	if (intr_pic_register(dev, OF_xref_from_node(node)) != 0) {
		device_printf(dev, "could not register PIC\n");
		goto fail;
	}

	if (bus_setup_intr(dev, sc->res[1], INTR_TYPE_MISC | INTR_MPSAFE,
	    mtk_gpio_intr, NULL, sc, &sc->intrhand) != 0)
		goto fail_pic;

	sc->busdev = gpiobus_attach_bus(dev);
	if (sc->busdev == NULL)
		goto fail_pic;

	return (0);
fail_pic:
	intr_pic_unregister(dev, OF_xref_from_node(node));
fail:
	if(sc->intrhand != NULL)
		bus_teardown_intr(dev, sc->res[1], sc->intrhand);
	bus_release_resources(dev, mtk_gpio_spec, sc->res);
	MTK_GPIO_LOCK_DESTROY(sc);
	return (ENXIO);
}

static int
mtk_gpio_detach(device_t dev)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);
	phandle_t node;

	node = ofw_bus_get_node(dev);
	intr_pic_unregister(dev, OF_xref_from_node(node));
	if (sc->intrhand != NULL)
		bus_teardown_intr(dev, sc->res[1], sc->intrhand);
	bus_release_resources(dev, mtk_gpio_spec, sc->res);
	MTK_GPIO_LOCK_DESTROY(sc);
	return (0);
}

static device_t
mtk_gpio_get_bus(device_t dev)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
mtk_gpio_pin_max(device_t dev, int *maxpin)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	*maxpin = sc->num_pins - 1;

	return (0);
}

static int
mtk_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	if (pin >= sc->num_pins)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	*caps = sc->pins[pin].pin_caps;
	MTK_GPIO_UNLOCK(sc);

	return (0);
}

static int
mtk_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	if (pin >= sc->num_pins)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	*flags = sc->pins[pin].pin_flags;
	MTK_GPIO_UNLOCK(sc);

	return (0);
}

static int
mtk_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	if (pin >= sc->num_pins)
		return (EINVAL);

	strncpy(name, sc->pins[pin].pin_name, GPIOMAXNAME - 1);
	name[GPIOMAXNAME - 1] = '\0';

	return (0);
}

static int
mtk_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct mtk_gpio_softc *sc;
	int retval;

	sc = device_get_softc(dev);

	if (pin >= sc->num_pins)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	retval = mtk_gpio_pin_set_direction(sc, pin,
	    flags & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT));
	if (retval == 0)
		retval = mtk_gpio_pin_set_invert(sc, pin,
		    flags & (GPIO_PIN_INVIN | GPIO_PIN_INVOUT));
	MTK_GPIO_UNLOCK(sc);

	return (retval);
}

static int
mtk_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	if (pin >= sc->num_pins || !(sc->pins[pin].pin_flags & GPIO_PIN_OUTPUT))
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	if (value)
		MTK_WRITE_4(sc, GPIO_PIOSET(sc), (1u << pin));
	else
		MTK_WRITE_4(sc, GPIO_PIORESET(sc), (1u << pin));
	MTK_GPIO_UNLOCK(sc);

	return (0);
}

static int
mtk_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct mtk_gpio_softc *sc;
	uint32_t data;

	sc = device_get_softc(dev);

	if (pin >= sc->num_pins || !(sc->pins[pin].pin_flags & GPIO_PIN_INPUT))
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	data = MTK_READ_4(sc, GPIO_PIODATA(sc));
	MTK_GPIO_UNLOCK(sc);
	*val = (data & (1u << pin)) ? 1 : 0;

	return (0);
}

static int
mtk_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct mtk_gpio_softc *sc = device_get_softc(dev);

	if (pin >= sc->num_pins || !(sc->pins[pin].pin_flags & GPIO_PIN_OUTPUT))
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	MTK_WRITE_4(sc, GPIO_PIOTOG(sc), (1u << pin));
	MTK_GPIO_UNLOCK(sc);

	return (0);
}

static int
mtk_gpio_pic_register(device_t dev, struct intr_irqsrc *isrc,
    boolean_t *is_percpu)
{
	struct mtk_gpio_softc *sc;
	uint32_t pin, tripol;

	sc = device_get_softc(dev);

	if (isrc->isrc_ncells != 2) {
		device_printf(sc->dev, "Invalid #interrupt-cells");
		return (EINVAL);
	}

	pin = isrc->isrc_cells[0];
	tripol = isrc->isrc_cells[1];
	if (pin >= sc->num_pins) {
		device_printf(sc->dev, "Invalid interrupt number %d", pin);
		return (EINVAL);
	}

	switch (tripol)
	{
	case IRQ_TYPE_EDGE_RISING:
		isrc->isrc_trig = INTR_TRIGGER_EDGE;
		isrc->isrc_pol  = INTR_POLARITY_HIGH;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		isrc->isrc_trig = INTR_TRIGGER_EDGE;
		isrc->isrc_pol  = INTR_POLARITY_LOW;
		break;
	default:
		device_printf(sc->dev, "unsupported trigger/polarity 0x%2x\n",
		    tripol);
		return (ENOTSUP);
	}

	isrc->isrc_nspc_type = INTR_IRQ_NSPC_PLAIN;
	isrc->isrc_nspc_num = pin;

	/*
	 * 1. The link between ISRC and controller must be set atomically.
	 * 2. Just do things only once in rare case when consumers
	 *    of shared interrupt came here at the same moment.
	 */
	MTK_GPIO_LOCK(sc);
	if (sc->pins[pin].pin_irqsrc != NULL) {
		MTK_GPIO_UNLOCK(sc);
		return (sc->pins[pin].pin_irqsrc == isrc ? 0 : EEXIST);
	}
	sc->pins[pin].pin_irqsrc = isrc;
	isrc->isrc_data = pin;
	MTK_GPIO_UNLOCK(sc);

	intr_irq_set_name(isrc, "%s,%u", device_get_nameunit(sc->dev), pin);
	return (0);
}

static int
mtk_gpio_pic_unregister(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	u_int irq;

	sc = device_get_softc(dev);

	MTK_GPIO_LOCK(sc);
	irq = isrc->isrc_data;
	if (sc->pins[irq].pin_irqsrc != isrc) {
		MTK_GPIO_UNLOCK(sc);
		return (sc->pins[irq].pin_irqsrc == NULL ? 0 : EINVAL);
	}
	sc->pins[irq].pin_irqsrc = NULL;
	isrc->isrc_data = 0;
	MTK_GPIO_UNLOCK(sc);

	intr_irq_set_name(isrc, "%s", "");
	return (0);
}

static void
mtk_gpio_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	uint32_t pin, mask, val;

	sc = device_get_softc(dev);

	pin = isrc->isrc_data;
	mask = 1u << pin;

	MTK_GPIO_LOCK(sc);

	if (isrc->isrc_pol == INTR_POLARITY_LOW) {
		val = MTK_READ_4(sc, GPIO_PIORENA(sc)) & ~mask;
		MTK_WRITE_4(sc, GPIO_PIORENA(sc), val);
		val = MTK_READ_4(sc, GPIO_PIOFENA(sc)) | mask;
		MTK_WRITE_4(sc, GPIO_PIOFENA(sc), val);
	} else {
		val = MTK_READ_4(sc, GPIO_PIOFENA(sc)) & ~mask;
		MTK_WRITE_4(sc, GPIO_PIOFENA(sc), val);
		val = MTK_READ_4(sc, GPIO_PIORENA(sc)) | mask;
		MTK_WRITE_4(sc, GPIO_PIORENA(sc), val);
	}

	sc->pins[pin].intr_trigger = isrc->isrc_trig;
	sc->pins[pin].intr_polarity = isrc->isrc_pol;

	MTK_GPIO_UNLOCK(sc);
}

static void
mtk_gpio_pic_enable_source(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	if (isrc->isrc_pol == INTR_POLARITY_LOW) {
		val = MTK_READ_4(sc, GPIO_PIOFENA(sc));
		MTK_WRITE_4(sc, GPIO_PIOFENA(sc),
		    val | (1u << isrc->isrc_data));
	} else {
		val = MTK_READ_4(sc, GPIO_PIORENA(sc));
		MTK_WRITE_4(sc, GPIO_PIORENA(sc),
		    val | (1u << isrc->isrc_data));
	}
}

static void
mtk_gpio_pic_disable_source(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	if (isrc->isrc_pol == INTR_POLARITY_LOW) {
		val = MTK_READ_4(sc, GPIO_PIOFENA(sc));
		MTK_WRITE_4(sc, GPIO_PIOFENA(sc),
		    val & ~(1u << isrc->isrc_data));
	} else {
		val = MTK_READ_4(sc, GPIO_PIORENA(sc));
		MTK_WRITE_4(sc, GPIO_PIORENA(sc),
		    val & ~(1u << isrc->isrc_data));
	}
}

static void
mtk_gpio_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	mtk_gpio_pic_disable_source(dev, isrc);
}

static void
mtk_gpio_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	mtk_gpio_pic_enable_source(dev, isrc);
}

static void
mtk_gpio_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);
	MTK_WRITE_4(sc, GPIO_PIOINT(sc), 1u << isrc->isrc_data);
}

static int
mtk_gpio_intr(void *arg)
{
	struct mtk_gpio_softc *sc;
	uint32_t i, interrupts;

	sc = arg;
	interrupts = MTK_READ_4(sc, GPIO_PIOINT(sc));

	for (i = 0; interrupts != 0; i++, interrupts >>= 1) {
		if ((interrupts & 0x1) == 0)
			continue;
                if (sc->pins[i].pin_irqsrc)
			intr_irq_dispatch(sc->pins[i].pin_irqsrc,
			    curthread->td_intr_frame);
		else
			device_printf(sc->dev, "spurious interrupt %d\n", i);
	}       

	return (FILTER_HANDLED);
}

static device_method_t mtk_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_gpio_probe),
	DEVMETHOD(device_attach,	mtk_gpio_attach),
	DEVMETHOD(device_detach,	mtk_gpio_detach),
        
	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		mtk_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		mtk_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	mtk_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	mtk_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	mtk_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	mtk_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		mtk_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		mtk_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	mtk_gpio_pin_toggle),
 
	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_source,	mtk_gpio_pic_disable_source),
	DEVMETHOD(pic_enable_intr,	mtk_gpio_pic_enable_intr),
	DEVMETHOD(pic_enable_source,	mtk_gpio_pic_enable_source),
	DEVMETHOD(pic_post_filter,	mtk_gpio_pic_post_filter),
	DEVMETHOD(pic_post_ithread,	mtk_gpio_pic_post_ithread),
	DEVMETHOD(pic_pre_ithread,	mtk_gpio_pic_pre_ithread),
	DEVMETHOD(pic_register,		mtk_gpio_pic_register),
	DEVMETHOD(pic_unregister,	mtk_gpio_pic_unregister),

	DEVMETHOD_END
};
        
static driver_t mtk_gpio_driver = {
	"gpio",
	mtk_gpio_methods,
	sizeof(struct mtk_gpio_softc),
};
        
static devclass_t mtk_gpio_devclass;
        
EARLY_DRIVER_MODULE(mtk_gpio, simplebus, mtk_gpio_driver,
    mtk_gpio_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
