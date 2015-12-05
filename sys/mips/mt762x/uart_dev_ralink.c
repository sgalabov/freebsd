#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_dev_ns8250.h>

#include "uart_if.h"

struct rt8250_softc {
	struct ns8250_softc ns8250_base;
};

static kobj_method_t rt8250_methods[] = {
        KOBJMETHOD(uart_probe,          ns8250_bus_probe),

        KOBJMETHOD(uart_attach,         ns8250_bus_attach),
        KOBJMETHOD(uart_detach,         ns8250_bus_detach),
        KOBJMETHOD(uart_flush,          ns8250_bus_flush),
        KOBJMETHOD(uart_getsig,         ns8250_bus_getsig),
        KOBJMETHOD(uart_ioctl,          ns8250_bus_ioctl),
        KOBJMETHOD(uart_ipend,          ns8250_bus_ipend),
        KOBJMETHOD(uart_param,          ns8250_bus_param),
        KOBJMETHOD(uart_receive,        ns8250_bus_receive),
        KOBJMETHOD(uart_setsig,         ns8250_bus_setsig),
        KOBJMETHOD(uart_transmit,       ns8250_bus_transmit),
        KOBJMETHOD_END
};

struct uart_class uart_ns8250_class = {
        "rt305x",
        rt8250_methods,
        sizeof(struct rt8250_softc),
        .uc_ops = &uart_ns8250_ops,
        .uc_range = 0x88,
        .uc_rclk = 50000000,
        .uc_rshift = 2
};
