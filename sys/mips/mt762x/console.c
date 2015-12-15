#include <sys/cdefs.h>
__FBSDID("$FreeBSD:");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/time.h>

#include <sys/cons.h>
#include <sys/consio.h>

#include <mips/rt305x/rt305xreg.h>
#include <mips/rt305x/console.h>

static cn_probe_t       uart_cnprobe;
static cn_init_t        uart_cninit;
static cn_term_t        uart_cnterm;
static cn_getc_t        uart_cngetc;
static cn_putc_t        uart_cnputc;
static cn_grab_t	uart_cngrab;
static cn_ungrab_t	uart_cnungrab;

static void
uart_cnprobe(struct consdev *cp)
{
        sprintf(cp->cn_name, "uart");
        cp->cn_pri = CN_NORMAL;
}

static void
uart_cninit(struct consdev *cp)
{
}

void
uart_cnputc(struct consdev *cp, int c)
{
        char chr = c;

        // Wait for any ongoing TX to finish
        while (!(LSR(CONFIG_CONSOLE) & LSR_TEMT));

        // Send byte out
        TBR(CONFIG_CONSOLE) = chr;

        // We need to send LF after CR
        if (chr == '\n')
                uart_cnputc(cp, '\r');
}

int
uart_cngetc(struct consdev * cp)
{
        // Wait for data to become available
        while (!(LSR(CONFIG_CONSOLE) & LSR_DR));

        // Read and return the resulting byte
        return (RBR(CONFIG_CONSOLE) & 0xff);
}

static void
uart_cnterm(struct consdev * cp)
{
        // XXX: Nothing for now
}

static void
uart_cngrab(struct consdev *cp)
{
}

static void
uart_cnungrab(struct consdev * cp)
{
}

CONSOLE_DRIVER(uart);
