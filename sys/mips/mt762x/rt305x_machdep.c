/*-
 * Copyright (C) 2010-2011 by Aleksandr Rybalko. All rights reserved.
 * Copyright (C) 2007 by Oleksandr Tymoshenko. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR HIS RELATIVES BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF MIND, USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_ddb.h"
#include "opt_global.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/imgact.h>
#include <sys/bio.h>
#include <sys/buf.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/cons.h>
#include <sys/exec.h>
#include <sys/ucontext.h>
#include <sys/proc.h>
#include <sys/kdb.h>
#include <sys/ptrace.h>
#include <sys/reboot.h>
#include <sys/signalvar.h>
#include <sys/sysent.h>
#include <sys/sysproto.h>
#include <sys/user.h>

#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>

#include <machine/bus.h>
#include <machine/cache.h>
#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpuinfo.h>
#include <machine/cpufunc.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/intr_machdep.h>
#include <machine/locore.h>
#include <machine/md_var.h>
#include <machine/pte.h>
#include <machine/sigframe.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#include <mips/mt762x/rt305xreg.h>
#include <mips/mt762x/rt305x_sysctlvar.h>
#include <mips/mt762x/gic.h>
#include <mips/mt762x/mips_cm.h>

extern int	*edata;
extern int	*end;
static char 	boot1_env[0x1000];

#define RT305X_IPI_INTERRUPT 41

#define RESET()	do { \
	*((volatile uint32_t *)0xbe000034) = 1; \
        while (1);				\
} while (0)

#define ECHO() do { \
	*((volatile uint32_t *)0xbe000c04) = 'e'; \
	*((volatile uint32_t *)0xbe000d04) = 'e'; \
} while (0)

void
platform_cpu_init()
{

	/* Nothing special */
}

static void
mips_init(void)
{
	int i;

	printf("entry: mips_init()\n");

	bootverbose = 1;
	//realmem = btoc(448 << 20);
	realmem = btoc(256 << 20);

	for (i = 0; i < 10; i++) {
		phys_avail[i] = 0;
	}

	/* phys_avail regions are in bytes */
	dump_avail[0] = phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
	dump_avail[1] = phys_avail[1] = ctob(realmem);

	physmem = realmem;

	init_param1();
	init_param2(physmem);
	mips_cpu_init();
#if defined(CPU_MIPS1004KC)
	if ((*(uint32_t *)(0xbf403000) & 0x81) != 0x81) {
		printf("HW COHERENCY DISABLED!\n");
		cpuinfo.cache_coherent_dma = FALSE;
	} else {
		printf("HW COHERENCY ENABLED!\n");
		cpuinfo.cache_coherent_dma = TRUE;
	}
	cpuinfo.cache_coherent_dma = FALSE;
#endif
	pmap_bootstrap();
	mips_proc0_init();
	mutex_init();
	kdb_init();
#ifdef KDB
	if (boothowto & RB_KDB)
		kdb_enter(KDB_WHY_BOOTFLAGS, "Boot flags requested debugger");
#endif
}

void
platform_reset(void)
{

#if 0
	__asm __volatile("li	$25, 0xbf000000");
	__asm __volatile("j	$25");
#else
	RESET();
#endif
	while (1);
}

struct cpulaunch {
        unsigned long   pc;
        unsigned long   gp;
        unsigned long   sp;
        unsigned long   a0;
        unsigned long   _pad[3];
        unsigned long   flags;
};

#define CPULAUNCH       0x00000f00
#define LOG2CPULAUNCH   5
#define LAUNCH_FREADY   1
#define LAUNCH_FGO      2
#define LAUNCH_FGONE    4

void mpwait(void);

static void mips_mt_park_aps(void);

void
platform_start(__register_t a0 __unused, __register_t a1 __unused, 
    __register_t a2 __unused, __register_t a3 __unused)
{
	vm_offset_t kernend;
//	uint64_t platform_counter_freq = PLATFORM_COUNTER_FREQ;
	int i;
	int argc = a0;
	char **argv = (char **)MIPS_PHYS_TO_KSEG0(a1);
	char **envp = (char **)MIPS_PHYS_TO_KSEG0(a2);

	/* clear the BSS and SBSS segments */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

	//mips_timer_early_init(440000000);
	mips_timer_init_params(440000000, 0);

	/* initialize console so that we have printf */
	boothowto |= (RB_SERIAL | RB_MULTIPLE);	/* Use multiple consoles */
	boothowto |= (RB_VERBOSE); // | (RB_SINGLE);
	cninit();

	init_static_kenv(boot1_env, sizeof(boot1_env));

	printf("U-Boot args (from %d args):\n", argc - 1);

	if (argc == 1)
		printf("\tNone\n");

	for (i = 1; i < argc; i++) {
		char *n = "argv  ", *arg;

		if (i > 99)
			break;

		if (argv[i])
		{
			arg = (char *)(intptr_t)MIPS_PHYS_TO_KSEG0(argv[i]);
			printf("\targv[%d] = %s\n", i, arg);
			sprintf(n, "argv%d", i);
			kern_setenv(n, arg);
		}
	}

	printf("Environment:\n");

	for (i = 0; envp[i] ; i++) {
		char *n, *arg;

		arg = (char *)(intptr_t)MIPS_PHYS_TO_KSEG0(envp[i]);
		printf("\t%s\n", arg);
		n = strsep(&arg, "=");
		if (arg == NULL)
			kern_setenv(n, "1");
		else
			kern_setenv(n, arg);
	}

#ifdef SMP
	CPU_ZERO(&platform_active_cpus);
	CPU_SET(0, &platform_active_cpus);
#endif
	if (1)
	mips_mt_park_aps();

	mips_cm_probe();

	mips_init();

	mips_timer_init_params(880000000, 0);

	gic_init();
}

static void
mips_mt_park_aps(void)
{
#ifdef SMP
	volatile struct cpulaunch *launch = (struct cpulaunch *)
					MIPS_PHYS_TO_KSEG0(CPULAUNCH);
	int i = 0, j, parked = 0;

	printf("Parking APs... ");

	for (i = 1; i < 2; i++) {
		launch += 1;
		j = 0;
		launch->pc = (unsigned long) mpwait;
		__asm __volatile ("sync");
		mips_barrier();
		launch->flags |= LAUNCH_FGO;
		__asm __volatile ("sync");
		mips_barrier();
		while ((launch->flags & LAUNCH_FGONE) == 0 && j < 10000)
			j++;
		if (j < 10000) parked++;
		__asm __volatile ("sync");
		mips_barrier();
	}

	printf("done, parked %d APs out of 3\n", parked);
#endif
}

