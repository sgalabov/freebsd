/*-
 * Copyright (C) 2015 by Stanislav Galabov. All rights reserved.
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

#include <mips/mtk/mtk_sysctlreg.h>

#include "opt_platform.h"
#include "opt_rt305x.h"

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

extern int	*edata;
extern int	*end;
static char 	boot1_env[0x1000];


void
platform_cpu_init()
{
	/* Nothing special */
}

static void
mips_init(void)
{
	int i;
	char *memsize;

	printf("entry: mips_init()\n");

	if ((memsize = kern_getenv("memsize")) != NULL)
		realmem = btoc(strtol(memsize, NULL, 0) << 20);
	else
		realmem = btoc(32 << 20);
	

	bootverbose = 1;

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

#if !defined(MT7620) && !defined(RT5350)
	__asm __volatile("li	$25, 0xbf000000");
	__asm __volatile("j	$25");
#else
	mtk_sysctl_set(SYSCTL_RSTCTRL, 1);
	while (1);
#endif
}

#define DEFAULT_COUNTER_FREQ	1000000000UL

void
platform_start(__register_t a0 __unused, __register_t a1 __unused, 
    __register_t a2 __unused, __register_t a3 __unused)
{
	vm_offset_t kernend;
	uint32_t platform_counter_freq;
	phandle_t pclock;
	int argc = a0, counter_freq_found = 0, i;
	char **argv = (char **)MIPS_PHYS_TO_KSEG0(a1);
	char **envp = (char **)MIPS_PHYS_TO_KSEG0(a2);
	void *dtbp;

	/* clear the BSS and SBSS segments */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

	dtbp = &fdt_static_dtb;
	if (OF_install(OFW_FDT, 0) == FALSE)
		while (1);
	if (OF_init((void *)dtbp) != 0)
		while (1);

	pclock = OF_finddevice("/cpus/cpu@0");
	if (pclock != -1) {
		if (OF_getencprop(pclock, "clock-frequency",
		    &platform_counter_freq, sizeof(platform_counter_freq)) <= 0)
			platform_counter_freq = DEFAULT_COUNTER_FREQ;
		else
			counter_freq_found = 1;
	} else {
		platform_counter_freq = DEFAULT_COUNTER_FREQ;
	}

	mips_timer_early_init(platform_counter_freq / 2);

	/* initialize console so that we have printf */
	boothowto |= (RB_SERIAL | RB_MULTIPLE);	/* Use multiple consoles */
	boothowto |= (RB_VERBOSE);
	cninit();

	init_static_kenv(boot1_env, sizeof(boot1_env));

	if (!counter_freq_found)
		printf("CPU clock frequency not defined, using default "
		    "(%dMHz)\n", platform_counter_freq / 1000000);
	else
		printf("CPU clock frequency is %dMHz\n",
		    platform_counter_freq / 1000000);

	printf("FDT dtbp at 0x%08x\n", (uint32_t)dtbp);
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

	for (i = 0; envp[i] && MIPS_IS_VALID_PTR(envp[i]); i++) {
		char *n, *arg;

		arg = (char *)(intptr_t)MIPS_PHYS_TO_KSEG0(envp[i]);
		if (! MIPS_IS_VALID_PTR(arg))
			continue;
		printf("\t%s\n", arg);
		n = strsep(&arg, "=");
		if (arg == NULL)
			kern_setenv(n, "1");
		else
			kern_setenv(n, arg);
	}


	mips_init();
	mips_timer_init_params(platform_counter_freq, 2);
}
