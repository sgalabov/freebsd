#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/types.h>
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

#ifdef SMP
#include <sys/smp.h>
#include <machine/smp.h>
#endif

#include <mips/mt762x/mips_cm.h>
#include <mips/mt762x/gic.h>
#include <mips/mt762x/interrupt.h>

#define RT305X_IPI_INTERRUPT (IRQ_IPI)

int xlr_ap_release[MAXCPU];
#ifndef SMP
void mpentry(void);
void mpentry(void) {}
#endif

uint32_t mips_cm_base;
uint32_t mips_cm_l2sync_base;
uint32_t mips_cpc_base;

uint32_t
__mips_cm_phys_base(void)
{
	uint32_t config3 = mips_rd_config3();
	uint32_t cmgcr;

	if (!(config3 & MIPS_CONFIG3_CMGCR_MASK))
		return 0;

	cmgcr = mips_rd_cmgcrbase();
	return ((cmgcr & MIPS_CMGCRF_BASE) << (36 - 32));
}

uint32_t mips_cm_phys_base(void)
	 __attribute__((weak, alias("__mips_cm_phys_base")));

uint32_t __mips_cm_l2sync_phys_base(void)
{
	uint32_t base_reg;

	base_reg = read_gcr_l2_only_sync_base();
	if (base_reg & CM_GCR_L2_ONLY_SYNC_BASE_SYNCEN_MSK)
		return (base_reg & CM_GCR_L2_ONLY_SYNC_BASE_SYNCBASE_MSK);

	return mips_cm_phys_base() + MIPS_CM_GCR_SIZE;
}

uint32_t mips_cm_l2sync_phys_base(void)
	 __attribute__((weak, alias("__mips_cm_l2sync_phys_base")));

static inline int
mips_cpc_present(void)
{

	return mips_cpc_base != 0;
}

static uint32_t
mips_cpc_default_phys_base(void)
{

	return (0);
}

static uint32_t
mips_cpc_phys_base(void)
{
	uint32_t cpc_base;

	if (!mips_cm_present())
		return (0);

	if (!(read_gcr_cpc_status() & CM_GCR_CPC_STATUS_EX_MSK))
		return (0);

	cpc_base = read_gcr_cpc_base();
	if (cpc_base & CM_GCR_CPC_BASE_CPCEN_MSK)
		return (cpc_base & CM_GCR_CPC_BASE_CPCBASE_MSK);

	cpc_base = mips_cpc_default_phys_base();
	write_gcr_cpc_base(cpc_base | CM_GCR_CPC_BASE_CPCEN_MSK);

	return (cpc_base);
}

static void
mips_cpc_probe(void)
{
	uint32_t addr;

	addr = mips_cpc_phys_base();
	if (!addr) {
		printf("MIPS CPC not found\n");
		mips_cpc_base = 0;
		return;
	} else {
		printf("MIPS CPC found\n");
	}

	mips_cpc_base = MIPS_PHYS_TO_KSEG1(addr);
}

static void
mips_cm_probe_l2sync(void)
{
	uint32_t major_rev;
	uint32_t addr;

	major_rev = (read_gcr_rev() & CM_GCR_REV_MAJOR_MSK) >>
			CM_GCR_REV_MAJOR_SHF;
	if (major_rev < 6) {
		printf("No L2 sync (%d < 6)\n", major_rev);
		return;
	}

	addr = mips_cm_l2sync_phys_base();
	if (!addr) {
		printf("No L2 sync (no addr)\n");
		return;
	}

	write_gcr_l2_only_sync_base(addr | CM_GCR_L2_ONLY_SYNC_BASE_SYNCEN_MSK);

	mips_cm_l2sync_base = MIPS_PHYS_TO_KSEG1(addr);

	printf("L2 only sync detected\n");
}


static void
dump_coherence_info(void)
{
	write_gcr_cl_other(1);

//	write_gcr_reg1_mask(0xffff0000);
//	write_gcr_base(0x1fbf80b0);

//	write_gcr_cl_coherence(0xff);
//	write_gcr_co_coherence(0xff);

	printf("Local core : 0x%08x\n", read_gcr_cl_coherence());
	printf("Other core : 0x%08x\n", read_gcr_co_coherence());
	printf("Reg0 base  : 0x%08x\n", read_gcr_reg0_base());
	printf("Reg0 mask  : 0x%08x\n", read_gcr_reg0_mask());
	printf("Reg1 base  : 0x%08x\n", read_gcr_reg1_base());
        printf("Reg1 mask  : 0x%08x\n", read_gcr_reg1_mask());
	printf("Reg2 base  : 0x%08x\n", read_gcr_reg2_base());
        printf("Reg2 mask  : 0x%08x\n", read_gcr_reg2_mask());
	printf("Reg3 base  : 0x%08x\n", read_gcr_reg3_base());
        printf("Reg3 mask  : 0x%08x\n", read_gcr_reg3_mask());
	printf("GCR Base   : 0x%08x\n", read_gcr_base());
	printf("GCR Control: 0x%08x\n", read_gcr_control());

	printf("MVPControl : 0x%08x\n", read_c0_mvpcontrol());
	printf("VPEConf0   : 0x%08x\n", read_c0_vpeconf0());

	printf("Num IOCUs  : %d\n", mips_cm_numiocu());

	printf("Config0    : 0x%08x\n", mips_rd_config());
	printf("Config7    : 0x%08x\n", mips_rd_config7());

}

extern int
mips_cm_probe(void)
{
	uint32_t addr;
	uint32_t base_reg;

	addr = mips_cm_phys_base();
	if (!addr)
		return (-1);

	mips_cm_base = MIPS_PHYS_TO_KSEG1(addr);

	base_reg = read_gcr_base();
	if ((base_reg & CM_GCR_BASE_GCRBASE_MSK) != addr) {
		printf("GCRs appear to be moved.\n");
		mips_cm_base = 0;
		return (-1);
	} else {
		printf("GCRs found at 0x%08x.\n", (uint32_t) addr);
	}

#ifdef notyet
	/* set default target to memory */
	base_reg &= ~CM_GCR_BASE_CMDEFTGT_MSK;
	base_reg |= CM_GCR_BASE_CMDEFTGT_MEM;
	write_gcr_base(base_reg);

	base_reg = mips_rd_config();
        base_reg &= ~(0x7);
        base_reg |= 0x5;
        mips_wr_config(base_reg);

        printf("%s(): 0x%08x\n", __FUNCTION__, mips_rd_config());
#endif
#ifdef notyet
	/* disable CM regions */
	write_gcr_reg0_base(CM_GCR_REGn_BASE_BASEADDR_MSK);
	write_gcr_reg0_mask(CM_GCR_REGn_MASK_ADDRMASK_MSK);
	write_gcr_reg1_base(CM_GCR_REGn_BASE_BASEADDR_MSK);
	write_gcr_reg1_mask(CM_GCR_REGn_MASK_ADDRMASK_MSK);
	write_gcr_reg2_base(CM_GCR_REGn_BASE_BASEADDR_MSK);
	write_gcr_reg2_mask(CM_GCR_REGn_MASK_ADDRMASK_MSK);
	write_gcr_reg3_base(CM_GCR_REGn_BASE_BASEADDR_MSK);
	write_gcr_reg3_mask(CM_GCR_REGn_MASK_ADDRMASK_MSK);
#endif

	/* probe for an L2-only sync region */
	mips_cm_probe_l2sync();

	mips_cpc_probe();

	dump_coherence_info();

	return 0;
}

#if 0
static int
mips_cm_numvpes(void)
{
	uint32_t ncores, nvpes, core_vpes, c, core_cfg;

	ncores = mips_cm_numcores();
	for (c = nvpes = 0; c < ncores; c++) {
		write_gcr_cl_other(c << CM_GCR_Cx_OTHER_CORENUM_SHF);
		core_cfg = read_gcr_co_config();
		core_vpes = ((core_cfg & CM_GCR_Cx_CONFIG_PVPE_MSK) >>
				CM_GCR_Cx_CONFIG_PVPE_SHF) + 1;
		nvpes += core_vpes;
	}

	write_gcr_cl_coherence(0xff);

	return nvpes;
}
#endif

#ifdef SMP
static uint32_t _gic_base = 0xbfbc0000;

#define MSK(n) ((1 << (n)) - 1)

#define GIC_SH_CONFIG_NUMINTRS_SHF      16
#define GIC_SH_CONFIG_NUMINTRS_MSK      (MSK(8) << GIC_SH_CONFIG_NUMINTRS_SHF)

#define GIC_SH_CONFIG_NUMVPES_SHF       0
#define GIC_SH_CONFIG_NUMVPES_MSK       (MSK(8) << GIC_SH_CONFIG_NUMVPES_SHF)

#define SHARED_SECTION_OFS              0x0000
#define GIC_SH_WEDGE_OFS                0x0280
#define GIC_SH_CONFIG_OFS               0x0000
#define VPE_LOCAL_SECTION_OFS           0x8000
#define VPE_OTHER_SECTION_OFS           0xc000

#define GIC_VPE_CTL_OFS                 0x0000
#define GIC_VPE_OTHER_ADDR_OFS          0x0080

#define GICWRITE(reg, data)	((reg) = (data))
#define GICREAD(reg, data)	((data) = (reg))

#define REG32(addr)		(*(volatile uint32_t *)(addr))

#define GIC_REG(segment, offset) \
	REG32(_gic_base + segment##_##SECTION_OFS + offset##_##OFS)


static void
print_irq_info(int vpe)
{
	uint32_t vpe_ctl;

	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), vpe);
	GICREAD(GIC_REG(VPE_OTHER, GIC_VPE_CTL), vpe_ctl);
	printf("%s(): vpe %d, VPE_CTL 0x%08x\n", __FUNCTION__, vpe, vpe_ctl);
}

extern void gic_init_old(void);

extern void
gic_init_old(void)
{
	uint32_t gicconfig;
        int numvpes, numintrs, i;

        GICREAD(GIC_REG(SHARED, GIC_SH_CONFIG), gicconfig);
        numintrs = (gicconfig & GIC_SH_CONFIG_NUMINTRS_MSK) >>
                   GIC_SH_CONFIG_NUMINTRS_SHF;
        numintrs = ((numintrs + 1) * 8);

        numvpes = (gicconfig & GIC_SH_CONFIG_NUMVPES_MSK) >>
                  GIC_SH_CONFIG_NUMVPES_SHF;
        numvpes = numvpes + 1;

	printf("%s(): numvpes %d, numintrs %d\n", __FUNCTION__, numvpes, numintrs);

	for (i = 0; i < numvpes; i++) {
		print_irq_info(i);
	}
}

void
platform_ipi_send(int cpuid)
{

	//printf("%s() for cpuid %d\n", __FUNCTION__, cpuid);
	gic_write(gic_reg(0x280), 0x80000000 | (32 + cpuid));
}

void
platform_ipi_clear(void)
{

	//printf("%d\n", platform_processor_id());
	gic_write(gic_reg(0x280), (32 + platform_processor_id()));
}

int
platform_ipi_intrnum(void)
{

	if (platform_processor_id() != 0)
	printf("%s(): %d\n", __FUNCTION__, platform_processor_id());
	return (RT305X_IPI_INTERRUPT);
}

struct cpu_group *
platform_smp_topo(void)
{

//	return (smp_topo_2level(CG_SHARE_L2, 2, CG_SHARE_L1, 2, CG_FLAG_THREAD));
	return (smp_topo_none());
}

void
platform_init_ap(int cpuid)
{
#if 1
	uint32_t reg_val;

	reg_val = mips_rd_status();
//	printf("%s(): %d, 0x%08x\n", __FUNCTION__, cpuid, reg_val);
	reg_val |= (3 << (8 + 2 + 4));
	reg_val &= ~(1 << (8 + 2 + 5));
	mips_wr_status(reg_val);
#endif
}

volatile cpuset_t platform_active_cpus;

void
platform_cpu_mask(cpuset_t *mask)
{
	uint32_t i;

	CPU_ZERO(mask);
	for (i = 0; i < 2; i++)
		CPU_SET(i, mask);
	printf("%s(): %d\n", __FUNCTION__, CPU_COUNT(mask));
}

int
platform_processor_id(void)
{

	return (mips_rd_ebase() & 0x3ff);
}

#if 0
static int
boot_core(int cpuid)
{
	uint32_t access, stat, seq_state, timeout;

	write_gcr_cl_other(cpuid << CM_GCR_Cx_OTHER_CORENUM_SHF);
	write_gcr_co_reset_base((uint32_t)mpentry);
	write_gcr_co_coherence(0);
	
	access = read_gcr_access();
	access |= 1 << (CM_GCR_ACCESS_ACCESSEN_SHF + cpuid);
	write_gcr_access(access);

	if (mips_cpc_present()) {
		write_cpc_cl_other(cpuid << CPC_Cx_OTHER_CORENUM_SHF);
//		write_cpc_co_cmd(CPC_Cx_CMD_PWRUP);
		write_cpc_co_cmd(CPC_Cx_CMD_RESET);
		timeout = 100;
		while (true) {
			stat = read_cpc_co_stat_conf();
			seq_state = stat & CPC_Cx_STAT_CONF_SEQSTATE_MSK;
			if (seq_state == CPC_Cx_STAT_CONF_SEQSTATE_U6)
				break;

			if (timeout) {
				timeout--;
				DELAY(10);
				continue;
			} else {
				break;
			}

			printf("Waiting for core %u to start... STAT_CONF=0x%x\n", cpuid, stat);
			DELAY(1000);
		}
	} else {
		write_gcr_co_reset_release(0);
	}

	printf("%s(): cpuid %d, entry 0x%08x, 0x%08x\n",
		__FUNCTION__, cpuid, (uint32_t)mpentry,
		(uint32_t)MIPS_PHYS_TO_KSEG1(mpentry));

	return (0);
}

static int
boot_vpe(int cpuid)
{
	uint32_t tcstatus, vpeconf0;

	dvpe();
	set_c0_mvpcontrol(MVPCONTROL_VPC);

	settc(cpuid);

	write_tc_c0_tcrestart((uint32_t)mpentry);
	tcstatus = read_tc_c0_tcstatus();
	tcstatus &= ~TCSTATUS_IXMT;
	tcstatus |= TCSTATUS_A;
	write_tc_c0_tcstatus(tcstatus);

	write_tc_c0_tchalt(0);

	vpeconf0 = read_vpe_c0_vpeconf0();
	vpeconf0 |= VPECONF0_VPA;
	write_vpe_c0_vpeconf0(vpeconf0);

	clear_c0_mvpcontrol(MVPCONTROL_VPC);
	evpe(EVPE_ENABLE);

	return (0);
}
#endif

//int xlr_ap_release[MAXCPU];

struct cpulaunch {
	unsigned long	pc;
	unsigned long	gp;
	unsigned long	sp;
	unsigned long	a0;
	unsigned long	_pad[3];
	unsigned long	flags;
};

#define CPULAUNCH       0x00000f00
#define LOG2CPULAUNCH	5
#define LAUNCH_FREADY   1
#define LAUNCH_FGO      2
#define LAUNCH_FGONE    4

int
platform_start_ap(int cpuid)
{

	if (cpuid == 0)
		return 0;

#if 1
#if 1
	xlr_ap_release[cpuid] = 1;
	__asm __volatile("sync");
	mips_barrier();
	return (0);
#else
{
	struct cpulaunch *launch =
		(struct cpulaunch *)
		MIPS_PHYS_TO_KSEG1(CPULAUNCH + (cpuid << LOG2CPULAUNCH));
	launch->pc = (unsigned long)mpentry;
	mips_barrier();
	__asm __volatile ("sync");
	launch->flags |= LAUNCH_FGO;
	__asm __volatile ("sync");
	mips_barrier();
	//printf("Waiting for CPU%d to get going...", cpuid);
	//while ((launch->flags & LAUNCH_FGONE) == 0)
	//	printf(".");
	//printf("gone\n");
	__asm __volatile ("sync");
	mips_barrier();
	return 0;
}
#endif
#else
	if (cpuid == 2)
		return boot_vpe(1);

	if (cpuid == 1)
		return boot_core(1);

	return (-1);
#endif
}
#endif

extern void
__synci(void)
{
	synci();
}
