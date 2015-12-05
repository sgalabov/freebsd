#ifndef __MIPS_CM_H__
#define __MIPS_CM_H__

extern uint32_t mips_cm_base;
extern uint32_t mips_cm_l2sync_base;

extern volatile cpuset_t platform_active_cpus;

extern uint32_t __mips_cm_phys_base(void);
extern uint32_t __mips_cm_l2sync_phys_base(void);

extern int mips_cm_probe(void);

static inline int mips_cm_present(void)
{

	return mips_cm_base != 0;
}

static inline int mips_cm_has_l2sync(void)
{

	return mips_cm_l2sync_base != 0;
}

#define MIPS_CM_GCB_OFS		0x0000
#define MIPS_CM_CLCB_OFS	0x2000
#define MIPS_CM_COCB_OFS	0x4000
#define MIPS_CM_GDB_OFS		0x8000

#define MIPS_CM_GCR_SIZE	0x8000
#define MIPS_CM_L2SYNC_SIZE	0x1000

#define BUILD_CM_R_(name, off)				\
static inline uint32_t *addr_gcr_##name(void)		\
{							\
	return (uint32_t *)(mips_cm_base + (off));	\
}							\
							\
static inline uint32_t read_gcr_##name(void)		\
{							\
	return *(addr_gcr_##name());			\
}

#define BUILD_CM__W(name, off)				\
static inline void write_gcr_##name(uint32_t value)	\
{							\
	*(addr_gcr_##name()) = value;			\
}

#define BUILD_CM_RW(name, off)				\
	BUILD_CM_R_(name, off)				\
	BUILD_CM__W(name, off)

#define BUILD_CM_Cx_R_(name, off)			\
	BUILD_CM_R_(cl_##name, MIPS_CM_CLCB_OFS + (off))\
	BUILD_CM_R_(co_##name, MIPS_CM_COCB_OFS + (off))

#define BUILD_CM_Cx__W(name, off)			\
	BUILD_CM__W(cl_##name, MIPS_CM_CLCB_OFS + (off))\
	BUILD_CM__W(co_##name, MIPS_CM_COCB_OFS + (off))

#define BUILD_CM_Cx_RW(name, off)			\
	BUILD_CM_Cx_R_(name, off)			\
	BUILD_CM_Cx__W(name, off)

/* GCB register accessor functions */
BUILD_CM_R_(config,		MIPS_CM_GCB_OFS + 0x00)
BUILD_CM_RW(base,		MIPS_CM_GCB_OFS + 0x08)
BUILD_CM_RW(control,		MIPS_CM_GCB_OFS + 0x10)
BUILD_CM_RW(access,		MIPS_CM_GCB_OFS + 0x20)
BUILD_CM_R_(rev,		MIPS_CM_GCB_OFS + 0x30)
BUILD_CM_RW(error_mask,		MIPS_CM_GCB_OFS + 0x40)
BUILD_CM_RW(error_cause,	MIPS_CM_GCB_OFS + 0x48)
BUILD_CM_RW(error_addr,		MIPS_CM_GCB_OFS + 0x50)
BUILD_CM_RW(error_mult,		MIPS_CM_GCB_OFS + 0x58)
BUILD_CM_RW(l2_only_sync_base,	MIPS_CM_GCB_OFS + 0x70)
BUILD_CM_RW(gic_base,		MIPS_CM_GCB_OFS + 0x80)
BUILD_CM_RW(cpc_base,		MIPS_CM_GCB_OFS + 0x88)
BUILD_CM_RW(reg0_base,		MIPS_CM_GCB_OFS + 0x90)
BUILD_CM_RW(reg0_mask,		MIPS_CM_GCB_OFS + 0x98)
BUILD_CM_RW(reg1_base,		MIPS_CM_GCB_OFS + 0xa0)
BUILD_CM_RW(reg1_mask,		MIPS_CM_GCB_OFS + 0xa8)
BUILD_CM_RW(reg2_base,		MIPS_CM_GCB_OFS + 0xb0)
BUILD_CM_RW(reg2_mask,		MIPS_CM_GCB_OFS + 0xb8)
BUILD_CM_RW(reg3_base, 		MIPS_CM_GCB_OFS + 0xc0)
BUILD_CM_RW(reg3_mask,		MIPS_CM_GCB_OFS + 0xc8)
BUILD_CM_R_(gic_status,		MIPS_CM_GCB_OFS + 0xd0)
BUILD_CM_R_(cpc_status,		MIPS_CM_GCB_OFS + 0xf0)

/* Core Local & Core Other register accessor functions */
BUILD_CM_Cx_RW(reset_release,	0x00)
BUILD_CM_Cx_RW(coherence,	0x08)
BUILD_CM_Cx_R_(config,		0x10)
BUILD_CM_Cx_RW(other,		0x18)
BUILD_CM_Cx_RW(reset_base,	0x20)
BUILD_CM_Cx_R_(id,		0x28)
BUILD_CM_Cx_RW(reset_ext_base,	0x30)
BUILD_CM_Cx_R_(tcid_0_priority,	0x40)
BUILD_CM_Cx_R_(tcid_1_priority,	0x48)
BUILD_CM_Cx_R_(tcid_2_priority,	0x50)
BUILD_CM_Cx_R_(tcid_3_priority,	0x58)
BUILD_CM_Cx_R_(tcid_4_priority,	0x60)
BUILD_CM_Cx_R_(tcid_5_priority,	0x68)
BUILD_CM_Cx_R_(tcid_6_priority,	0x70)
BUILD_CM_Cx_R_(tcid_7_priority,	0x78)
BUILD_CM_Cx_R_(tcid_8_priority,	0x80)

/* GCR_CONFIG register fields */
#define CM_GCR_CONFIG_NUMIOCU_SHF	8
#define CM_GCR_CONFIG_NUMIOCU_MSK	(0xf << 8)
#define CM_GCR_CONFIG_PCORES_SHF	0
#define CM_GCR_CONFIG_PCORES_MSK	(0xff << 0)

/* GCR_BASE register fields */
#define CM_GCR_BASE_GCRBASE_SHF		15
#define CM_GCR_BASE_GCRBASE_MSK		(0x1ffff << 15)
#define CM_GCR_BASE_CMDEFTGT_SHF	0
#define CM_GCR_BASE_CMDEFTGT_MSK	(0x3 << 0)
#define  CM_GCR_BASE_CMDEFTGT_DISABLED	0
#define  CM_GCR_BASE_CMDEFTGT_MEM	1
#define  CM_GCR_BASE_CMDEFTGT_IOCU0	2
#define  CM_GCR_BASE_CMDEFTGT_IOCU1	3

/* GCR_ACCESS register fields */
#define CM_GCR_ACCESS_ACCESSEN_SHF	0
#define CM_GCR_ACCESS_ACCESSEN_MSK	(0xff << 0)

/* GCR_REV register fields */
#define CM_GCR_REV_MAJOR_SHF		8
#define CM_GCR_REV_MAJOR_MSK		(0xff << 8)
#define CM_GCR_REV_MINOR_SHF		0
#define CM_GCR_REV_MINOR_MSK		(0xff << 0)

/* GCR_ERROR_CAUSE register fields */
#define CM_GCR_ERROR_CAUSE_ERRTYPE_SHF	27
#define CM_GCR_ERROR_CAUSE_ERRTYPE_MSK	(0x1f << 27)
#define CM_GCR_ERROR_CAUSE_ERRINFO_SHF	0
#define CM_GCR_ERROR_CAUSE_ERRINFO_MSK	(0x7ffffff << 0)

/* GCR_ERROR_MULT register fields */
#define CM_GCR_ERROR_MULT_ERR2ND_SHF	0
#define CM_GCR_ERROR_MULT_ERR2ND_MSK	(0x1f << 0)

/* GCR_L2_ONLY_SYNC_BASE register fields */
#define CM_GCR_L2_ONLY_SYNC_BASE_SYNCBASE_SHF	12
#define CM_GCR_L2_ONLY_SYNC_BASE_SYNCBASE_MSK	(0xfffff << 12)
#define CM_GCR_L2_ONLY_SYNC_BASE_SYNCEN_SHF	0
#define CM_GCR_L2_ONLY_SYNC_BASE_SYNCEN_MSK	(0x1 << 0)

/* GCR_GIC_BASE register fields */
#define CM_GCR_GIC_BASE_GICBASE_SHF	17
#define CM_GCR_GIC_BASE_GICBASE_MSK	(0x7fff << 17)
#define CM_GCR_GIC_BASE_GICEN_SHF	0
#define CM_GCR_GIC_BASE_GICEN_MSK	(0x1 << 0)

/* GCR_CPC_BASE register fields */
#define CM_GCR_CPC_BASE_CPCBASE_SHF	17
#define CM_GCR_CPC_BASE_CPCBASE_MSK	(0x7fff << 17)
#define CM_GCR_CPC_BASE_CPCEN_SHF	0
#define CM_GCR_CPC_BASE_CPCEN_MSK	(0x1 << 0)

/* GCR_REGn_BASE register fields */
#define CM_GCR_REGn_BASE_BASEADDR_SHF	16
#define CM_GCR_REGn_BASE_BASEADDR_MSK	(0xffff << 16)

/* GCR_REGn_MASK register fields */
#define CM_GCR_REGn_MASK_ADDRMASK_SHF	16
#define CM_GCR_REGn_MASK_ADDRMASK_MSK	(0xffff << 16)
#define CM_GCR_REGn_MASK_CCAOVR_SHF	5
#define CM_GCR_REGn_MASK_CCAOVR_MSK	(0x3 << 5)
#define CM_GCR_REGn_MASK_CCAOVREN_SHF	4
#define CM_GCR_REGn_MASK_CCAOVREN_MSK	(0x1 << 4)
#define CM_GCR_REGn_MASK_DROPL2_SHF	2
#define CM_GCR_REGn_MASK_DROPL2_MSK	(0x1 << 2)
#define CM_GCR_REGn_MASK_CMTGT_SHF	0
#define CM_GCR_REGn_MASK_CMTGT_MSK	(0x3 << 0)
#define  CM_GCR_REGn_MASK_CMTGT_DISABLED	0
#define  CM_GCR_REGn_MASK_CMTGT_MEM	 	1
#define  CM_GCR_REGn_MASK_CMTGT_IOCU0		2
#define  CM_GCR_REGn_MASK_CMTGT_IOCU1		3

/* GCR_GIC_STATUS register fields */
#define CM_GCR_GIC_STATUS_EX_SHF	0
#define CM_GCR_GIC_STATUS_EX_MSK	(0x1 << 0)

/* GCR_CPC_STATUS register fields */
#define CM_GCR_CPC_STATUS_EX_SHF	0
#define CM_GCR_CPC_STATUS_EX_MSK	(0x1 << 0)

/* GCR_Cx_COHERENCE register fields */
#define CM_GCR_Cx_COHERENCE_COHDOMAINEN_SHF	0
#define CM_GCR_Cx_COHERENCE_COHDOMAINEN_MSK	(0xff << 0)

/* GCR_Cx_CONFIG register fields */
#define CM_GCR_Cx_CONFIG_IOCUTYPE_SHF	10
#define CM_GCR_Cx_CONFIG_IOCUTYPE_MSK	(0x3 << 10)
#define CM_GCR_Cx_CONFIG_PVPE_SHF	0
#define CM_GCR_Cx_CONFIG_PVPE_MSK	(0x1ff << 0)

/* GCR_Cx_OTHER register fields */
#define CM_GCR_Cx_OTHER_CORENUM_SHF	16
#define CM_GCR_Cx_OTHER_CORENUM_MSK	(0xffff << 16)

/* GCR_Cx_RESET_BASE register fields */
#define CM_GCR_Cx_RESET_BASE_BEVEXCBASE_SHF	12
#define CM_GCR_Cx_RESET_BASE_BEVEXCBASE_MSK	(0xfffff << 12)

/* GCR_Cx_RESET_EXT_BASE register fields */
#define CM_GCR_Cx_RESET_EXT_BASE_EVARESET_SHF	31
#define CM_GCR_Cx_RESET_EXT_BASE_EVARESET_MSK	(0x1 << 31)
#define CM_GCR_Cx_RESET_EXT_BASE_UEB_SHF	30
#define CM_GCR_Cx_RESET_EXT_BASE_UEB_MSK	(0x1 << 30)
#define CM_GCR_Cx_RESET_EXT_BASE_BEVEXCMASK_SHF	20
#define CM_GCR_Cx_RESET_EXT_BASE_BEVEXCMASK_MSK	(0xff << 20)
#define CM_GCR_Cx_RESET_EXT_BASE_BEVEXCPA_SHF	1
#define CM_GCR_Cx_RESET_EXT_BASE_BEVEXCPA_MSK	(0x7f << 1)
#define CM_GCR_Cx_RESET_EXT_BASE_PRESENT_SHF	0
#define CM_GCR_Cx_RESET_EXT_BASE_PRESENT_MSK	(0x1 << 0)

static inline uint32_t mips_cm_numcores(void)
{

	if (!mips_cm_present())
		return (0);

	return (((read_gcr_config() & CM_GCR_CONFIG_PCORES_MSK)
		>> CM_GCR_CONFIG_PCORES_SHF) + 1);
}

static inline uint32_t mips_cm_numiocu(void)
{

	if (!mips_cm_present())
		return (0);

	return ((read_gcr_config() & CM_GCR_CONFIG_NUMIOCU_MSK)
		>> CM_GCR_CONFIG_NUMIOCU_SHF);
}

static inline int mips_cm_l2sync(void)
{

	if (!mips_cm_has_l2sync())
		return (-1);

	*((uint32_t *)mips_cm_l2sync_base) = 0;
	return (0);
}

/* CPC Defines */
extern uint32_t mips_cpc_base;

#define MIPS_CPC_GCB_OFS		0x0000
#define MIPS_CPC_CLCB_OFS		0x2000
#define MIPS_CPC_COCB_OFS		0x4000

#define BUILD_CPC_R_(name, off) 			\
static inline uint32_t read_cpc_##name(void)		\
{							\
	return *((volatile uint32_t *)mips_cpc_base + (off));	\
}

#define BUILD_CPC__W(name, off)				\
static inline void write_cpc_##name(uint32_t value)	\
{							\
	*((volatile uint32_t *)mips_cpc_base + (off)) = value;	\
}

#define BUILD_CPC_RW(name, off)				\
	BUILD_CPC_R_(name, off)				\
	BUILD_CPC__W(name, off)

#define BUILD_CPC_Cx_R_(name, off)				\
	BUILD_CPC_R_(cl_##name, MIPS_CPC_CLCB_OFS + (off))	\
	BUILD_CPC_R_(co_##name, MIPS_CPC_COCB_OFS + (off))

#define BUILD_CPC_Cx__W(name, off)				\
	BUILD_CPC__W(cl_##name, MIPS_CPC_CLCB_OFS + (off))	\
	BUILD_CPC__W(co_##name, MIPS_CPC_COCB_OFS + (off))

#define BUILD_CPC_Cx_RW(name, off)				\
	BUILD_CPC_Cx_R_(name, off)				\
	BUILD_CPC_Cx__W(name, off)

BUILD_CPC_RW(access,	MIPS_CPC_GCB_OFS + 0x00)
BUILD_CPC_RW(seqdel,	MIPS_CPC_GCB_OFS + 0x08)
BUILD_CPC_RW(rail,	MIPS_CPC_GCB_OFS + 0x10)
BUILD_CPC_RW(resetlen,	MIPS_CPC_GCB_OFS + 0x18)
BUILD_CPC_R_(revision,	MIPS_CPC_GCB_OFS + 0x20)

BUILD_CPC_Cx_RW(cmd,		0x00)
BUILD_CPC_Cx_RW(stat_conf,	0x08)
BUILD_CPC_Cx_RW(other,		0x10)

#define CPC_Cx_OTHER_CORENUM_SHF	16
#define CPC_Cx_OTHER_CORENUM_MSK	(0xff << 16)

#define CPC_Cx_CMD_SHF			0
#define CPC_Cx_CMD_MSK			(0xf << 0)
#define  CPC_Cx_CMD_CLOCKOFF		1
#define  CPC_Cx_CMD_PWRDOWN		2
#define  CPC_Cx_CMD_PWRUP		3
#define  CPC_Cx_CMD_RESET		4

#define CPC_Cx_STAT_CONF_SEQSTATE_SHF	19
#define CPC_Cx_STAT_CONF_SEQSTATE_MSK	(0xf << 19)
#define  CPC_Cx_STAT_CONF_SEQSTATE_U6	(0x7 << 19)

/* VPE and TC stuff */

#define MVPCONTROL_EVP		(0x01)
#define MVPCONTROL_VPC		(0x01 << 1)
#define TCSTATUS_IXMT		(0x01 << 10)
#define TCSTATUS_A		(0x01 << 13)
#define VPECONF0_VPA		(0x01)

#define VPECONTROL_TARGTC	(0xff)

#define __read_32bit_c0_register(source, sel)                           \
({ int __res;                                                           \
        if (sel == 0)                                                   \
                __asm__ __volatile__(                                   \
                        "mfc0\t%0, " #source "\n\t"                     \
                        : "=r" (__res));                                \
        else                                                            \
                __asm__ __volatile__(                                   \
                        ".set\tmips32\n\t"                              \
                        "mfc0\t%0, " #source ", " #sel "\n\t"           \
                        ".set\tmips0\n\t"                               \
                        : "=r" (__res));                                \
        __res;                                                          \
})

#define __write_32bit_c0_register(register, sel, value)                 \
do {                                                                    \
        if (sel == 0)                                                   \
                __asm__ __volatile__(                                   \
                        "mtc0\t%z0, " #register "\n\t"                  \
                        : : "Jr" ((unsigned int)(value)));              \
        else                                                            \
                __asm__ __volatile__(                                   \
                        ".set\tmips32\n\t"                              \
                        "mtc0\t%z0, " #register ", " #sel "\n\t"        \
                        ".set\tmips0"                                   \
                        : : "Jr" ((unsigned int)(value)));              \
} while (0)

#define mftc0(rt,sel)                                                   \
({                                                                      \
         unsigned long  __res;                                          \
                                                                        \
        __asm__ __volatile__(                                           \
        "       .set    push                                    \n"     \
        "       .set    mips32r2                                \n"     \
        "       .set    noat                                    \n"     \
        "       # mftc0 $1, $" #rt ", " #sel "                  \n"     \
        "       .word   0x41000800 | (" #rt " << 16) | " #sel " \n"     \
        "       move    %0, $1                                  \n"     \
        "       .set    pop                                     \n"     \
        : "=r" (__res));                                                \
                                                                        \
        __res;                                                          \
})

#define mttc0(rd, sel, v)                                                       \
({                                                                      \
        __asm__ __volatile__(                                           \
        "       .set    push                                    \n"     \
        "       .set    mips32r2                                \n"     \
        "       .set    noat                                    \n"     \
        "       move    $1, %0                                  \n"     \
        "       # mttc0 %0," #rd ", " #sel "                    \n"     \
        "       .word   0x41810000 | (" #rd " << 11) | " #sel " \n"     \
        "       .set    pop                                     \n"     \
        :                                                               \
        : "r" (v));                                                     \
})

static inline void ehb(void)
{
        __asm__ __volatile__(
        "       .set    mips32r2                                \n"
        "       ehb                                             \n"
        "       .set    mips0                                   \n");
}

extern void __synci(void);

static inline void synci(void)
{
	__asm__ __volatile__(
	"	.set	mips32r2				\n"
	"	synci	0($0)					\n"
	"	nop						\n"
	"	ehb						\n"
	"	nop						\n"
	"	.set	mips0					\n");
}

#define read_c0_vpecontrol()            __read_32bit_c0_register($1, 1)
#define write_c0_vpecontrol(val)        __write_32bit_c0_register($1, 1, val)

#define read_c0_vpeconf0()		__read_32bit_c0_register($1, 2)

#define read_c0_mvpcontrol()            __read_32bit_c0_register($0, 1)
#define write_c0_mvpcontrol(val)        __write_32bit_c0_register($0, 1, val)

#define write_tc_c0_tcrestart(val)      mttc0(2, 3, val)

#define read_tc_c0_tcstatus()           mftc0(2, 1)
#define write_tc_c0_tcstatus(val)       mttc0(2, 1, val)

#define write_tc_c0_tchalt(val)         mttc0(2, 4, val)

#define read_vpe_c0_vpeconf0()          mftc0(1, 2)
#define write_vpe_c0_vpeconf0(val)      mttc0(1, 2, val)

#define __BUILD_SET_C0(name)                                    \
static inline unsigned int                                      \
set_c0_##name(unsigned int set)                                 \
{                                                               \
        unsigned int res, new;                                  \
                                                                \
        res = read_c0_##name();                                 \
        new = res | set;                                        \
        write_c0_##name(new);                                   \
                                                                \
        return res;                                             \
}                                                               \
                                                                \
static inline unsigned int                                      \
clear_c0_##name(unsigned int clear)                             \
{                                                               \
        unsigned int res, new;                                  \
                                                                \
        res = read_c0_##name();                                 \
        new = res & ~clear;                                     \
        write_c0_##name(new);                                   \
                                                                \
        return res;                                             \
}                                                               \
                                                                \
static inline unsigned int                                      \
change_c0_##name(unsigned int change, unsigned int val)         \
{                                                               \
        unsigned int res, new;                                  \
                                                                \
        res = read_c0_##name();                                 \
        new = res & ~change;                                    \
        new |= (val & change);                                  \
        write_c0_##name(new);                                   \
                                                                \
        return res;                                             \
}

__BUILD_SET_C0(mvpcontrol)

#define settc(tc)                                                       \
do {                                                                    \
        write_c0_vpecontrol((read_c0_vpecontrol()&~VPECONTROL_TARGTC) | (tc)); \
        ehb();                                                          \
} while (0)

#define instruction_hazard()                                            \
do {                                                                    \
        unsigned long tmp;                                              \
                                                                        \
        __asm__ __volatile__(                                           \
        "       .set    mips64r2                                \n"     \
        "       dla     %0, 1f                                  \n"     \
        "       jr.hb   %0                                      \n"     \
        "       .set    mips0                                   \n"     \
        "1:                                                     \n"     \
        : "=r" (tmp));                                                  \
} while (0)

static inline unsigned int dvpe(void)
{
        int res = 0;

        __asm__ __volatile__(
        "       .set    push                                            \n"
        "       .set    noreorder                                       \n"
        "       .set    noat                                            \n"
        "       .set    mips32r2                                        \n"
        "       .word   0x41610001              # dvpe $1               \n"
        "       move    %0, $1                                          \n"
        "       ehb                                                     \n"
        "       .set    pop                                             \n"
        : "=r" (res));

        instruction_hazard();

        return res;
}

static inline void __raw_evpe(void)
{
        __asm__ __volatile__(
        "       .set    push                                            \n"
        "       .set    noreorder                                       \n"
        "       .set    noat                                            \n"
        "       .set    mips32r2                                        \n"
        "       .word   0x41600021              # evpe                  \n"
        "       ehb                                                     \n"
        "       .set    pop                                             \n");
}

#define EVPE_ENABLE MVPCONTROL_EVP

static inline void evpe(int previous)
{
        if ((previous & MVPCONTROL_EVP))
                __raw_evpe();
}

#endif /* __MIPS_CM_H__ */
