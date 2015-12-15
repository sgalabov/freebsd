#ifndef __SMP_CPS_H__
#define __SMP_CPS_H__

struct boot_config {
	uint32_t core;
	uint32_t vpe;
	uint32_t pc;
	uint32_t sp;
	uint32_t gp;
};

extern struct boot_config mips_cps_bootcfg;

#endif /* __SMP_CPS_H__ */
