/* drivers/gpu/mali400/mali/platform/pegasus-m400/exynos4_pmm.c
 *
 * Copyright 2011 by S.LSI. Samsung Electronics Inc.
 * San#24, Nongseo-Dong, Giheung-Gu, Yongin, Korea
 *
 * Samsung SoC Mali400 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file exynos4_pmm.c
 * Platform specific Mali driver functions for the exynos 4XXX based platforms
 */

#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "exynos4_pmm.h"
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_PM_RUNTIME)
#include <plat/pd.h>
#endif

#include <asm/io.h>
#include <mach/regs-pmu.h>

#include <linux/workqueue.h>

#define MALI_DVFS_STEPS 5
#define MAX_MALI_DVFS_STEPS 5
#define MALI_DVFS_WATING 10 /* msec */
#define MALI_DVFS_DEFAULT_STEP 0
#define MALI_DVFS_CLK_DEBUG 0

#ifdef CONFIG_CPU_FREQ
#include <mach/asv.h>
#define EXYNOS4_ASV_ENABLED
#endif

#include <plat/cpu.h>

#define CHIPID_REG		(S5P_VA_CHIPID + 0x4)

static int bMaliDvfsRun = 0;
static _mali_osk_atomic_t bottomlock_status;
static int bottom_lock_step;

typedef struct mali_dvfs_tableTag{
	unsigned int clock;
	unsigned int freq;
	unsigned int vol;
}mali_dvfs_table;

typedef struct mali_dvfs_statusTag{
	unsigned int currentStep;
	mali_dvfs_table * pCurrentDvfs;

}mali_dvfs_currentstatus;

typedef struct mali_dvfs_thresholdTag{
	unsigned int downthreshold;
	unsigned int upthreshold;
}mali_dvfs_threshold_table;

typedef struct mali_dvfs_staycount{
	unsigned int staycount;
}mali_dvfs_staycount_table;

typedef struct mali_dvfs_stepTag{
	int clk;
	int vol;
}mali_dvfs_step;

mali_dvfs_step step[MALI_DVFS_STEPS]={
	/*step 0 clk*/ {108,   950000},
	/*step 1 clk*/ {160,   950000},
	/*step 2 clk*/ {200,  1000000},
	/*step 3 clk*/ {266,  1050000},
	/*step 4 clk*/ {266,  1050000}
};

mali_dvfs_staycount_table mali_dvfs_staycount[MALI_DVFS_STEPS]={
	/*step 0*/{0},
	/*step 1*/{0},
	/*step 2*/{0},
	/*step 3*/{0},
	/*step 4*/{0}
};

int step0_clk = 108;
int step0_vol = 950000;
int step1_clk = 160;
int step1_vol = 950000;
int step0_up = 60;
int step1_down = 50;
int step2_clk = 200;
int step2_vol = 1000000;
int step1_up = 60;
int step2_down = 50;
int step3_clk = 266;
int step3_vol = 1050000;
int step2_up = 85;
int step3_down = 50;
int step4_clk = 266;
int step4_vol = 1050000;
int step3_up = 85;
int step4_down = 70;

mali_dvfs_table mali_dvfs_all[MAX_MALI_DVFS_STEPS]={
	{108   ,1000000   ,  950000},
	{160   ,1000000   ,  950000},
	{200   ,1000000   , 1000000},
	{266   ,1000000   , 1050000},
	{266   ,1000000   , 1050000} };

mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS]={
	{108  ,1000000    , 950000},
	{160  ,1000000    , 950000},
	{200  ,1000000    ,1000000},
	{266  ,1000000    ,1050000},
	{266  ,1000000    ,1050000}
};

mali_dvfs_threshold_table mali_dvfs_threshold[MALI_DVFS_STEPS]={
	{0   , 40},
	{50  , 60},
	{50  , 85},
	{50  , 85},
	{70  ,100}
};

/* dvfs status */
mali_dvfs_currentstatus maliDvfsStatus;
int mali_dvfs_control = 0;

typedef struct mali_runtime_resumeTag{
	int clk;
	int vol;
	unsigned int step;
}mali_runtime_resume_table;

mali_runtime_resume_table mali_runtime_resume = {160, 950000, 1};

#ifdef EXYNOS4_ASV_ENABLED
#define ASV_LEVEL	12	/* ASV0, 1, 11 is reserved */
#define ASV_LEVEL_PRIME	13	/* ASV0, 1, 12 is reserved */
#define ASV_LEVEL_PD	13
#define ASV_8_LEVEL	8
#define ASV_5_LEVEL	5

static unsigned int asv_3d_volt_5_table[ASV_5_LEVEL][MALI_DVFS_STEPS] = {
	/* L4(108MHz), L3(160MHz), L2(266MHz), L1(330MHz) */
	{ 950000, 1000000, 1100000, 1150000, 1200000},	/* S */
	{ 950000, 1000000, 1100000, 1150000, 1200000},	/* A */
	{ 900000,  950000, 1000000, 1100000, 1200000},	/* B */
	{ 900000,  950000, 1000000, 1050000, 1150000},	/* C */
	{ 900000,  950000,  950000, 1000000, 1100000},	/* D */
};

static unsigned int asv_3d_volt_8_table[ASV_8_LEVEL][MALI_DVFS_STEPS] = {
	/* L4(100MHz), L3(160MHz), L2(266MHz)), L1(330MHz) */
	{ 950000, 1000000, 1100000, 1150000, 1200000},	/* SS */
	{ 950000, 1000000, 1100000, 1150000, 1200000},	/* A1 */
	{ 950000, 1000000, 1100000, 1150000, 1200000},	/* A2 */
	{ 900000,  950000, 1000000, 1100000, 1200000},	/* B1 */
	{ 900000,  950000, 1000000, 1100000, 1200000},	/* B2 */
	{ 900000,  950000, 1000000, 1050000, 1150000},	/* C1 */
	{ 900000,  950000, 1000000, 1050000, 1150000},	/* C2 */
	{ 900000,  950000,  950000, 1000000, 1100000},	/* D1 */
};
#endif /* EXYNOS4_ASV_ENABLED */

#define EXTXTALCLK_NAME		"ext_xtal"
#define VPLLSRCCLK_NAME		"vpll_src"
#define FOUTVPLLCLK_NAME	"fout_vpll"
#define SCLVPLLCLK_NAME		"sclk_vpll"
#define GPUMOUT1CLK_NAME	"mout_g3d1"

#define MPLLCLK_NAME		"mout_mpll"
#define GPUMOUT0CLK_NAME	"mout_g3d0"
#define GPUCLK_NAME		"sclk_g3d"
#define CLK_DIV_STAT_G3D	0x1003C62C
#define CLK_DESC		"clk-divider-status"

static struct clk *ext_xtal_clock	= NULL;
static struct clk *vpll_src_clock	= NULL;
static struct clk *fout_vpll_clock	= NULL;
static struct clk *sclk_vpll_clock	= NULL;

static struct clk *mpll_clock		= NULL;
static struct clk *mali_parent_clock	= NULL;
static struct clk  *mali_mout0_clock	= NULL;
static struct clk *mali_clock		= NULL;

/* Orion */
extern int mali_use_vpll;

int mali_gpu_clk = 108;
int mali_gpu_vol = 950000;

static unsigned int GPU_MHZ	= 1000000;

int gpu_power_state;
static int bPoweroff;
atomic_t clk_active;

#if MALI_VOLTAGE_LOCK
int mali_lock_vol = 0;
static _mali_osk_atomic_t voltage_lock_status;
static mali_bool mali_vol_lock_flag = 0;
#endif

/* Declare for sysfs */
#ifdef CONFIG_MALI_DVFS
module_param(mali_dvfs_control, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP| S_IROTH); /* rw-rw-r-- */
MODULE_PARM_DESC(mali_dvfs_control, "Mali Current DVFS");
#endif

module_param(mali_gpu_clk, int, S_IRUSR | S_IRGRP | S_IROTH); /* r--r--r-- */
MODULE_PARM_DESC(mali_gpu_clk, "Mali Current Clock");

module_param(mali_gpu_vol, int, S_IRUSR | S_IRGRP | S_IROTH); /* r--r--r-- */
MODULE_PARM_DESC(mali_gpu_vol, "Mali Current Voltage");

module_param(gpu_power_state, int, S_IRUSR | S_IRGRP | S_IROTH); /* r--r--r-- */
MODULE_PARM_DESC(gpu_power_state, "Mali Power State");

#ifdef CONFIG_REGULATOR
struct regulator *g3d_regulator = NULL;
#endif

mali_io_address clk_register_map = 0;

/* DVFS */
unsigned int mali_dvfs_utilization = 255;
u64 mali_dvfs_time[MALI_DVFS_STEPS];
static void mali_dvfs_work_handler(struct work_struct *w);
static struct workqueue_struct *mali_dvfs_wq = 0;
extern mali_io_address clk_register_map;
_mali_osk_lock_t *mali_dvfs_lock = 0;
int mali_runtime_resumed = -1;
static DECLARE_WORK(mali_dvfs_work, mali_dvfs_work_handler);

#ifdef CONFIG_REGULATOR
void mali_regulator_disable(void)
{
	if (IS_ERR_OR_NULL(g3d_regulator)) {
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_disable : g3d_regulator is null\n"));
		return;
	}
	regulator_disable(g3d_regulator);
	bPoweroff = 1;
}

void mali_regulator_enable(void)
{
	if (IS_ERR_OR_NULL(g3d_regulator)) {
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_enable : g3d_regulator is null\n"));
		return;
	}
	regulator_enable(g3d_regulator);
	bPoweroff = 0;
}

void mali_regulator_set_voltage(int min_uV, int max_uV)
{
	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);

	if (IS_ERR_OR_NULL(g3d_regulator)) {
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_set_voltage : g3d_regulator is null\n"));
		_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
		return;
	}
	MALI_DEBUG_PRINT(1, ("= regulator_set_voltage: %d, %d \n",min_uV, max_uV));
	regulator_set_voltage(g3d_regulator, min_uV, max_uV);
	mali_gpu_vol = regulator_get_voltage(g3d_regulator);
	MALI_DEBUG_PRINT(1, ("Mali voltage: %d\n", mali_gpu_vol));
	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
}
#endif

unsigned long mali_clk_get_rate(void)
{
	return clk_get_rate(mali_clock);
}

static unsigned int get_mali_dvfs_status(void)
{
	return maliDvfsStatus.currentStep;
}

mali_bool mali_clk_get(mali_bool bis_vpll)
{
	if (bis_vpll == MALI_TRUE) {
		if (ext_xtal_clock == NULL) {
			ext_xtal_clock = clk_get(NULL, EXTXTALCLK_NAME);
			if (IS_ERR(ext_xtal_clock)) {
				MALI_PRINT(("MALI Error : failed to get source ext_xtal_clock\n"));
				return MALI_FALSE;
			}
			clk_enable(ext_xtal_clock);
		}

		if (vpll_src_clock == NULL) {
			vpll_src_clock = clk_get(NULL, VPLLSRCCLK_NAME);
			if (IS_ERR(vpll_src_clock)) {
				MALI_PRINT(("MALI Error : failed to get source vpll_src_clock\n"));
				return MALI_FALSE;
			}
			clk_enable(vpll_src_clock);
		}

		if (fout_vpll_clock == NULL) {
			fout_vpll_clock = clk_get(NULL, FOUTVPLLCLK_NAME);
			if (IS_ERR(fout_vpll_clock)) {
				MALI_PRINT(("MALI Error : failed to get source fout_vpll_clock\n"));
				return MALI_FALSE;
			}
			clk_enable(fout_vpll_clock);
		}

		if (sclk_vpll_clock == NULL) {
			sclk_vpll_clock = clk_get(NULL, SCLVPLLCLK_NAME);
			if (IS_ERR(sclk_vpll_clock)) {
				MALI_PRINT(("MALI Error : failed to get source sclk_vpll_clock\n"));
				return MALI_FALSE;
			}
			clk_enable(sclk_vpll_clock);
		}

		if (mali_parent_clock == NULL) {
			mali_parent_clock = clk_get(NULL, GPUMOUT1CLK_NAME);

			if (IS_ERR(mali_parent_clock)) {
				MALI_PRINT(( "MALI Error : failed to get source mali parent clock\n"));
				return MALI_FALSE;
			}
			clk_enable(mali_parent_clock);
		}

		if (mali_mout0_clock == NULL) {
			mali_mout0_clock = clk_get(NULL, GPUMOUT0CLK_NAME);

			if (IS_ERR(mali_mout0_clock)) {
				MALI_PRINT( ( "MALI Error : failed to get source mali mout0 clock\n"));
				return MALI_FALSE;
			}
		}
	/* mpll */
	} else {
		if (mpll_clock == NULL) {
			mpll_clock = clk_get(NULL, MPLLCLK_NAME);

			if (IS_ERR(mpll_clock)) {
				MALI_PRINT(("MALI Error : failed to get source mpll clock\n"));
				return MALI_FALSE;
			}
		}

		if (mali_parent_clock == NULL) {
			mali_parent_clock = clk_get(NULL, GPUMOUT0CLK_NAME);

			if (IS_ERR(mali_parent_clock)) {
				MALI_PRINT(( "MALI Error : failed to get source mali parent clock\n"));
				return MALI_FALSE;
			}
		}
	}

	/* mali clock get always. */
	if (mali_clock == NULL) {
		mali_clock = clk_get(NULL, GPUCLK_NAME);

		if (IS_ERR(mali_clock)) {
			MALI_PRINT(("MALI Error : failed to get source mali clock\n"));
			return MALI_FALSE;
		}
	}

	return MALI_TRUE;
}

void mali_clk_put(mali_bool binc_mali_clock)
{
	if (mali_parent_clock) {
		clk_put(mali_parent_clock);
		mali_parent_clock = NULL;
	}

	if (mali_mout0_clock) {
		clk_put(mali_mout0_clock);
		mali_mout0_clock = NULL;
	}

	if (mpll_clock) {
		clk_put(mpll_clock);
		mpll_clock = NULL;
	}

	if (sclk_vpll_clock) {
		clk_put(sclk_vpll_clock);
		sclk_vpll_clock = NULL;
	}

	if (binc_mali_clock && fout_vpll_clock) {
		clk_put(fout_vpll_clock);
		fout_vpll_clock = NULL;
	}

	if (vpll_src_clock) {
		clk_put(vpll_src_clock);
		vpll_src_clock = NULL;
	}

	if (ext_xtal_clock) {
		clk_put(ext_xtal_clock);
		ext_xtal_clock = NULL;
	}

	if (binc_mali_clock && mali_clock) {
		clk_put(mali_clock);
		mali_clock = NULL;
	}
}

mali_bool mali_clk_set_rate(unsigned int clk, unsigned int mhz)
{
	mali_bool bis_vpll = mali_use_vpll;

	int err;
	unsigned long rate = (unsigned long)clk * (unsigned long)mhz;

	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	MALI_DEBUG_PRINT(3, ("Mali platform: Setting frequency to %d mhz\n", clk));

	if (mali_clk_get(bis_vpll) == MALI_FALSE) {
		_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
		return MALI_FALSE;
	}

	if (bis_vpll) {
		clk_set_rate(fout_vpll_clock, (unsigned int)clk * GPU_MHZ);
		clk_set_parent(vpll_src_clock, ext_xtal_clock);
		clk_set_parent(sclk_vpll_clock, fout_vpll_clock);

		clk_set_parent(mali_parent_clock, sclk_vpll_clock);
		clk_set_parent(mali_clock, mali_parent_clock);
	} else {
		clk_set_parent(mali_parent_clock, mpll_clock);
		clk_set_parent(mali_clock, mali_parent_clock);
	}

	if (atomic_read(&clk_active) == 0) {
		if (clk_enable(mali_clock) < 0) {
			_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
			return MALI_FALSE;
		}
		atomic_set(&clk_active, 1);
	}

	err = clk_set_rate(mali_clock, rate);
	if (err > 0)
		MALI_PRINT_ERROR(("Failed to set Mali clock: %d\n", err));

	rate = mali_clk_get_rate();

	if (bis_vpll)
		mali_gpu_clk = (int)(rate / mhz);
	else
		mali_gpu_clk = (int)((rate + 500000) / mhz);

	MALI_DEBUG_PRINT(1, ("Mali frequency %d\n", rate / mhz));
	GPU_MHZ = mhz;

	mali_clk_put(MALI_FALSE);

	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);

	return MALI_TRUE;
}

int get_mali_dvfs_control_status(void)
{
	return mali_dvfs_control;
}

mali_bool set_mali_dvfs_current_step(unsigned int step)
{
	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	maliDvfsStatus.currentStep = step % MAX_MALI_DVFS_STEPS;
	if (step >= MAX_MALI_DVFS_STEPS)
		mali_runtime_resumed = maliDvfsStatus.currentStep;

	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	return MALI_TRUE;
}

static mali_bool set_mali_dvfs_status(u32 step,mali_bool boostup)
{
	u32 validatedStep=step;
	int err;

	if (boostup) {
#ifdef CONFIG_REGULATOR
		/* change the voltage */
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
		/* change the clock */
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
	} else {
		/* change the clock */
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
#ifdef CONFIG_REGULATOR
		/* change the voltage */
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
	}

	mali_clk_put(MALI_FALSE);

#if MALI_DVFS_CLK_DEBUG
	pRegMaliClkDiv = ioremap(0x1003c52c,32);
	pRegMaliMpll = ioremap(0x1003c22c,32);
	MALI_PRINT(("Mali MPLL reg:%d, CLK DIV: %d \n",*pRegMaliMpll, *pRegMaliClkDiv));
#endif

#ifdef EXYNOS4_ASV_ENABLED
	if (samsung_rev() < EXYNOS4412_REV_2_0) {
		if (mali_dvfs[step].clock == 160)
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_100V);
		else
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_130V);
	}
#endif

	set_mali_dvfs_current_step(validatedStep);
	/* for future use */
	maliDvfsStatus.pCurrentDvfs = &mali_dvfs[validatedStep];

	return MALI_TRUE;
}

static void mali_platform_wating(u32 msec)
{
	/*
	 * sample wating
	 * change this in the future with proper check routine.
	 */
	unsigned int read_val;
	while(1) {
		read_val = _mali_osk_mem_ioread32(clk_register_map, 0x00);
		if ((read_val & 0x8000)==0x0000) break;

		_mali_osk_time_ubusydelay(100); /* 1000 -> 100 : 20101218 */
	}
}

static mali_bool change_mali_dvfs_status(u32 step, mali_bool boostup )
{
	MALI_DEBUG_PRINT(4, ("> change_mali_dvfs_status: %d, %d \n",step, boostup));

	if (!set_mali_dvfs_status(step, boostup)) {
		MALI_DEBUG_PRINT(1, ("error on set_mali_dvfs_status: %d, %d \n",step, boostup));
		return MALI_FALSE;
	}

	/* wait until clock and voltage is stablized */
	mali_platform_wating(MALI_DVFS_WATING); /* msec */

	return MALI_TRUE;
}

#ifdef EXYNOS4_ASV_ENABLED
extern unsigned int exynos_result_of_asv;

static mali_bool mali_dvfs_table_update(void)
{
	unsigned int step_num = MALI_DVFS_STEPS;
	unsigned int i, exynos_result_of_asv_group, target_asv;

	exynos_result_of_asv_group = exynos_result_of_asv & 0xf;
	target_asv = exynos_result_of_asv >> 28;
	MALI_PRINT(("exynos_result_of_asv_group = 0x%x, target_asv = 0x%x\n", exynos_result_of_asv_group, target_asv));

	for (i = 0; i < step_num; i++) {
		if (target_asv == 0x8) { //SUPPORT_1400MHZ
			mali_dvfs[i].vol = asv_3d_volt_5_table[i][exynos_result_of_asv_group];
		} else if (target_asv == 0x4){ //SUPPORT_1200MHZ
			mali_dvfs[i].vol = asv_3d_volt_8_table[i][exynos_result_of_asv_group];
		}
		MALI_PRINT(("mali_dvfs[%d].vol = %d \n", i, mali_dvfs[i].vol));

		/* Update voltage using for resume */
		if (mali_runtime_resume.clk == mali_dvfs[i].clock) {
			mali_runtime_resume.vol = mali_dvfs[i].vol;
			MALI_PRINT(("mali_runtime_resume.vol = %d \n", mali_runtime_resume.vol));
		}

		/* update voltage using for init timing */
		if (mali_gpu_clk == mali_dvfs[i].clock) {
			mali_gpu_vol = mali_dvfs[i].vol;
			MALI_PRINT(("init_gpu_vol = %d \n", mali_gpu_vol));
		}
	}
	return MALI_TRUE;
}
#endif

static unsigned int decideNextStatus(unsigned int utilization)
{
	static unsigned int level = 0;
	static int mali_dvfs_clk = 0;

	if (mali_runtime_resumed >= 0) {
		level = mali_runtime_resumed;
		mali_runtime_resumed = -1;
		return level;
	}

	if (mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold
			<= mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold) {
		MALI_PRINT(("upthreadshold is smaller than downthreshold: %d < %d\n",
				mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold,
				mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold));
		return level;
	}

	if (!mali_dvfs_control && level == maliDvfsStatus.currentStep) {
		if (utilization > (int)(255 * mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold / 100) &&
				level < MALI_DVFS_STEPS - 1) {
			level++;
#if 0 /* this prevents the usage of 5th step -gm */
			if ((samsung_rev() < EXYNOS4412_REV_2_0) && (maliDvfsStatus.currentStep == 3)) {
				level=get_mali_dvfs_status();
			}
#endif
		}
		if (utilization < (int)(255 * mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold / 100) &&
				level > 0) {
			level--;
		}
	} else if (mali_dvfs_control == 999) {
		int i = 0;
		for (i = 0; i < MALI_DVFS_STEPS; i++) {
			step[i].clk = mali_dvfs_all[i].clock;
		}
		i = 0;
		for (i = 0; i < MALI_DVFS_STEPS; i++) {
			mali_dvfs[i].clock = step[i].clk;
		}
		mali_dvfs_control = 0;
		level = 0;

		step0_clk = step[0].clk;
		change_dvfs_tableset(step0_clk, 0);
		step1_clk = step[1].clk;
		change_dvfs_tableset(step1_clk, 1);
		step2_clk = step[2].clk;
		change_dvfs_tableset(step2_clk, 2);
		step3_clk = step[3].clk;
		change_dvfs_tableset(step3_clk, 3);
		step4_clk = step[4].clk;
		change_dvfs_tableset(step4_clk, 4);
	} else if (mali_dvfs_control != mali_dvfs_clk && mali_dvfs_control != 999) {
		if (mali_dvfs_control < mali_dvfs_all[1].clock && mali_dvfs_control > 0) {
			int i = 0;
			for (i = 0; i < MALI_DVFS_STEPS; i++) {
				step[i].clk = mali_dvfs_all[0].clock;
			}
			maliDvfsStatus.currentStep = 0;
		} else if (mali_dvfs_control < mali_dvfs_all[2].clock && mali_dvfs_control >= mali_dvfs_all[1].clock) {
			int i = 0;
			for (i = 0; i < MALI_DVFS_STEPS; i++) {
				step[i].clk = mali_dvfs_all[1].clock;
			}
			maliDvfsStatus.currentStep = 1;
		} else if (mali_dvfs_control < mali_dvfs_all[3].clock && mali_dvfs_control >= mali_dvfs_all[2].clock) {
			int i = 0;
			for (i = 0; i < MALI_DVFS_STEPS; i++) {
				step[i].clk = mali_dvfs_all[2].clock;
			}
			maliDvfsStatus.currentStep = 2;
		} else if (mali_dvfs_control < mali_dvfs_all[4].clock && mali_dvfs_control >= mali_dvfs_all[3].clock) {
			int i = 0;
			for (i = 0; i < MALI_DVFS_STEPS; i++) {
				step[i].clk  = mali_dvfs_all[3].clock;
			}
			maliDvfsStatus.currentStep = 3;
		} else {
			int i = 0;
			for (i = 0; i < MALI_DVFS_STEPS; i++) {
				step[i].clk  = mali_dvfs_all[4].clock;
			}
			maliDvfsStatus.currentStep = 4;
		}
		step0_clk = step[0].clk;
		change_dvfs_tableset(step0_clk, 0);
		step1_clk = step[1].clk;
		change_dvfs_tableset(step1_clk, 1);
		step2_clk = step[2].clk;
		change_dvfs_tableset(step2_clk, 2);
		step3_clk = step[3].clk;
		change_dvfs_tableset(step3_clk, 3);
		step4_clk = step[4].clk;
		change_dvfs_tableset(step4_clk, 4);
		level = maliDvfsStatus.currentStep;
	}

	mali_dvfs_clk = mali_dvfs_control;

	if (_mali_osk_atomic_read(&bottomlock_status) > 0) {
		if (level < bottom_lock_step)
			level = bottom_lock_step;
	}

	return level;
}

static mali_bool mali_dvfs_status(unsigned int utilization)
{
	unsigned int nextStatus = 0;
	unsigned int curStatus = 0;
	mali_bool boostup = MALI_FALSE;
	static int stay_count = 0; /* to prevent frequent switch */

	MALI_DEBUG_PRINT(4, ("> mali_dvfs_status: %d \n",utilization));

	/* decide next step */
	curStatus = get_mali_dvfs_status();
	nextStatus = decideNextStatus(utilization);

	MALI_DEBUG_PRINT(4, ("= curStatus %d, nextStatus %d, maliDvfsStatus.currentStep %d \n", curStatus, nextStatus, maliDvfsStatus.currentStep));
	/* if next status is same with current status, don't change anything */
	if ((curStatus != nextStatus && stay_count == 0)) {
		/*check if boost up or not*/
		if (nextStatus > maliDvfsStatus.currentStep) boostup = 1;

		/* change mali dvfs status */
		if (!change_mali_dvfs_status(nextStatus,boostup)) {
			MALI_DEBUG_PRINT(1, ("error on change_mali_dvfs_status \n"));
			return MALI_FALSE;
		}
		stay_count = mali_dvfs_staycount[maliDvfsStatus.currentStep].staycount;
	} else {
		if (stay_count > 0)
			stay_count--;
	}

	return MALI_TRUE;
}

int mali_dvfs_is_running(void)
{
	return bMaliDvfsRun;
}

void mali_dvfs_late_resume(void)
{
	/* set the init clock as low when resume */
	set_mali_dvfs_status(0, 0);
}

static void mali_dvfs_work_handler(struct work_struct *w)
{
	int change_clk = 0;
	int change_step = 0;
	bMaliDvfsRun = 1;

	/* dvfs table change when clock was changed */
	if (step0_clk != mali_dvfs[0].clock) {
		MALI_PRINT(("::: step0_clk change to %d Mhz\n", step0_clk));
		change_clk = step0_clk;
		change_step = 0;
		step0_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step1_clk != mali_dvfs[1].clock) {
		MALI_PRINT(("::: step1_clk change to %d Mhz\n", step1_clk));
		change_clk = step1_clk;
		change_step = 1;
		step1_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step0_up != mali_dvfs_threshold[0].upthreshold) {
		MALI_PRINT(("::: step0_up change to %d %\n", step0_up));
		mali_dvfs_threshold[0].upthreshold = step0_up;
	}
	if (step1_down != mali_dvfs_threshold[1].downthreshold) {
		MALI_PRINT((":::step1_down change to %d %\n", step1_down));
		mali_dvfs_threshold[1].downthreshold = step1_down;
	}
	if (step2_clk != mali_dvfs[2].clock) {
		MALI_PRINT(("::: step2_clk change to %d Mhz\n", step2_clk));
		change_clk = step2_clk;
		change_step = 2;
		step2_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step1_up != mali_dvfs_threshold[1].upthreshold) {
		MALI_PRINT((":::step1_up change to %d %\n", step1_up));
		mali_dvfs_threshold[1].upthreshold = step1_up;
	}
	if (step2_down != mali_dvfs_threshold[2].downthreshold) {
		MALI_PRINT((":::step2_down change to %d %\n", step2_down));
		mali_dvfs_threshold[2].downthreshold = step2_down;
	}
	if (step3_clk != mali_dvfs[3].clock) {
		MALI_PRINT(("::: step3_clk change to %d Mhz\n", step3_clk));
		change_clk = step3_clk;
		change_step = 3;
		step3_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step2_up != mali_dvfs_threshold[2].upthreshold) {
		MALI_PRINT((":::step2_up change to %d %\n", step2_up));
		mali_dvfs_threshold[2].upthreshold = step2_up;
	}
	if (step3_down != mali_dvfs_threshold[3].downthreshold) {
		MALI_PRINT((":::step3_down change to %d %\n", step3_down));
		mali_dvfs_threshold[3].downthreshold = step3_down;
	}
	if (step4_clk != mali_dvfs[4].clock) {
		MALI_PRINT(("::: step4_clk change to %d Mhz\n", step4_clk));
		change_clk = step4_clk;
		change_step = 4;
		step4_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step3_up != mali_dvfs_threshold[3].upthreshold) {
		MALI_PRINT((":::step3_up change to %d %\n", step3_up));
		mali_dvfs_threshold[3].upthreshold = step3_up;
	}
	if (step4_down != mali_dvfs_threshold[4].downthreshold) {
		MALI_PRINT((":::step4_down change to %d %\n", step4_down));
		mali_dvfs_threshold[4].downthreshold = step4_down;
	}

#ifdef DEBUG
	mali_dvfs[0].vol = step0_vol;
	mali_dvfs[1].vol = step1_vol;
	mali_dvfs[2].vol = step2_vol;
	mali_dvfs[3].vol = step3_vol;
	mali_dvfs[4].vol = step4_vol;
#endif
	MALI_DEBUG_PRINT(3, ("=== mali_dvfs_work_handler\n"));

	if (!mali_dvfs_status(mali_dvfs_utilization))
		MALI_DEBUG_PRINT(1,( "error on mali dvfs status in mali_dvfs_work_handler"));

	bMaliDvfsRun = 0;
}

mali_bool init_mali_dvfs_status(int step)
{
	/*
	 * default status
	 * add here with the right function to get initilization value.
	 */

	if (!mali_dvfs_wq)
		mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");

	_mali_osk_atomic_init(&bottomlock_status, 0);

	mali_dvfs_table_update();

	/* add a error handling here */
	set_mali_dvfs_current_step(step);

	return MALI_TRUE;
}

void deinit_mali_dvfs_status(void)
{
	if (mali_dvfs_wq)
		destroy_workqueue(mali_dvfs_wq);

	_mali_osk_atomic_term(&bottomlock_status);

	mali_dvfs_wq = NULL;
}

mali_bool mali_dvfs_handler(unsigned int utilization)
{
	mali_dvfs_utilization = utilization;
	queue_work_on(0, mali_dvfs_wq, &mali_dvfs_work);

	/*add error handle here*/
	return MALI_TRUE;
}

static mali_bool init_mali_clock(void)
{
	mali_bool ret = MALI_TRUE;
	mali_bool bis_vpll = mali_use_vpll;

	if (mali_clock != 0)
		return ret; /* already initialized */

	mali_dvfs_lock = _mali_osk_lock_init(_MALI_OSK_LOCKFLAG_NONINTERRUPTABLE
			| _MALI_OSK_LOCKFLAG_ONELOCK, 0, 0);
	if (mali_dvfs_lock == NULL)
		return _MALI_OSK_ERR_FAULT;

	if (!mali_clk_get(bis_vpll)) {
		MALI_PRINT(("Error: Failed to get Mali clock\n"));
		ret = MALI_FALSE;
		goto err_clk;
	}

	mali_clk_set_rate((unsigned int)mali_gpu_clk, GPU_MHZ);

	MALI_PRINT(("init_mali_clock mali_clock %x\n", mali_clock));

#ifdef CONFIG_REGULATOR
	g3d_regulator = regulator_get(NULL, "vdd_g3d");

	if (IS_ERR(g3d_regulator)) {
		MALI_PRINT(("MALI Error : failed to get vdd_g3d\n"));
		ret = MALI_FALSE;
		goto err_regulator;
	}

	regulator_enable(g3d_regulator);
	mali_regulator_set_voltage(mali_gpu_vol, mali_gpu_vol);

#ifdef EXYNOS4_ASV_ENABLED
	if (samsung_rev() < EXYNOS4412_REV_2_0) {
		if (mali_gpu_clk == 160)
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_100V);
		else
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_130V);
	}
#endif
#endif

	mali_clk_put(MALI_FALSE);

	gpu_power_state = 0;
	bPoweroff = 1;

	return MALI_TRUE;

#ifdef CONFIG_REGULATOR
err_regulator:
	regulator_put(g3d_regulator);
#endif
err_clk:
	mali_clk_put(MALI_TRUE);

	return ret;
}

static mali_bool deinit_mali_clock(void)
{
	if (mali_clock == 0)
		return MALI_TRUE;

#ifdef CONFIG_REGULATOR
	if (g3d_regulator) {
		regulator_put(g3d_regulator);
		g3d_regulator = NULL;
	}
#endif

	mali_clk_put(MALI_TRUE);

	return MALI_TRUE;
}

static _mali_osk_errcode_t enable_mali_clocks(void)
{
	int err;

	if (atomic_read(&clk_active) == 0) {
		err = clk_enable(mali_clock);
		MALI_DEBUG_PRINT(3,("enable_mali_clocks mali_clock %p error %d \n", mali_clock, err));
		atomic_set(&clk_active, 1);
	}

	mali_runtime_resume.vol = mali_dvfs_get_vol(MALI_DVFS_STEPS + 1);
	/* set clock rate */
#ifdef CONFIG_MALI_DVFS
	if (get_mali_dvfs_control_status() != 0 || mali_gpu_clk >= mali_runtime_resume.clk) {
		mali_clk_set_rate(mali_gpu_clk, GPU_MHZ);
	} else {
#ifdef CONFIG_REGULATOR
		mali_regulator_set_voltage(mali_runtime_resume.vol, mali_runtime_resume.vol);

#ifdef EXYNOS4_ASV_ENABLED
		if (samsung_rev() < EXYNOS4412_REV_2_0) {
			if (mali_runtime_resume.clk == 160)
				exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_100V);
			else
				exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_130V);
		}
#endif
#endif
		mali_clk_set_rate(mali_runtime_resume.clk, GPU_MHZ);
		set_mali_dvfs_current_step(mali_runtime_resume.step);
	}
#else
	mali_clk_set_rate((unsigned int)mali_gpu_clk, GPU_MHZ);
	maliDvfsStatus.currentStep = MALI_DVFS_DEFAULT_STEP;
#endif

	MALI_SUCCESS;
}

static _mali_osk_errcode_t disable_mali_clocks(void)
{
	if (atomic_read(&clk_active) == 1) {
		clk_disable(mali_clock);
		atomic_set(&clk_active, 0);
	}
	MALI_DEBUG_PRINT(3,("disable_mali_clocks mali_clock %p \n", mali_clock));

	MALI_SUCCESS;
}

/* Some defines changed names in later Odroid-A kernels. Make sure it works for both. */
#ifndef S5P_G3D_CONFIGURATION
#define S5P_G3D_CONFIGURATION S5P_PMU_G3D_CONF
#endif
#ifndef S5P_G3D_STATUS
#define S5P_G3D_STATUS S5P_PMU_G3D_CONF + 0x4
#endif

_mali_osk_errcode_t g3d_power_domain_control(int bpower_on)
{
	if (bpower_on) {
		void __iomem *status;
		u32 timeout;
		__raw_writel(S5P_INT_LOCAL_PWR_EN, S5P_G3D_CONFIGURATION);
		status = S5P_G3D_STATUS;

		timeout = 10;
		while ((__raw_readl(status) & S5P_INT_LOCAL_PWR_EN)
				!= S5P_INT_LOCAL_PWR_EN) {
			if (timeout == 0) {
				MALI_PRINTF(("Power domain  enable failed.\n"));
				return -ETIMEDOUT;
			}
			timeout--;
			_mali_osk_time_ubusydelay(100);
		}
	} else {
		void __iomem *status;
		u32 timeout;
		__raw_writel(0, S5P_G3D_CONFIGURATION);

		status = S5P_G3D_STATUS;
		/* Wait max 1ms */
		timeout = 10;
		while (__raw_readl(status) & S5P_INT_LOCAL_PWR_EN) {
			if (timeout == 0) {
				MALI_PRINTF(("Power domain  disable failed.\n" ));
				return -ETIMEDOUT;
			}
			timeout--;
			_mali_osk_time_ubusydelay(100);
		}
	}

	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_init(struct device *dev)
{
	MALI_CHECK(init_mali_clock(), _MALI_OSK_ERR_FAULT);

	atomic_set(&clk_active, 0);

#ifdef CONFIG_MALI_DVFS
	if (!clk_register_map) clk_register_map = _mali_osk_mem_mapioregion( CLK_DIV_STAT_G3D, 0x20, CLK_DESC );
	if (!init_mali_dvfs_status(MALI_DVFS_DEFAULT_STEP))
		MALI_DEBUG_PRINT(1, ("mali_platform_init failed\n"));
#endif

	mali_platform_power_mode_change(dev, MALI_POWER_MODE_ON);

	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_deinit(struct device *dev)
{

	mali_platform_power_mode_change(dev, MALI_POWER_MODE_DEEP_SLEEP);
	deinit_mali_clock();

#ifdef CONFIG_MALI_DVFS
	deinit_mali_dvfs_status();
	if (clk_register_map) {
		_mali_osk_mem_unmapioregion(CLK_DIV_STAT_G3D, 0x20, clk_register_map);
		clk_register_map = NULL;
	}
#endif

	MALI_SUCCESS;
}

void mali_force_mpll(void);
void mali_restore_vpll_mode(void);

_mali_osk_errcode_t mali_platform_power_mode_change(struct device *dev, mali_power_mode power_mode)
{
	switch (power_mode) {
		case MALI_POWER_MODE_ON:
			MALI_DEBUG_PRINT(3, ("Mali platform: Got MALI_POWER_MODE_ON event, %s\n",
								 bPoweroff ? "powering on" : "already on"));
			if (bPoweroff == 1) {
#if !defined(CONFIG_PM_RUNTIME)
				g3d_power_domain_control(1);
#endif
				MALI_DEBUG_PRINT(4,("enable clock \n"));
				enable_mali_clocks();
				mali_restore_vpll_mode();
				gpu_power_state = 1;
				bPoweroff = 0;
			}
			break;
		case MALI_POWER_MODE_LIGHT_SLEEP:
		case MALI_POWER_MODE_DEEP_SLEEP:
			MALI_DEBUG_PRINT(3, ("Mali platform: Got %s event, %s\n", power_mode ==
						MALI_POWER_MODE_LIGHT_SLEEP ?  "MALI_POWER_MODE_LIGHT_SLEEP" :
						"MALI_POWER_MODE_DEEP_SLEEP", bPoweroff ? "already off" : "powering off"));
			if (bPoweroff == 0) {
				mali_force_mpll();
				disable_mali_clocks();
#if !defined(CONFIG_PM_RUNTIME)
				g3d_power_domain_control(0);
#endif
				gpu_power_state = 0;
				bPoweroff = 1;
			}

			break;
	}
	MALI_SUCCESS;
}

void mali_gpu_utilization_handler(struct mali_gpu_utilization_data *data)
{
	if (bPoweroff == 0) {
#ifdef CONFIG_MALI_DVFS
		if (!mali_dvfs_handler(data->utilization_gpu))
			MALI_DEBUG_PRINT(1, ("error on mali dvfs status in utilization\n"));
#endif
	}
}

int change_dvfs_tableset(int change_clk, int change_step)
{
	int err;

	mali_dvfs[change_step].clock = change_clk;
	MALI_PRINT((":::mali dvfs step %d clock and voltage = %d Mhz, %d V\n",change_step, mali_dvfs[change_step].clock, mali_dvfs[change_step].vol));

	if (maliDvfsStatus.currentStep == change_step) {
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[change_step].vol, mali_dvfs[change_step].vol);
#endif
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[change_step].clock, mali_dvfs[change_step].freq);
	}

	return mali_dvfs[change_step].clock;
}

void mali_default_step_set(int step, mali_bool boostup)
{
	mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);

	if (maliDvfsStatus.currentStep == 1)
		set_mali_dvfs_status(step, boostup);
}

int mali_dvfs_bottom_lock_push(int lock_step)
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);

	if (prev_status < 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for push\n"));
		return -1;
	}
	/* not a bad idea to limit locking to 4th step, so let's leave this -gm */
	if (samsung_rev() < EXYNOS4412_REV_2_0)
		lock_step = min(lock_step, MALI_DVFS_STEPS - 2);
	else
		lock_step = min(lock_step, MALI_DVFS_STEPS - 1);

	if (bottom_lock_step < lock_step) {
		bottom_lock_step = lock_step;
		if (get_mali_dvfs_status() < lock_step) {
			mali_regulator_set_voltage(mali_dvfs[lock_step].vol,
						   mali_dvfs[lock_step].vol);
			mali_clk_set_rate(mali_dvfs[lock_step].clock,
					  mali_dvfs[lock_step].freq);
			set_mali_dvfs_current_step(lock_step);
		}
	}
	return _mali_osk_atomic_inc_return(&bottomlock_status);
}

int mali_dvfs_bottom_lock_pop(void)
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);

	if (prev_status <= 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for pop\n"));
		return -1;
	} else if (prev_status == 1) {
		bottom_lock_step = 0;
		MALI_PRINT(("gpu bottom lock release\n"));
	}

	return _mali_osk_atomic_dec_return(&bottomlock_status);
}

int mali_dvfs_get_vol(int step)
{
	step = step % MAX_MALI_DVFS_STEPS;
	MALI_DEBUG_ASSERT(step < MAX_MALI_DVFS_STEPS);

	return mali_dvfs[step].vol;
}

#if MALI_VOLTAGE_LOCK
int mali_voltage_lock_push(int lock_vol)
{
	int prev_status = _mali_osk_atomic_read(&voltage_lock_status);

	if (prev_status < 0) {
		MALI_PRINT(("gpu voltage lock status is not valid for push\n"));
		return -1;
	}
	if (prev_status == 0) {
		mali_lock_vol = lock_vol;
		if (mali_gpu_vol < mali_lock_vol)
			mali_regulator_set_voltage(mali_lock_vol, mali_lock_vol);
	} else {
		MALI_PRINT(("gpu voltage lock status is already pushed, current lock voltage : %d\n", mali_lock_vol));
		return -1;
	}

	return _mali_osk_atomic_inc_return(&voltage_lock_status);
}

int mali_voltage_lock_pop(void)
{
	if (_mali_osk_atomic_read(&voltage_lock_status) <= 0) {
		MALI_PRINT(("gpu voltage lock status is not valid for pop\n"));
		return -1;
	}
	return _mali_osk_atomic_dec_return(&voltage_lock_status);
}

int mali_voltage_lock_init(void)
{
	mali_vol_lock_flag = MALI_TRUE;

	MALI_SUCCESS;
}

int mali_use_vpll_save;
void mali_restore_vpll_mode(void)
{
	mali_use_vpll = mali_use_vpll_save;
}

void mali_force_mpll(void)
{
	mali_use_vpll_save = mali_use_vpll;
	mali_use_vpll = false;
	mali_regulator_set_voltage(mali_gpu_vol, mali_gpu_vol);
	mali_clk_set_rate((unsigned int)mali_gpu_clk, GPU_MHZ);
}

int mali_vol_get_from_table(int vol)
{
	int i;
	for (i = 0; i < MALI_DVFS_STEPS; i++) {
		if (mali_dvfs[i].vol >= vol)
			return mali_dvfs[i].vol;
	}
	MALI_PRINT(("Failed to get voltage from mali_dvfs table, maximum voltage is %d uV\n", mali_dvfs[MALI_DVFS_STEPS-1].vol));
	return 0;
}
#endif
