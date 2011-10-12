/* arch/arm/mach-msm/clock.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <mach/msm_rpcrouter.h>

#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <mach/msm_iomap.h>
#include <mach/amss_para.h>
#include <asm/io.h>

#include "clock.h"
#include "clock-regime_amss.h"
#include "proc_comm.h"

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static LIST_HEAD(clocks);

enum {
	DEBUG_UNKNOWN_ID	= 1<<0,
	DEBUG_UNKNOWN_FREQ	= 1<<1,
	DEBUG_MDNS		= 1<<2,
	DEBUG_UNKNOWN_CMD	= 1<<3,
};
static int debug_mask=0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);


static int pc_clk_is_enabled(uint32_t id);
static int pc_clk_enable(uint32_t id);
static void pc_clk_disable(uint32_t id);

#if 1
#define D(x...) printk(KERN_DEBUG "clock-wince: " x)
#else
#define D(x...) do {} while (0)
#endif

struct mdns_clock_params
{
	unsigned long freq;
	uint32_t calc_freq;
	uint32_t md;
	uint32_t ns;
	uint32_t pll_freq;
	uint32_t clk_id;
};

struct msm_clock_params
{
	unsigned idx;
	unsigned offset;  // Offset points to .ns register
	bool setup_mdns;
	char	*name;
	struct   clk_ops *ops;
};

static int max_clk_rate[NR_CLKS], min_clk_rate[NR_CLKS];

#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#define TCX0			19200000 // Hz
#define PLL_FREQ(l, m, n)	(TCX0 * (l) + TCX0 * (m) / (n))

static unsigned int pll_get_rate(int n)
{
	unsigned int mode, L, M, N, freq;

 if (n == -1) return TCX0;
 if (n > 3)
  return 0;
 else
 {
	mode = readl(PLLn_BASE(n) + 0x0);
	L = readl(PLLn_BASE(n) + 0x4);
	M = readl(PLLn_BASE(n) + 0x8);
	N = readl(PLLn_BASE(n) + 0xc);
	freq = PLL_FREQ(L, M, N);
	printk(KERN_INFO "PLL%d: MODE=%08x L=%08x M=%08x N=%08x freq=%u Hz (%u MHz)\n",
		n, mode, L, M, N, freq, freq / 1000000); \
 }

 return freq;
}

static unsigned int idx2pll(uint32_t idx)
{
 int ret;

 switch(idx)
 {
  case 0: /* TCX0 */
   ret=-1;
  break;
  case 1: /* PLL1 */
   ret=1;
  break;
  case 4: /* PLL0 */
   ret=0;
  break;
  default:
   ret=4; /* invalid */
 }

 return ret;
}

struct clk_ops vfe_clk_ops = {
    .enable = msm_clk_regime_amss_enable,
    .disable = msm_clk_regime_amss_disable,
    .set_rate = msm_clk_regime_amss_set_rate,
    .set_flags = msm_clk_regime_amss_set_flags,
    .is_enabled = msm_clk_regime_amss_is_on,
};

struct clk_ops vdc_clk_ops = {
    .enable = msm_clk_regime_amss_enable,
    .disable = msm_clk_regime_amss_disable,
};

/* Note that some are not used
 * like, we use MicroP instead of GP CLK
 */
static struct msm_clock_params msm_clock_parameters_def[NR_CLKS] = {
	[ADSP_CLK] = {			.offset = 0x34,				.name="ADSP_CLK",},
	[GP_CLK] = { 			.offset = 0x5c,				.name="GP_CLK",},
	[GRP_CLK] = { .idx = 3,							.name="GRP_CLK",},
	[IMEM_CLK] = { .idx = 3,						.name="IMEM_CLK",},
	[I2C_CLK] = { 			.offset = 0x68,	.setup_mdns = 1,	.name="I2C_CLK",},
	[MDC_CLK] = {			.offset = 0x7c,				.name="MDC_CLK",	.ops=&vfe_clk_ops},
	[MDP_CLK] = { .idx = 9,							.name="MDP_CLK",},
	[PMDH_CLK] = {			.offset = 0x8c, .setup_mdns = 0,	.name="PMDH_CLK",},
	[SDAC_CLK] = { 			.offset = 0x9c,				.name="SDAC_CLK",},
	[SDC1_CLK] = { 			.offset = 0xa4,	.setup_mdns = 1,	.name="SDC1_CLK",},
	[SDC2_CLK] = { 			.offset = 0xac,	.setup_mdns = 1,	.name="SDC2_CLK",},
	[SDC3_CLK] = { 			.offset = 0xb4,	.setup_mdns = 1,	.name="SDC3_CLK",},
	[SDC4_CLK] = {			.offset = 0xbc,	.setup_mdns = 1,	.name="SDC4_CLK",},
	[SDC1_PCLK] = { .idx = 7,						.name="SDC1_PCLK",},
	[SDC2_PCLK] = { .idx = 8,						.name="SDC2_PCLK",},
	[SDC3_PCLK] = { .idx = 27,						.name="SDC3_PCLK",},
	[SDC4_PCLK] = { .idx = 28,						.name="SDC4_PCLK",},
	[UART1_CLK] = {			.offset = 0xe0,				.name="UART1_CLK",},
	[UART2_CLK] = {			.offset = 0xe0,				.name="UART2_CLK",},
	[UART3_CLK] = {			.offset = 0xe0,				.name="UART3_CLK",},
	[UART1DM_CLK] =	{ .idx = 17,	.offset = 0xd4,	.setup_mdns = 1,	.name="UART1DM_CLK",},
	[UART2DM_CLK] =	{ .idx = 26,	.offset = 0xdc,	.setup_mdns = 1,	.name="UART2DM_CLK",},
	[USB_HS_CLK] = {		.offset = 0x2c0,			.name="USB_HS_CLK",},
	[USB_HS_PCLK] =	{ .idx = 25,						.name="USB_HS_PCLK",},
	[VFE_CLK] = {			.offset = 0x44,	.setup_mdns = 1,	.name="VFE_CLK",	.ops=&vfe_clk_ops},
	[VFE_MDC_CLK] = {		.offset = 0x44,				.name="VFE_MDC_CLK",	.ops=&vfe_clk_ops},
	[VDC_CLK] = {			.offset = 0xf0,				.name="VDC_CLK",	.ops=&vdc_clk_ops},
};

static struct msm_clock_params* msm_clock_parameters;

// This formula is used to generate md and ns reg values
#define MSM_CLOCK_REG(frequency,M,N,D,PRE,a5,SRC,MNE,pll_frequency) { \
	.freq = (frequency), \
	.md = ((0xffff & (M)) << 16) | (0xffff & ~((D) << 1)), \
	.ns = ((0xffff & ~((N) - (M))) << 16) \
	    | ((0xff & (0xa | (MNE))) << 8) \
	    | ((0x7 & (a5)) << 5) \
	    | ((0x3 & (PRE)) << 3) \
	    | (0x7 & (SRC)), \
	.pll_freq = (pll_frequency), \
	.calc_freq = (pll_frequency*M/((PRE+1)*N)), \
}

static struct mdns_clock_params *msm_clock_freq_parameters;

// GSM phones typically use a 245 MHz PLL0
struct mdns_clock_params msm_clock_freq_parameters_pll0_245[] = {

	MSM_CLOCK_REG(  144000,   3, 0x64, 0x32, 3, 3, 0, 1, 19200000), /* SD, 144kHz */
#if 0 /* wince uses this clock setting for UART2DM */
	MSM_CLOCK_REG( 1843200,     3, 0x64, 0x32, 3, 2, 4, 1, 245760000), /*  115200*16=1843200 */
//	MSM_CLOCK_REG(            , 2, 0xc8, 0x64, 3, 2, 1, 1, 768888888), /* 1.92MHz for 120000 bps */
#else
	MSM_CLOCK_REG( 7372800,   3, 0x64, 0x32, 0, 2, 4, 1, 245760000), /*  460800*16, will be divided by 4 for 115200 */
#endif
	MSM_CLOCK_REG(12000000,   1, 0x20, 0x10, 1, 3, 1, 1, 768000000), /* SD, 12MHz */
	MSM_CLOCK_REG(14745600,   3, 0x32, 0x19, 0, 2, 4, 1, 245760000), /* BT, 921600 (*16)*/
	MSM_CLOCK_REG(19200000,   1, 0x0a, 0x05, 3, 3, 1, 1, 768000000), /* SD, 19.2MHz */
	MSM_CLOCK_REG(24000000,   1, 0x08, 0x04, 3, 2, 1, 1, 768000000), /* SD & VFE_CLK, 24MHz */
	MSM_CLOCK_REG(32000000,   1, 0x0c, 0x06, 1, 3, 1, 1, 768000000), /* SD, 32MHz */
	MSM_CLOCK_REG(58982400,   6, 0x19, 0x0c, 0, 2, 4, 1, 245760000), /* BT, 3686400 (*16) */
	MSM_CLOCK_REG(64000000,0x19, 0x60, 0x30, 0, 2, 4, 1, 245760000), /* BT, 4000000 (*16) */
	{0, 0, 0, 0, 0, 0},
};

// CDMA phones typically use a 196 MHz PLL0
struct mdns_clock_params msm_clock_freq_parameters_pll0_196[] = {

	MSM_CLOCK_REG(  144000,   3, 0x64, 0x32, 3, 3, 0, 1, 19200000), /* SD, 144kHz */
	MSM_CLOCK_REG( 7372800,   3, 0x50, 0x28, 0, 2, 4, 1, 196608000), /*  460800*16, will be divided by 4 for 115200 */
	MSM_CLOCK_REG(12000000,   1, 0x20, 0x10, 1, 3, 1, 1, 768000000), /* SD, 12MHz */
	MSM_CLOCK_REG(14745600,   3, 0x28, 0x14, 0, 2, 4, 1, 196608000), /* BT, 921600 (*16)*/
	MSM_CLOCK_REG(19200000,   1, 0x0a, 0x05, 3, 3, 1, 1, 768000000), /* SD, 19.2MHz */
	MSM_CLOCK_REG(24000000,   1, 0x10, 0x08, 1, 3, 1, 1, 768000000), /* SD, 24MHz */
	MSM_CLOCK_REG(32000000,   1, 0x0c, 0x06, 1, 3, 1, 1, 768000000), /* SD, 32MHz */
	MSM_CLOCK_REG(58982400,   3, 0x0a, 0x05, 0, 2, 4, 1, 196608000), /* BT, 3686400 (*16) */
	MSM_CLOCK_REG(64000000,0x7d, 0x180, 0xC0, 0, 2, 4, 1, 196608000), /* BT, 4000000 (*16) */
	{0, 0, 0, 0, 0, 0},
};

// defines from MSM7500_Core.h

// often used defines
#define MSM_PRPH_WEB_NS_REG	( MSM_CLK_CTL_BASE+0x80 )
#define MSM_GRP_NS_REG				( MSM_CLK_CTL_BASE+0x84 )
#define MSM_AXI_RESET 					( MSM_CLK_CTL_BASE+0x208 )
#define MSM_ROW_RESET 				( MSM_CLK_CTL_BASE+0x214 )
#define MSM_VDD_GRP_GFS_CTL	( MSM_CLK_CTL_BASE+0x284 )
#define MSM_VDD_VDC_GFS_CTL	( MSM_CLK_CTL_BASE+0x288 )
#define MSM_RAIL_CLAMP_IO			( MSM_CLK_CTL_BASE+0x290 )

#define REG_OR( reg, value ) do { u32 i = readl( (reg) ); writel( i | (value), (reg) ); } while(0)
#define REG_AND( reg, value ) do {	u32 i = readl( reg ); writel( i & ~value, reg); } while(0)
#define REG_SET( reg, value ) do { writel( value, reg ); } while(0)

static inline struct msm_clock_params msm_clk_get_params(uint32_t id)
{
	struct msm_clock_params empty = { };
	if (id < NR_CLKS)
		return msm_clock_parameters[id];
	return empty;
}

static inline uint32_t msm_clk_enable_bit(uint32_t id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	if (!params.idx) return 0;
	return 1U << params.idx;
}

static inline unsigned msm_clk_reg_offset(uint32_t id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	return params.offset;
}

static int set_mdns_host_clock(uint32_t id, unsigned long freq)
{
	int n = 0;
	unsigned offset;
	bool found, enabled, needs_setup;
	struct msm_clock_params params;
	found = 0;

	params = msm_clk_get_params(id);
	offset = params.offset;
	needs_setup = params.setup_mdns;

	if (id == EBI1_CLK)
		return 0;

	if(debug_mask & DEBUG_MDNS)
		D("set mdns: %u, %lu; bitidx=%u, offset=%x\n", id, freq,
	  params.idx, params.offset);

	if (params.ops != NULL && params.ops->set_rate != NULL) {
		/* Currently handles VFE_CLK, VFE_MDC_CLK, MDC_CLK */

		/* -1 means the func wasn't able to set rate, use standard way */
		if(params.ops->set_rate(id, freq) != -1 )
			return 0;
	}

	if (!params.offset) {
		printk(KERN_WARNING "%s: FIXME! Don't know how to set clock %u - no known Md/Ns reg\n", __func__, id);
		return 0;
	}
    

	if (needs_setup) {
		enabled = pc_clk_is_enabled(id);
		if (enabled)
			pc_clk_disable(id);

		while (msm_clock_freq_parameters[n].freq) {
			n++;
		}

		for (n--; n >= 0; n--) {
			if (freq >= msm_clock_freq_parameters[n].freq) {
				// This clock requires MD and NS regs to set frequency:
				writel(msm_clock_freq_parameters[n].md, MSM_CLK_CTL_BASE + offset - 4);
				writel(msm_clock_freq_parameters[n].ns, MSM_CLK_CTL_BASE + offset);
//				msleep(5);
				if(debug_mask&DEBUG_MDNS)
					D("%s: %u, freq=%lu calc_freq=%u pll%d=%u expected pll =%u\n", __func__, id,
				msm_clock_freq_parameters[n].freq,
				msm_clock_freq_parameters[n].calc_freq,
				msm_clock_freq_parameters[n].ns&7,
				pll_get_rate(idx2pll(msm_clock_freq_parameters[n].ns&7)),
				msm_clock_freq_parameters[n].pll_freq);
				found = 1;
				break;
			}
		}

		if (enabled)
			pc_clk_enable(id);
	}


	if ((!found && needs_setup) && (debug_mask & DEBUG_UNKNOWN_FREQ)) {
		printk(KERN_WARNING "clock-wince: FIXME! set_sdcc_host_clock could not "
			"find suitable parameter for freq %lu\n", freq);
	}

       return 0;
}

static unsigned long get_mdns_host_clock(uint32_t id)
{
	int n;
	unsigned offset;
	uint32_t mdreg;
	uint32_t nsreg;
	unsigned long freq = 0;

	offset = msm_clk_reg_offset(id);
	if (offset == 0)
		return -EINVAL;

	mdreg = readl(MSM_CLK_CTL_BASE + offset - 4);
	nsreg = readl(MSM_CLK_CTL_BASE + offset);

	n = 0;
	while (msm_clock_freq_parameters[n].freq) {
		if (msm_clock_freq_parameters[n].md == mdreg &&
			msm_clock_freq_parameters[n].ns == nsreg) {
			freq = msm_clock_freq_parameters[n].freq;
			break;
		}
		n++;
	}

	return freq;
}

static int pc_clk_enable(uint32_t id)
{
	struct msm_clock_params params;
	int done=0;
	params = msm_clk_get_params(id);

	switch (id) {
		case MDP_CLK:
			//Check this mask
			writel ((readl(MSM_CLK_CTL_BASE) & 0x3ffffdff) | 0x200, MSM_CLK_CTL_BASE);
			done = 1;
			break;
	
		case PMDH_CLK:
			writel ((readl(MSM_CLK_CTL_BASE + params.offset) & 0x67f) | 0x800, MSM_CLK_CTL_BASE + params.offset);
			writel ((readl(MSM_CLK_CTL_BASE + params.offset) & 0xc7f) | 0x200, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case I2C_CLK:
			writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x600) | 0x800, MSM_CLK_CTL_BASE + params.offset);
			writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0xc00) | 0x200, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case ADSP_CLK:
			writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x7ff7ff) | 0x800, MSM_CLK_CTL_BASE + params.offset);
			writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x6fffff) | 0x100000, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case UART1_CLK:
		  	writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x10, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x20, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;

		case UART2_CLK:
		  	writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x400, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x800, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;

		case UART3_CLK:
		  	writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x10000, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x20000, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case SDAC_CLK:
		case SDC1_CLK:
		case SDC2_CLK:
		case SDC3_CLK:
		case SDC4_CLK:
		case USB_HS_CLK:
			writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x800, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x100, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x200, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;

		case IMEM_CLK:
			// handled by GRP_CLK operations
			done = 1;
			break;

		case GRP_CLK:
			msm_clk_regime_amss_grp_rail_switch( 1 );
			done = 1;
			break;
		
		default:
		    break;
	}

	if (params.idx)
	{
		writel(readl(MSM_CLK_CTL_BASE) | (1U << params.idx), MSM_CLK_CTL_BASE);
		done=1;
	}
	else if (params.ops != NULL && params.ops->enable != NULL) {
		params.ops->enable(id);              
		done=1;
	}

	if (done)
		return 0;

	//XXX: too spammy, extreme debugging only: D(KERN_DEBUG "%s: %d\n", __func__, id);
	if(debug_mask & DEBUG_UNKNOWN_ID)
		printk(KERN_WARNING "%s: FIXME! enabling a clock that doesn't have an ena bit "
		       "or ns-only offset: %u\n", __func__, id);

	return 0;
}

static void pc_clk_disable(uint32_t id)
{
	int done = 0;
	struct msm_clock_params params;
	params = msm_clk_get_params(id);

	//GRP and IMEM use special order.But do they really need it?
	if (params.idx && (id != GRP_CLK && id != IMEM_CLK))
	{
		writel(readl(MSM_CLK_CTL_BASE) & ~(1U << params.idx), MSM_CLK_CTL_BASE);
		done = 1;

	}
	else if (params.ops != NULL && params.ops->disable != NULL) {
	        params.ops->disable(id);
        	return;
	}

	switch (id) {
		case MDP_CLK:
			writel (readl(MSM_CLK_CTL_BASE) & 0x3ffffdff, MSM_CLK_CTL_BASE);
			done = 1;
			break;
	
		case UART1_CLK:
		  	writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x20, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x10, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;

		case UART2_CLK:
		  	writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x800, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x400, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;

		case UART3_CLK:
		  	writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x20000, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x10000, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case SDAC_CLK:
		case SDC1_CLK:
		case SDC2_CLK:
		case SDC3_CLK:
		case SDC4_CLK:
		case USB_HS_CLK:
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x200, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x800, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x100, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case I2C_CLK:
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0xc00, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x600, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case ADSP_CLK:
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x6fffff, MSM_CLK_CTL_BASE + params.offset);
			if (readl(MSM_CLK_CTL_BASE + params.offset) & 0x280) {
				done = 1;
				break;			
			}
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x7ff7ff, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;
	
		case PMDH_CLK:
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0xc7f, MSM_CLK_CTL_BASE + params.offset);
			writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x67f, MSM_CLK_CTL_BASE + params.offset);
			done = 1;
			break;

		case IMEM_CLK:
			// handled by GRP_CLK operations
			done = 1;
			break;

		case GRP_CLK:
			msm_clk_regime_amss_grp_rail_switch(0);
			writel(readl(MSM_CLK_CTL_BASE) & ~(1U << params.idx), MSM_CLK_CTL_BASE);
			done = 1;
			break;
		
		default:
		break;
	}

	if (done)
		return;

	if(debug_mask&DEBUG_UNKNOWN_ID)
		printk(KERN_WARNING "%s: FIXME! disabling a clock that doesn't have an "
		       "ena bit: %u\n", __func__, id);

}

static int pc_clk_set_rate(uint32_t id, unsigned long rate)
{
	int retval;
	retval = 0;

	if(DEBUG_MDNS)
		D("%s: id=%u rate=%lu\n", __func__, id, rate);

	retval = set_mdns_host_clock(id, rate);

	return retval;
}

static int pc_clk_set_min_rate(uint32_t id, unsigned long rate)
{
	if (id < NR_CLKS)
	 min_clk_rate[id]=rate;
	else if(debug_mask&DEBUG_UNKNOWN_ID)
	 printk(KERN_WARNING " FIXME! clk_set_min_rate not implemented; %u:%lu NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static int pc_clk_set_max_rate(uint32_t id, unsigned long rate)
{
	if (id < NR_CLKS)
	 max_clk_rate[id]=rate;
	else if(debug_mask&DEBUG_UNKNOWN_ID)
	 printk(KERN_WARNING " FIXME! clk_set_max_rate not implemented; %u:%lu NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static unsigned long pc_clk_get_rate(uint32_t id)
{
	unsigned long rate = 0;

	switch (id) {
		/* known MD/NS clocks, MSM_CLK dump and arm/mach-msm/clock-7x30.c */
		case SDC1_CLK:
		case SDC2_CLK:
		case SDC3_CLK:
		case SDC4_CLK:
		case UART1DM_CLK:
		case UART2DM_CLK:
		case USB_HS_CLK:
		case SDAC_CLK:
		case TV_DAC_CLK:
		case TV_ENC_CLK:
		case USB_OTG_CLK:
			rate = get_mdns_host_clock(id);
			break;

		case SDC1_PCLK:
		case SDC2_PCLK:
		case SDC3_PCLK:
		case SDC4_PCLK:
			rate = 64000000; /* g1 value */
			break;

		default:
			//TODO: support all clocks
			if(debug_mask&DEBUG_UNKNOWN_ID)
				printk("%s: unknown clock: id=%u\n", __func__, id);
			rate = 0;
	}

	return rate;
}

static int pc_clk_set_flags(uint32_t id, unsigned long flags)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);

	/* check if clock has specific ops routines */
	if (params.ops != NULL && params.ops->set_flags != NULL) {
		return params.ops->set_flags(id, flags);              
	}

	if(debug_mask&DEBUG_UNKNOWN_CMD)
		printk(KERN_WARNING "%s not implemented for clock: id=%u, flags=%lu\n", __func__, id, flags);
	return 0;
}

static int pc_clk_is_enabled(uint32_t id)
{
	int is_enabled = 0;
	unsigned bit;
	bit = msm_clk_enable_bit(id);
	if (bit > 0)
	{
		is_enabled = (readl(MSM_CLK_CTL_BASE) & bit) != 0;
	}
	//XXX: is this necessary?
	if (id==SDC1_PCLK || id==SDC2_PCLK || id==SDC3_PCLK || id==SDC4_PCLK)
		is_enabled = 1;
	return is_enabled;
}

static int pc_pll_request(unsigned id, unsigned on)
{
	if(debug_mask&DEBUG_UNKNOWN_CMD)
		printk(KERN_WARNING "%s not implemented for PLL=%u\n", __func__, id);

	return 0;
}

/*
 * Standard clock functions defined in include/linux/clk.h
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == dev)
			goto found_it;

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == NULL)
			goto found_it;

	clk = ERR_PTR(-ENOENT);
found_it:
	mutex_unlock(&clocks_mutex);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	if (clk->id == ACPU_CLK)
	{
		return -ENOTSUPP;
	}
	spin_lock_irqsave(&clocks_lock, flags);
	clk->count++;
	if (clk->count == 1)
		pc_clk_enable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	BUG_ON(clk->count == 0);
	clk->count--;
	if (clk->count == 0)
		pc_clk_disable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	return pc_clk_get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	if (clk->flags & CLKFLAG_USE_MAX_TO_SET) {
		ret = pc_clk_set_max_rate(clk->id, rate);
		if (ret)
			return ret;
	}
	if (clk->flags & CLKFLAG_USE_MIN_TO_SET) {
		ret = pc_clk_set_min_rate(clk->id, rate);
		if (ret)
			return ret;
	}

	if (clk->flags & CLKFLAG_USE_MAX_TO_SET ||
		clk->flags & CLKFLAG_USE_MIN_TO_SET)
		return ret;

	return pc_clk_set_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	return pc_clk_set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);


void __init msm_clock_init(void)
{
	struct clk *clk;
	msm_clock_parameters = msm_clock_parameters_def;
	
	spin_lock_init(&clocks_lock);
	mutex_lock(&clocks_mutex);
	for (clk = msm_clocks; clk && clk->name; clk++) {
		list_add_tail(&clk->list, &clocks);
	}

	if (pll_get_rate(0) == 196608000) {
		// cdma pll0 = 196 MHz
		msm_clock_freq_parameters = msm_clock_freq_parameters_pll0_196;
	} else {
		// default gsm pll0 = 245 MHz
		msm_clock_freq_parameters = msm_clock_freq_parameters_pll0_245;
	}

	mutex_unlock(&clocks_mutex);
}

/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */

#define PMIC_API_PROG		0x30000055
#define PMIC_API_VERS 		0x0
#define PMIC_API_GET_KHZ_PROC	0x1
static void get_clk_khz(void)
{
	struct msm_rpc_endpoint *pmic_ep;
	int rc;
	struct {
		struct rpc_request_hdr hdr;
		uint32_t data[1];
	} req;
	
	pmic_ep = msm_rpc_connect(PMIC_API_PROG, PMIC_API_VERS, 0);
	if (IS_ERR(pmic_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(pmic_ep));
		goto close;
	}

	pr_info("IMEM OLD: VAL = %d\n", readl(MSM_IMEM_BASE));
	req.data[0] = cpu_to_be32(1);
	rc = msm_rpc_call(pmic_ep, PMIC_API_GET_KHZ_PROC, &req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);
	
	msleep(100),
	pr_info("IMEM NEW: VAL = %d\n", readl(MSM_IMEM_BASE));

close:
	msm_rpc_close(pmic_ep);
}

static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	unsigned count = 0;

    // disabled to fix kernel crashed
//	if (strstr(saved_command_line, "kexec"))
//		return 0;

    return 0;

	// reset imem config, I guess all devices need this so somewhere here would be good.
	// it needs to be moved to somewhere else.
	// note: this needs to be done before all clocks get disabled.
	writel( 0, MSM_IMEM_BASE );
	pr_info("reset imem_config\n");
	get_clk_khz();

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clocks_lock, flags);
			if (!clk->count) {
				count++;
				pc_clk_disable(clk->id);
			}
			spin_unlock_irqrestore(&clocks_lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);

	return 0;
}

late_initcall(clock_late_init);



/* Debug fs */
static int debug_read_pc_clk(char *buf, int max)
{
    int i = 0;
    struct clk *clk;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, list) {
        if( pc_clk_is_enabled(clk->id) ) {
            i += scnprintf(buf + i, max - i, "%s is enabled @ %ld Hz, count=%d\n", clk->name, clk_get_rate(clk), clk->count);
        } else {
            i += scnprintf(buf + i, max - i, "%s is disabled, count =%d\n", clk->name, clk->count);
        }
    }
    mutex_unlock(&clocks_mutex);

	return i;
}

static int dump_memory_addr(int addr, char *buf, int max) {
    int i = 0, n, j;

    for(j=0; j<256; j+=16) {
        i += scnprintf(buf + i, max - i, "%p | ", MSM_CLK_CTL_BASE + j + addr);
        for(n=0; n<16; n+=4) {
            i += scnprintf(buf + i, max - i, "%08x ", readl(MSM_CLK_CTL_BASE + j + n + addr) );
        }
        i += scnprintf(buf + i, max - i, "\n");
    }

    return i;
}

static int dump_memory(char *buf, int max) {
    return dump_memory_addr(0, buf, max);
}

static int dump_memory_100(char *buf, int max) {
    return dump_memory_addr(0x100, buf, max);
}

static int dump_memory_200(char *buf, int max) {
    return dump_memory_addr(0x200, buf, max);
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static ssize_t debug_write(struct file *file, const char __user *buf,
		     size_t count, loff_t *ppos)
{
	//unsigned long cmd = simple_strtoul(buf, NULL, 16);
    char *pbuffer = kzalloc(count, GFP_KERNEL);
    int n;

    if ( copy_from_user(pbuffer, buf, count) ) {
		return -EFAULT;
	}
    
    while( (pbuffer[0] != 0) && (pbuffer[0] != '\n') ){ 
        switch(pbuffer[0]) {
            case 'f':
                for (n=0; n < 9; n++) {
                    printk("Freq %ld, md=0x%x, ns=0x%x\n", 
                        msm_clock_freq_parameters[n].freq,
				        msm_clock_freq_parameters[n].md,
				        msm_clock_freq_parameters[n].ns);
                }
            break;

        	default:
        		printk(KERN_ERR "msm_clocks: cmd '%c' not supported\n", pbuffer[0]);
		    break;
	    }
        pbuffer++;
    };

	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
    .write = debug_write,
	.open = debug_open,
};
static const struct file_operations debug_ops_readonly = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
			 struct dentry *dent,
			 int (*fill)(char *buf, int max))
{
	debugfs_create_file(name, mode, dent, fill, &debug_ops);
}

static int __init pc_clk_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("msm_clocks", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

    debugfs_create_file("dump", S_IRUSR, dent, &dump_memory, &debug_ops_readonly);
    debugfs_create_file("dump_100", S_IRUSR, dent, &dump_memory_100, &debug_ops_readonly);
    debugfs_create_file("dump_200", S_IRUSR, dent, &dump_memory_200, &debug_ops_readonly);

	debug_create("msm_clk", 0644, dent, &debug_read_pc_clk);

	return 0;
}
device_initcall(pc_clk_dbg_init);


