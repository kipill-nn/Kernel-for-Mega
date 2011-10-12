/*
 * Author: Jbruneaux
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ARCH_ARM_MACH_MSM_CLOCK_REGIME_AMSS_H
#define _ARCH_ARM_MACH_MSM_CLOCK_REGIME_AMSS_H


#ifdef MSM_CLOCK_REGIME_GLOBAL
#define GLOBAL_REGIME
#else
#define GLOBAL_REGIME extern
#endif



#undef GLOBAL_REGIME  

/* Convert from clock.h IDs to Clk regime IDs */
struct clk_regime_lut_t {
    int clk_id;             /* ID of the clock as defined in clock.h */
    int clk_regime_id;      /* ID of the clock to be passed to the clk regime remote app */
};

/* Exported functions */
int msm_clk_regime_amss_enable(int id);
int msm_clk_regime_amss_disable(int id);
int msm_clk_regime_amss_is_on(int id);
int msm_clk_regime_amss_set_rate(int id, int rate);
int msm_clk_regime_amss_set_flags(int id, int flags);
int msm_clk_regime_amss_vfe_rail_switch(unsigned int bOn);
int msm_clk_regime_amss_vfe_sel_src(unsigned int flags);
int msm_clk_regime_amss_vfe_set_rate(unsigned rate);
int msm_clk_regime_amss_vdc_rail_switch(unsigned int bOn);
int msm_clk_regime_amss_grp_rail_switch(unsigned int bOn);

/* debugfs */
int debug_read_amss_clk(char *buf, int max);
ssize_t debug_write_amss(struct file *file, const char __user *buf,
		     size_t count, loff_t *ppos);

#endif /* _ARCH_ARM_MACH_MSM_CLOCK_REGIME_AMSS_H */
