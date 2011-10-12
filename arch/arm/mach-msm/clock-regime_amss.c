/*
 * Author: Jbruneaux
 *
 * Description : This file contains a wince AMSS compatible clock regime
 * implementation. Known to be compatible at this time with AMSS version
 * 5225, 6125 and 6150
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <mach/msm_iomap.h>
#include <asm/io.h>

#include "clock.h"

#define MSM_CLOCK_REGIME_AMSS_GLOBAL
#include "clock-regime_amss.h"

/***************************************************************************************************
 * Defines
 ***************************************************************************************************/

#if 1
#define CRGM_DBG(x...) printk(KERN_DEBUG "clock-regime-amss: " x)
#else
#define CRGM_DBG(x...) do {} while (0)
#endif

#define MSM_GLBL_CLK_STATE      ( MSM_CLK_CTL_BASE+0x4 )
#define MSM_CAM_MD_REG          ( MSM_CLK_CTL_BASE+0x40 )
#define MSM_CAM_NS_REG          ( MSM_CLK_CTL_BASE+0x44 )
#define MSM_PRPH_WEB_NS_REG     ( MSM_CLK_CTL_BASE+0x80 )
#define MSM_GRP_NS_REG          ( MSM_CLK_CTL_BASE+0x84 )
#define MSM_CLK_HALT_STATEA     ( MSM_CLK_CTL_BASE+0x100 )	// was 0x108, should be 0x100
#define MSM_CLK_HALT_STATEB     ( MSM_CLK_CTL_BASE+0x104 )	// was 0x108, should be 0x104
#define MSM_AXI_RESET           ( MSM_CLK_CTL_BASE+0x208 )
#define MSM_APPS_RESET          ( MSM_CLK_CTL_BASE+0x210 )
#define MSM_ROW_RESET           ( MSM_CLK_CTL_BASE+0x214 )
#define MSM_VDD_VFE_GFS_CTL     ( MSM_CLK_CTL_BASE+0x280 )
#define MSM_VDD_GRP_GFS_CTL     ( MSM_CLK_CTL_BASE+0x284 )
#define MSM_VDD_VDC_GFS_CTL     ( MSM_CLK_CTL_BASE+0x288 )
#define MSM_RAIL_CLAMP_IO       ( MSM_CLK_CTL_BASE+0x290 )

#define REG_OR( reg, value ) do { u32 i = readl( (reg) ); writel( i | (value), (reg) ); } while(0)
#define REG_AND( reg, value ) do {	u32 i = readl( reg ); writel( i & ~value, reg); } while(0)
#define REG_AND_MASK( reg, value ) do {	u32 i = readl( reg ); writel( i & value, reg); } while(0)
#define REG_AND_OR( reg, value_and, value_or ) do { u32 i = readl( reg ); writel( (i & ~(value_and)) | (value_or), reg); } while(0)
#define REG_ANDM_OR( reg, value_and, value_or ) do { u32 i = readl( reg ); writel( (i & (value_and)) | (value_or), reg); } while(0)
#define REG_AND_ADD( reg, value_and, value_add ) do { u32 i = readl( reg ); writel( (i & ~(value_and)) + (value_add), reg); } while(0)
#define REG_SET( reg, value ) do { writel( value, reg ); } while(0)

/***************************************************************************************************
 * Private vars
 ***************************************************************************************************/

static unsigned char clk_7_state = 0;
static unsigned char clk_40_41_freq_ndx = 0;     // seems to be an index in clocks_params array


struct clocks_params_t {
    unsigned int frequency;
    unsigned int param0;
    unsigned int source;
    unsigned int src_div;
    unsigned int param3;
    unsigned int md;
    unsigned int ns;
};

struct clocks_params_t clocks_params_low[] = {
/* 2048 */      { 0x800,        1,   4,   3,   1,   0x1FFF0,    0xFFF1 },
/* 2822 */      { 0xB06,        1,   4,   0,   1,   0x93E6FF,   0xE792 },
/* 3072 */      { 0xC00,        1,   4,   3,   1,   0x1FFF5,    0xFFF6 },
/* 4096 */      { 0x1000,       1,   4,   3,   1,   0x2FFF0,    0xFFF2 },
/* 5644 */      { 0x160C,       1,   4,   0,   1,   0x93F37F,   0xF412 },
/* 6144 */      { 0x1800,       1,   4,   3,   1,   0x1FFFA,    0xFFFB },
/* 8192 */      { 0x2000,       1,   4,   1,   1,   0x2FFF0,    0xFFF2 },
/* 11289 */     { 0x2C19,       1,   4,   0,   1,   0x93F9BF,   0xFA52 },
/* 12288 */     { 0x3000,       1,   4,   1,   1,   0x1FFFA,    0xFFFB },
};

struct clocks_params_t clocks_params[] = {
/* 3Mhz */      { 0x2DC6C0,     2,   1,   3,   0,   0x1FFBF,    0xFFC0 },
/* 4.8Mhz */    { 0x493E00,     0,   0,   3,   0,   0,          0 },
/* 5.19Mhz */   { 0x4F3170,     2,   1,   1,   0,   0xADCDFF,   0xCEAC },
/* 5.25Mhz */   { 0x501BD0,     2,   1,   1,   0,   0x7FDFF,    0xFE06 },
/* 5.4Mhz */    { 0x5265C0,     2,   1,   1,   0,   0x9FD7F,    0xFD88 },
/* 6Mhz */      { 0x5B8D80,     2,   1,   3,   0,   0x1FFDF,    0xFFE0 },
/* 6.4Mhz */    { 0x61A800,     0,   0,   2,   0,   0,          0 },
/* 8Mhz */      { 0x7A1200,     2,   1,   3,   0,   0x1FFE7,    0xFFE8 },
/* 9.4Mhz */    { 0x8F6EC0,     2,   1,   1,   0,   0x2FF87F,   0xF8AE },
/* 9.5645Mhz */ { 0x91F154,     2,   1,   1,   0,   0x5792447,  0x29C0 },
/* 9.6Mhz */    { 0x927C00,     0,   0,   1,   0,   0,          0 },
/* 10.0325Mhz */{ 0x9D8C08,     2,   1,   1,   0,   0x19DC3FF,  0xC59C },
/* 10.5Mhz */   { 0xA037A0,     2,   1,   1,   0,   0x7FEFF,    0xFF06 },
/* 12 Mhz */    { 0xB71B00,     2,   1,   3,   0,   0x1FFEF,    0xFFF0 },
/* 13 Mhz */    { 0xC65D40,     2,   1,   1,   0,   0xDFE7F,    0xFE8C },
/* 16Mhz */     { 0xF42400,     2,   1,   3,   0,   0x1FFF3,    0xFFF4 },
/* 18.8Mhz */   { 0x11EDD80,    2,   1,   1,   0,   0x2FFC3F,   0xFC6E },
/* 19.129Mhz */ { 0x123E2A8,    2,   1,   1,   0,   0xBD71252,  0x1E29 },
/* 19.2Mhz */   { 0x124F800,    0,   0,   0,   0,   0,          0 },
/* 20.65Mhz */  { 0x13B1810,    2,   1,   1,   0,   0x19DE1FF,  0xE39C },
/* 21Mhz */     { 0x1406F40,    2,   1,   1,   0,   0x7FF7F,    0xFF86 },
/* 24Mhz */     { 0x16E3600,    2,   1,   3,   0,   0x1FFF7,    0xFFF8 },
/* 25.6Mhz */   { 0x186A000,    2,   1,   3,   0,   0x2FFF0,    0xFFF2 },
/* 32Mhz */     { 0x1E84800,    2,   1,   3,   0,   0x1FFF9,    0xFFFA },
/* 36Mhz */     { 0x2255100,    2,   1,   1,   0,   0x3FFDF,    0xFFE2 },
/* 37.6Mhz */   { 0x23DBB00,    2,   1,   1,   0,   0x2FFE1F,   0xFE4E },
/* 38.258Mhz */ { 0x247C550,    2,   1,   1,   0,   0x17AE1252, 0x2A00 },
/* 38.4Mhz */   { 0x249F000,    2,   1,   3,   0,   0x1FFFA,    0xFFFB },
/* 40Mhz */     { 0x2625A00,    2,   1,   1,   0,   0x5FFCF,    0xFFD4 },
/* 41.3Mhz */   { 0x2763020,    2,   1,   1,   0,   0x19DF0FF,  0xF29C },
/* 42Mhz */     { 0x280DE80,    2,   1,   1,   0,   0x7FFBF,    0xFFC6 },
/* 48Mhz */     { 0x2DC6C00,    2,   1,   3,   0,   0x1FFFB,    0xFFFC },
/* 51.2Mhz */   { 0x30D4000,    2,   1,   1,   0,   0x2FFF0,    0xFFF2 },
/* 64Mhz */     { 0x3D09000,    2,   1,   3,   0,   0x1FFFC,    0xFFFD },
/* 65Mhz */     { 0x3DFD240,    2,   1,   1,   0,   0x41FE7F,   0xFEC0 },
/* 76.8Mhz */   { 0x493E000,    2,   1,   3,   0,   0x2FFFA,    0xFFFC },
};

struct clk_regime_lut_t clk_regime_lut[] = {
    {VFE_CLK,           VFE_CLK},
    {VFE_MDC_CLK,       VFE_MDC_CLK},
    {MDC_CLK,           0x51},
    {VDC_CLK,           0x58},
    {0x37,              0x37},
};

/***************************************************************************************************
 * Private functions
 ***************************************************************************************************/
static inline void clk_regime_msm_log(int clock, int status)
{
    CRGM_DBG("Clock %d status changed to %d\n", clock, status);
}

/* Convert a clk ID to rpc clk regime clk ID */
static int msm_rpc_clk_regime_get_id(int id)
{
    int i;
    int clk_regime_id = -1;

    for(i = 0;
         i < ARRAY_SIZE(clk_regime_lut);
         i++)
    {
        if( id == clk_regime_lut[i].clk_id ) {
            clk_regime_id = clk_regime_lut[i].clk_regime_id;
            break;
        }
    }

    return clk_regime_id;
}
/* Warning, clock ids used in this function seems to be different 
 * from ids used in enable or disable functions...
 */
static int clk_regime_is_on(int id)
{
    int rc = 0;

   	switch(id)
	{
	    case 0:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x100000;
        break;

	    case 1:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x10000;
        break;

	    case 2:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x8000;
        break;

	    case 3:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x4000;
        break;

	    case 4:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x200;
        break;

	    case 8:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x10000000;
        break;

        case 9:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x8000000;
        break;

        case 10:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x100;
        break;

        case 11:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x80;
        break;

        case 12:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x40;
        break;

        case 13:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x20;
        break;

        case 14:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x10;
        break;

        case 15:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x8;
        break;

        case 16:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x4;
        break;

        case 17:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x2;
        break;

        case 22:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x800;
        break;

        case 23:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x400;
        break;

        case 25:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x1000;
        break;

        case 26:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x1;
        break;

        case 27:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x8000;
        break;

        case 28:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x4000;
        break;

        case 29:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x80000000;
        break;

        case 30:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x40000000;
        break;

        case 31:
            rc = readl(MSM_CLK_CTL_BASE + 0x34) & 0x20000;
        break;

        case 32:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x20000000;
        break;

        case 33:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x10000000;
        break;

        case 35:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x4000000;
        break;

        case 36:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x2000000;
        break;

        case 37:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x1000000;
        break;

        case 38:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x800000;
        break;

        case 39:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x2000;
        break;

        case 40:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x400000;
        break;

        case 41:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x200000;
        break;

        case 43:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x100000;
        break;

        case 44:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x80000;
        break;

        case 45:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x40000;
        break;

        case 46:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x20000;
        break;

        case 48:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x10000;
        break;

        case 49:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x8000;
        break;

        case 50:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x4000;
        break;

        case 51:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x2000;
        break;

        case 52:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x1000;
        break;

        case 53:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x800;
        break;

        case 54:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x400;
        break;

        case 55:
            if ( !(readl(MSM_CAM_NS_REG) & 0x4000) ) {
                if ( !(readl(MSM_CLK_HALT_STATEA) & 0x200) ) {
                    rc = 1;
                } else {
                    rc = !(readl(MSM_CAM_NS_REG) & 0x800);
                }
            } else {
                rc = !(readl(MSM_CAM_NS_REG) & 0x800);
            }
        break;

        case 56:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x100;
        break;

        case 57:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x80;
        break;

        case 58:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x40;
        break;

        case 59:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x20;
        break;

        case 60:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x10;
        break;

        case 61:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x8;
        break;

        case 62:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x4;
        break;

        case 63:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x2;
        break;

        case 64:
            rc = readl(MSM_CLK_HALT_STATEA) & 0x1;
        break;

        case 65:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x1000000;
        break;

        case 66:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x2000000;
        break;

        case 67:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x400000;
        break;

        case 68:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x200000;
        break;

        case 69:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x40000;
        break;

        case 70:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x80000;
        break;

        case 71:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x40000;
        break;

        case 73:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x20000;
        break;

        case 74:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x2000;
        break;

        case 75:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x1000;
        break;

        case 76:
            rc = readl(MSM_CLK_CTL_BASE + 0xC4) & 0x40;
        break;

        case 77:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x800;
        break;

        case 78:
            rc = readl(MSM_CLK_CTL_BASE + 0xCC) & 0x1000;
        break;

        case 79:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x400;
        break;

        case 80:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x200;
        break;

        case 81:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x100;
        break;

        case 82:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x40;
        break;

        case 83:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x800000;
        break;

        case 84:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x80;
        break;

        case 85:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x20;
        break;

        case 86:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x10;
        break;

        case 87:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x8;
        break;

        case 88:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x4;
        break;

        case 89:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x4000000;
        break;

        case 92:
            rc = readl(MSM_GLBL_CLK_STATE) & 0x2000000;
        break;

        case 95:
            rc = readl(MSM_CLK_HALT_STATEB) & 0x2;
        break;

        case 96:
            if ( (readl(MSM_CAM_NS_REG) & 0x4000) || 
                   (readl(MSM_CLK_HALT_STATEB) & 0x1) )   {
                rc = !(readl(MSM_CAM_NS_REG) & 0x200);
            } 
        break;

        default:
            printk("clk_regime_is_on: Clock branch out of range (%d)\n", id);
        break;
    }
    
    rc = !rc;

    printk("clk_regime_is_on: %d is %d\n", id, rc);

    return rc;
}

void clk_regime_wait_on_off(int clock, int status_waited)
{
    int cnt = 0;
    if( clk_regime_is_on(clock) != status_waited ) {
        do {
            mdelay(5);
            cnt += 1;             
        } while( (cnt < 100) && (clk_regime_is_on(clock) != status_waited) );
        if ( cnt >= 100 ) { 
            printk("CLKREGIME wait on off exceed %d tries!\n", cnt);
        }
    }
}

static int clk_regime_off(int clock);

void clk_regime_off_stateful(int clk_id)
{
    switch(clk_id) 
    {
        case 7:
            if( clk_7_state == 0 ) {
                clk_regime_off(7);
            }
        break;

        case 11:
            if ( !(readl(MSM_VDD_VDC_GFS_CTL) & 0x100) &&
                    !(readl(MSM_GLBL_CLK_STATE) & 0x100) ) {
                 clk_regime_off(11);
            }
        break;

        case 12:
            if ( (readl(MSM_RAIL_CLAMP_IO) & 0x20) &&
                    !(readl(MSM_VDD_VFE_GFS_CTL) & 0x100) &&
                    !(readl(MSM_CLK_CTL_BASE) & 0x20) &&
                    !(readl(MSM_CLK_CTL_BASE + 0xE8) & 0x200) &&
                    !(readl(MSM_CLK_CTL_BASE) & 0x2000000) &&
                    !(readl(MSM_CLK_CTL_BASE + 0x2C0) & 0x200) &&
                    (clk_7_state == 0) ) {
                clk_regime_off(12);
            }
        break;

        case 13:
            if ( (readl(MSM_RAIL_CLAMP_IO) & 0x20) ) {          
                clk_regime_off(13);
            }
        break;

        case 14: 
            if ( !(readl(MSM_CLK_CTL_BASE) & 0x8000) &&        
                    !(readl(MSM_CLK_CTL_BASE) & 0x10000) ) { 
                clk_regime_off(14);
            }
        break;

        default:
            printk("Invalid LI clk: %d\n", clk_id);
        break;
    }
}

int msm_clk_regime_amss_vdc_rail_switch(unsigned int bOn);

static int clk_regime_on(int clock)
{
    int rc = 0;

	switch(clock)
	{
		case 0:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x100000);
			clk_regime_msm_log(1, 1);	
		break;
		
		case 1:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x40000);
			clk_regime_msm_log(2, 1);	
		break;		

		case 2:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x10000);
			clk_regime_msm_log(4, 1);	
		break;	
		
		case 3:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x8000);
			clk_regime_msm_log(5, 1);	
		break;	

		case 4:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4000);
			clk_regime_msm_log(6, 1);	
		break;			
		
		case 5:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x2000);		
			clk_regime_msm_log(7, 1);	
		break;	

		case 7:
			clk_7_state = 1;
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4);
			// TODO : clk_regime_msm_reset(0x12);
		break;	
		
		case 11:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x8);
			clk_regime_msm_log(0x11, 1);	
		break;	
		
		case 12:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4);		
			clk_regime_msm_log(0x12, 1);	
		break;		
		
		case 13:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x2);				
			clk_regime_msm_log(0x13, 1);	
		break;			

		case 14:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x800);				
			clk_regime_msm_log(9, 1);	
		break;	
		
		case 15:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x400);	
			clk_regime_msm_log(0xA, 1);	
		break;			
		
		case 17:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x1000);	
			clk_regime_msm_log(0x8, 1);	
		break;	
	
		case 18:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x1);	
			clk_regime_msm_log(0x14, 1);	
		break;	
		
		case 19:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x30, 0x07FFF, 0x800);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x30, 0x07FFF, 0x200);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x30, 0x07FFF, 0x80);
			clk_regime_msm_log(0x3C, 1);	
		break;	
		
		case 20:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x200);
			clk_regime_msm_log(0xB, 1);	
		break;			
		
		case 24:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x10000000);
			clk_regime_msm_log(0x50, 1);	
		break;			
		
		case 25:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x08000000);
			clk_regime_msm_log(0x4F, 1);	
		break;			

		case 26:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x100);
			clk_regime_msm_log(0xC, 1);	
		break;	

		case 27:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x80);
			clk_regime_msm_log(0xD, 1);	
		break;	
		
		case 28:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x40);
			clk_regime_msm_log(0xE, 1);	
		break;			
		
		case 29:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4 | 0x20);		
			clk_regime_msm_log(0xF, 1);	
		break;		
		
		case 30:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x10);		
			clk_regime_msm_log(0x10, 1);	
		break;				
	
		case 31:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x28, 0x0FFFF, 0x800);				
			if( (readl(MSM_CLK_CTL_BASE + 0x28) & 0x0C000) & 0x8000)
			{
				REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x2C, 0x7FFF, 0x800);				
				REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x28, 0x0EFFF, 0x800);				
			}				
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x28, 0x0FFFF, 0x200 | 0x80);		
			clk_regime_msm_log(0x1F, 1);	
		break;	

		case 32:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x2C, 0x07FFF, 0x800 | 0x200 | 0x80);
			clk_regime_msm_log(0x1D, 1);	
		break;	

		case 33:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x34, 0x07FFFFF, 0x800 | 0x100000);
			clk_regime_msm_log(0x19, 1);	
		break;	
	
		case 35:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x34, 0x07FFFFF, 0x80000 | 0x20000);		
			clk_regime_msm_log(0x17, 1);	
		break;	

		case 36:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x34, 0x07FFFFF, 0x10000 | 0x4000);				
			clk_regime_msm_log(0x18, 1);	
		break;	
		
		case 37:		
			clk_regime_on(7);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x34, 0x07FFFFF, 0x800 | 0x200);				
			clk_regime_msm_log(0x15, 1);	
		break;	
		
		case 38:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x34, 0x07FFFFF, 0x800 | 0x80);						
			clk_regime_msm_log(0x16, 1);	
		break;	
		
		case 39:
			if( (readl(MSM_CLK_CTL_BASE + 0x4C) & 0x060) != 0)
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x4C, 0x100);						
			}
			REG_OR(MSM_CLK_CTL_BASE + 0x4C, 0x800);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x3C, 0x07FF, 0x200);
			clk_regime_msm_log(0x1A, 1);
		break;
		
		case 40:
			REG_ANDM_OR(MSM_CAM_NS_REG, 0xFFFFFFF8, clocks_params[clk_40_41_freq_ndx].source & 0x7);
				
			if( (readl(MSM_CAM_NS_REG) & 0x060) != 0)
			{
				REG_OR(MSM_CAM_NS_REG, 0x100);
			}	
			
			REG_OR(MSM_CAM_NS_REG, 0x800);
			clk_regime_msm_log(0x2B, 1);	
		break;	

		case 41:
			REG_ANDM_OR(MSM_CAM_NS_REG, 0xFFFFFFF8, clocks_params[clk_40_41_freq_ndx].source & 0x7);
					
			if( (readl(MSM_CAM_NS_REG) & 0x060) != 0)
			{
				REG_OR(MSM_CAM_NS_REG, 0x100);				
			}	
			clk_regime_msm_log(0x4A, 1);	
		break;	
		
		case 42:
			if( (readl(MSM_CLK_CTL_BASE + 0x4C) & 0x060) != 0)
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x4C, 0x100);
			}
			REG_OR(MSM_CLK_CTL_BASE + 0x4C, 0x800 | 0x4000 | 0x200);
				
			clk_regime_msm_log(0x26, 1);
		break;		
		
		case 43:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x50, 0xE7F, 0x800 | 0x200);				
			clk_regime_msm_log(0x21, 1);	
		break;	

		case 44:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x54, 0xE00, 0x800 | 0x200);	
			clk_regime_msm_log(0x22, 1);	
		break;			
		
		case 45:
			REG_OR(MSM_CLK_CTL_BASE + 0x5C, 0x100 | 0x800 | 0x200);	
			clk_regime_msm_log(0x23, 1);	
		break;	
		
		case 47:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x68, 0xE00, 0x800 | 0x200);
			clk_regime_msm_log(0x25, 1);	
		break;			
		
		case 48:
			REG_OR(MSM_CLK_CTL_BASE + 0x70, 0x100 | 0x800 | 0x200);	
			clk_regime_msm_log(0x28, 1);	
		break;			
	
		case 49:
			REG_OR(MSM_CLK_CTL_BASE + 0x70, 0x100 | 0x800 | 0x1000);			
			clk_regime_msm_log(0x27, 1);	
		break;
	
		case 50:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x78, 0xB000, 0x100 | 0x800);
			REG_OR(MSM_CLK_CTL_BASE + 0x70, 0x100 | 0x800);			
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x78, 0xB000, 0x200);
			clk_regime_msm_log(0x27, 1);	
		break;	
		
		case 51:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x7C, 0xFFF, 0x800 | 0x200);
			clk_regime_msm_log(0x2A, 1);	
		break;			
		
		case 52:
			REG_ANDM_OR(MSM_PRPH_WEB_NS_REG, 0xE00, 0x800 | 0x200);		
			clk_regime_msm_log(0, 1);	
		break;			
	
		case 53:
			REG_ANDM_OR(MSM_PRPH_WEB_NS_REG, 0x1FFF, 0x800 | 0x200);		
			clk_regime_msm_log(0x38, 1);	
		break;		
	
		case 54:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x84, 0x1FFF, 0x800 | 0x200);	
			clk_regime_msm_log(0x39, 1);	
		break;		
	
		case 55:
			if( (readl(MSM_CLK_CTL_BASE + 0x4C) & 0x060) != 0)
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x4C, 0x100);
			}
			REG_OR(MSM_CLK_CTL_BASE + 0x4C, 0x800);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x88,  0x5FF);	
			clk_regime_msm_log(0x2D, 1);
		break;		
	
		case 56:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x8C, 0xE7F, 0x800 | 0x200);	
			clk_regime_msm_log(0x2E, 1);	
		break;	
		
		case 57:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x90, 0x3E00, 0x800 | 0x200);
			clk_regime_msm_log(0x30, 1);	
		break;	

		case 58:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x94, 0x1FFFFF, 0x100000 | 0x40000);
			clk_regime_msm_log(0x2F, 1);	
		break;

		case 59:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x94, 0x1FFFFF, 0x1000 | 0x400);
			clk_regime_msm_log(0x42, 1);	
		break;

		case 60:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x94, 0x1FFFFF, 0x100 | 0x40);		
			clk_regime_msm_log(0x1B, 1);	
		break;
		
		case 61:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x94, 0x1FFFFF, 0x10 | 0x4);			
			clk_regime_msm_log(0x41, 1);	
		break;		
	
		case 62:
			if( (readl(MSM_CLK_CTL_BASE + 0x9C) & 0x060) )
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x9C, 0x100);
			}
			
			REG_OR(MSM_CLK_CTL_BASE + 0x9C, 0x800 | 0x200);
				
			if( readl(MSM_CLK_CTL_BASE + 0x9C) & 0x7 ) {
			    clk_regime_wait_on_off(62, 1);
            }
				
			clk_regime_msm_log(0x32, 1);	
		break;	

		case 63:
			if( (readl(MSM_CLK_CTL_BASE + 0x9C) & 0x060) )
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x9C, 0x100);
			}

			REG_OR(MSM_CLK_CTL_BASE + 0x9C, 0x4000 | 0x1000);			
			clk_regime_msm_log(0x51, 1);	
		break;	
		
		case 64:
			if( (readl(MSM_CLK_CTL_BASE + 0x104) & 0x060) )
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x104, 0x100);
			}

			REG_OR(MSM_CLK_CTL_BASE + 0x104, 0x800 | 0x200);
				
			clk_regime_msm_log(0x33, 1);	
			clk_regime_wait_on_off(63, 1);
		break;			
		
		case 65:
			if( (readl(MSM_CLK_HALT_STATEB) & 0x060) )
			{
				REG_OR(MSM_CLK_HALT_STATEB, 0x100);
			}

			REG_OR(MSM_CLK_HALT_STATEB, 0x800 | 0x200);
				
			clk_regime_msm_log(0x34, 1);	
			clk_regime_wait_on_off(64, 1);
		break;			
		
		case 66:
			if( (readl(MSM_CLK_CTL_BASE + 0x114) & 0x060) )
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x114, 0x100);
			}

			REG_OR(MSM_CLK_CTL_BASE + 0x114, 0x800 | 0x200);
			
			clk_regime_msm_log(0x4D, 1);	
			clk_regime_wait_on_off(65, 1);
		break;			
		
		case 67:
			if( (readl(MSM_CLK_CTL_BASE + 0x11C) & 0x060) )
			{
				REG_OR(MSM_CLK_CTL_BASE + 0x11C, 0x100);
			}

			REG_OR(MSM_CLK_CTL_BASE + 0x11C, 0x800 | 0x200);
				
			clk_regime_msm_log(0x4E, 1);	
			clk_regime_wait_on_off(66, 1);
		break;			
		
		case 68:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xC0, 0xE00, 0x800 | 0x200);
			clk_regime_msm_log(0x3E, 1);	
		break;		
		
		case 69:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xC4, 0xFC0, 0x100 | 0x40);
			clk_regime_msm_log(0x37, 1);	
		break;		

		case 70:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xCC, 0xC000, 0x100);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xC4, 0xFC0, 0x800 | 0x200);
			
			clk_regime_msm_log(0x3F, 1);	
			clk_regime_wait_on_off(77, 1);	
		break;			
		
		case 71:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xCC, 0xC000, 0x100);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xCC, 0xFC0, 0x800 | 0x1000);
			clk_regime_msm_log(0, 1);	
		break;	
		
		case 72:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xCC, 0xC000, 0x100);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xCC, 0xFC0, 0x800 | 0x200);		
			clk_regime_msm_log(0x40, 1);	
		break;			

		case 73:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE0, 0x7FFFFFF, 0x10 | 0x20);				
			clk_regime_msm_log(0x43, 1);	
		break;	

		case 74:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE0, 0x7FFFFFF, 0x400 | 0x800);				
			clk_regime_msm_log(0x45, 1);	
		break;	
		
		case 75:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE0, 0x7FFFFFF, 0x10000 | 0x20000);				
			clk_regime_msm_log(0x46, 1);	
		break;			

		case 76:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x020000);
			clk_regime_msm_log(3, 1);				
			REG_OR(MSM_CLK_CTL_BASE + 0xD4, 0x100 | 0x800 | 0x200);		
			clk_regime_msm_log(0x44, 1);	
		break;	
		
		case 77:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4000000);
			REG_OR(MSM_CLK_CTL_BASE + 0xDC, 0x100 | 0x800 | 0x200);					
			clk_regime_msm_log(0x4B, 1);	
		break;			
		
		case 78:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE0, 0x7FFFFFF, 0x2000000);
			clk_regime_msm_log(0, 1);
		break;	

		case 79:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE0, 0x7FFFFFF, 0x4000000);
			clk_regime_msm_log(0, 1);
		break;
		
		case 80:
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE8, 0xFFFFFF, 0x100 | 0x800 | 0x1000);	
			clk_regime_msm_log(0x48, 1);	
		break;			
		
		case 81:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0xE8, 0xFFFFFF, 0x100 | 0x800 | 0x200);						
			clk_regime_msm_log(0x47, 1);	
		break;		
		
		case 82:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4);
			REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x2C0, 0xFFFFFF, 0x100 | 0x800 | 0x200);
			clk_regime_msm_log(0x4C, 1);	
		break;			

		case 85:
			REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 0x4 | 0x2000000);			
			clk_regime_msm_log(0x52, 1);	
		break;	
		
		case 88:
                        msm_clk_regime_amss_vdc_rail_switch(1);
			clk_regime_msm_log(0x49, 1);
		break;
		
		default :
			printk("%s: Clock branch out of range (%d)", __func__, clock);
			rc = -1;
		break;
	}
    
    return rc;

}

int msm_clk_regime_amss_vfe_rail_switch(unsigned int bOn);

static int clk_regime_off(int clock)
{
    int regval = 0;
    int rc = 0;

	switch(clock)
	{
		case 0:
			REG_AND(MSM_CLK_CTL_BASE, 0x100000);
		break;
		
		case 1:
			REG_AND(MSM_CLK_CTL_BASE, 0x40000);
		break;

		case 2:
			REG_AND(MSM_CLK_CTL_BASE, 0x10000);
		break;
		
		case 3:
			REG_AND(MSM_CLK_CTL_BASE, 0x8000);
		break;		
	
		case 4:
			REG_AND(MSM_CLK_CTL_BASE, 0x4000);
		break;	

		case 5:
			REG_AND(MSM_CLK_CTL_BASE, 0x2000);
		break;	
	
		case 7:
			clk_regime_off_stateful(0xC);
		break;
	
		case 11:
			REG_AND(MSM_CLK_CTL_BASE, 0x8);    
		break;

		case 12:
			REG_AND(MSM_CLK_CTL_BASE, 0x4);    
		break;
		
		case 13:
			REG_AND(MSM_CLK_CTL_BASE, 0x2);
		break;		
	
		case 14:
			REG_AND(MSM_CLK_CTL_BASE, 0x800);
		break;				

		case 15:
			REG_AND(MSM_CLK_CTL_BASE, 0x400);
		break;						

		case 17:
			REG_AND(MSM_CLK_CTL_BASE, 0x1000);
		break;			
		
		case 18:
			REG_AND(MSM_CLK_CTL_BASE, 0x1);
		break;	
	
		case 19:
			REG_AND(MSM_CLK_CTL_BASE + 0x030, (0x200 | 0x80 | 0x800));
		break;	
		
		case 20:
			REG_AND(MSM_CLK_CTL_BASE, 0x200);
		break;	
		
		case 24:
			REG_AND(MSM_CLK_CTL_BASE, 0x10000000);
		break;			
		
		case 25:
			REG_AND(MSM_CLK_CTL_BASE, 0xC8000000); // 0xC3200000 << 6
		break;
		
		case 26:
			REG_AND(MSM_CLK_CTL_BASE, 0x100);
		break;			

		case 27:
			REG_AND(MSM_CLK_CTL_BASE, 0x80);
		break;	
		
		case 28:
			REG_AND(MSM_CLK_CTL_BASE, 0x40);
		break;			
		
		case 29:
			REG_AND(MSM_CLK_CTL_BASE, 0x20);
			clk_regime_off_stateful(0xC);
		break;			
		
		case 30:
			REG_AND(MSM_CLK_CTL_BASE, 0x10);
		break;					

		case 31:
			REG_AND(MSM_CLK_CTL_BASE + 0x28, (0x800 | 0x200 | 0x80) );
			if ( !(readl(MSM_CLK_CTL_BASE + 0x28) & 0xA000) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x2C) & 0x200) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x2C, 0x800);
				}
			}			
		break;	
		
		case 32:
			REG_AND(MSM_CLK_CTL_BASE + 0x2C, (0x200 | 0x80) );
			if ( !(readl(MSM_CLK_CTL_BASE + 0x28) & 0xA000) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x28) & 0x200) ) {
					return rc;
				}
			}				
			REG_AND(MSM_CLK_CTL_BASE + 0x2C, 0x800);
		break;	
		
		case 33:
			REG_AND(MSM_CLK_CTL_BASE + 0x34, 0x100000 );
			if ( !(readl(MSM_CLK_CTL_BASE + 0x34) & 0x200) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x34) & 0x80) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x34, 0x800 );
				}
			}
		break;
		
		case 35:
			REG_AND(MSM_CLK_CTL_BASE + 0x34, (0x80000 | 0x20000) );
		break;
		
		case 36:
			REG_AND(MSM_CLK_CTL_BASE + 0x34, (0x10000 | 0x4000) );
		break;

		case 37:
			REG_AND(MSM_CLK_CTL_BASE + 0x34, 0x200 );
			clk_regime_off(7);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x34) & 0x100000) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x34) & 0x80) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x34, 0x800 );
				}
			}
		break;
		
		case 38:
			REG_AND(MSM_CLK_CTL_BASE + 0x34, 0x80 );
			if ( !(readl(MSM_CLK_CTL_BASE + 0x34) & 0x100000) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x34) & 0x200) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x34, 0x800 );
				}
			}
		break;	

		case 39:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x3C, 0x5FF);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x4C) & 0x200) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x4C) & 0x4000) ) {
					if ( !(readl(MSM_CLK_CTL_BASE + 0x88) & 0x200) ) {
						REG_AND(MSM_CLK_CTL_BASE + 0x3C, (0x800 | 0x100) );
					}
				}
			}			
		break;
		
		case 40:
			REG_AND(MSM_CAM_NS_REG, 0x800);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x88) & 0x100) ) {
				REG_AND(MSM_CAM_NS_REG, 0x100);
				REG_OR(MSM_CAM_NS_REG, 7);
			}
		break;
			
		case 41:
			msm_clk_regime_amss_vfe_rail_switch(0);
			if ( !(readl(MSM_CAM_NS_REG) & 0x800) ) {
				REG_AND(MSM_CAM_NS_REG, 0x100);
				REG_OR(MSM_CAM_NS_REG, 7);
			}
		break;
		
		case 42:
			REG_AND(MSM_CLK_CTL_BASE + 0x4C, 0x200);
			regval = readl(0x1FFC010);
			if ( ( regval == 0x23 ) || ( regval == 0x2D ) || ( regval == 0x2A ) ) {
			} else {
				clk_regime_wait_on_off(50, 0);
				REG_AND(MSM_CLK_CTL_BASE + 0x4C, 0x4000);
				if ( !(readl(MSM_CLK_CTL_BASE + 0x3C) & 0x200) ) {
					if ( !(readl(MSM_CLK_CTL_BASE + 0x88) & 0x200) ) {
						REG_AND(MSM_CLK_CTL_BASE + 0x4C, (0x800 | 0x100) );
					}
				}
			}			
		break;
		
		case 43:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x50, 0xC7F);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x50, 0x67F);			
		break;
		
		case 44:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x54, 0xC00);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x54, 0x600);
		break;
		
		case 45:
			REG_AND(MSM_CLK_CTL_BASE + 0x5C, (0x200 | 0x800 | 0x100) );
		break;
		
		case 47:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x68, 0xC00);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x68, 0x600);
		break;
		
		case 48:
			REG_AND(MSM_CLK_CTL_BASE + 0x70, 0x200);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x78) & 0x200) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x70) & 0x1000) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x70, (0x800 | 0x100) );
				}
			}			
		break;
		
		case 49:
			REG_AND(MSM_CLK_CTL_BASE + 0x70, 0x1000);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x78) & 0x200) ) {
				if ( !(readl(MSM_CLK_CTL_BASE +  0x70) & 0x200) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x70, (0x800 | 0x100) );
				}
			}			
		break;
		
		case 50:
			REG_AND(MSM_CLK_CTL_BASE + 0x78, (0xB200 | 0xB800 | 0xB100) );
			if ( !(readl(MSM_CLK_CTL_BASE + 0x70) & 0x200) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x70) & 0x2000) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0x70, (0x800 | 0x100) );
				}
			}			
		break;
			
		case 51:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x7C, 0xDFF);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x7C, 0x7FF);	
		break;
		
		case 52:
			REG_AND_MASK(MSM_PRPH_WEB_NS_REG, 0xC00);
			REG_AND_MASK(MSM_PRPH_WEB_NS_REG, 0x600);			
		break;

		case 53:
			REG_AND(MSM_CLK_CTL_BASE + 0x84, 0x200);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x84) & 0x100) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0x84, 0x800);
			}
		break;		
			
		case 54:
			if ( !(readl(MSM_CLK_CTL_BASE + 0x84) & 0x200) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0x84, 0x800);
			}
		break;
			
		case 55:
			REG_ANDM_OR(MSM_PRPH_WEB_NS_REG, 0x7FF, 0x200);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x4C) & 0x200) ) {
				if ( !(readl(MSM_CLK_CTL_BASE + 0x4C) & 0x4000) ) {
					if ( !(readl(MSM_CLK_CTL_BASE + 0x3C) & 0x200) ) {
						REG_AND(MSM_CLK_CTL_BASE + 0x4C, (0x800 | 0x100) );
					}
				}
			}			
		break;
		
		case 56:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x8C, 0xC7F);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x8C, 0x67F);
		break;
		
		case 57:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x90, 0x3C00);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0x90, 0x3600);
		break;
		
		case 58:
			REG_AND(MSM_CLK_CTL_BASE + 0x94, (0x40000 | 0x100000) );
		break;

		case 59:
			REG_AND(MSM_CLK_CTL_BASE + 0x94, (0x400 | 0x1000) );
		break;
		
		case 60:
			REG_AND(MSM_CLK_CTL_BASE + 0x94, (0x40 | 0x100) );
		break;
		
		case 61:
			REG_AND(MSM_CLK_CTL_BASE + 0x94, (0x4 | 0x10) );
		break;
		
		case 62:
			REG_AND(MSM_CLK_CTL_BASE + 0x9C, 0x200);
			if( (regval & 0x7) != 6 ) {
				clk_regime_wait_on_off(62, 0);
			}
			REG_AND(MSM_CLK_CTL_BASE + 0x9C, 0x800);
			if ( !(readl(MSM_CLK_CTL_BASE + 0x9C) & 0x1000) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0x9C, 0x100);
			}
		break;

		case 63:
			REG_AND(MSM_CLK_CTL_BASE + 0x9C, (0x1000 | 0x4000) );
			if ( !(readl(MSM_CLK_CTL_BASE + 0x9C) & 0x200) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0x9C, 0x100);
			}
		break;		
		
		case 64:
			REG_AND(MSM_CLK_CTL_BASE + 0xA4, 0x200);
			clk_regime_wait_on_off(63, 0);
			REG_AND(MSM_CLK_CTL_BASE + 0xA4, (0x800 | 0x100) );
		break;
		
		case 65:
			REG_AND(MSM_CLK_CTL_BASE + 0xAC, 0x200);		
			clk_regime_wait_on_off(64, 0);
			REG_AND(MSM_CLK_CTL_BASE + 0xAC, (0x800 | 0x100) );
		break;
		
		case 66:
			REG_AND(MSM_CLK_CTL_BASE + 0xB4, 0x200);		
			clk_regime_wait_on_off(65, 0);
			REG_AND(MSM_CLK_CTL_BASE + 0xB4, (0x800 | 0x100) );
		break;
		
		case 67:
			REG_AND(MSM_CLK_CTL_BASE + 0xBC, 0x200);		
			clk_regime_wait_on_off(66, 0);
			REG_AND(MSM_CLK_CTL_BASE + 0xBC, (0x800 | 0x100) );			
		break;	

		case 68:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xC0, 0xC00);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xC0, 0x600);
		break;
		
		case 69:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xC4, 0xF80);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xC4, 0xEC0);
		break;		
		
		case 70:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xC4, 0xDC0);
			clk_regime_wait_on_off(77, 0);
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xC4, 0x7C0);
			if ( !(readl(MSM_CLK_CTL_BASE + 0xC4) & 0x800) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xC100);
			}			
		break;	
		
		case 71:
			REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xD000);
			if ( !(readl(MSM_CLK_CTL_BASE + 0xCC) & 0x200) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xC800);
				if ( !(readl(MSM_CLK_CTL_BASE + 0xC4) & 0x800) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xC100);
				}
			}			
		break;
		
		case 72:
			REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xC200);
			if ( !(readl(MSM_CLK_CTL_BASE + 0xCC) & 0x1000) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xC800);
				if ( !(readl(MSM_CLK_CTL_BASE + 0xC4) & 0x800) ) {
					REG_AND(MSM_CLK_CTL_BASE + 0xCC, 0xC100);
				}
			}			
		break;
		
		case 73:
			REG_AND(MSM_CLK_CTL_BASE + 0xE0, (0x20 | 0x10) );
		break;
		
		case 74:
			REG_AND(MSM_CLK_CTL_BASE + 0xE0, (0x800 | 0x400) );
		break;
		
		case 75:
			REG_AND(MSM_CLK_CTL_BASE + 0xE0, (0x20000 | 0x10000) );
		break;
		
		case 76:
			REG_AND(MSM_CLK_CTL_BASE + 0xD4, (0x200 | 0x800 | 0x100) );
			REG_AND(MSM_CLK_CTL_BASE, 0x20000);
		break;
	
		case 77:
			REG_AND(MSM_CLK_CTL_BASE + 0xDC, (0x200 | 0x800 | 0x100) );
			REG_AND(MSM_CLK_CTL_BASE, 0xC4000000);
		break;
		
		case 78:
			REG_AND(MSM_CLK_CTL_BASE + 0xE0, 0xFA000000);
		break;
		
		case 79:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xE0, 0x3FFFFFF);
		break;
	
		case 80:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xE8, 0xFFEFFF);
			if ( !(readl(MSM_CLK_CTL_BASE + 0xE8) & 0x200) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0xE8, (0x800 | 0x100) );
			}			
		break;
	
		case 81:
			REG_AND_MASK(MSM_CLK_CTL_BASE + 0xE8, 0xFFFDFF);			
			if ( !(readl(MSM_CLK_CTL_BASE + 0xE8) & 0x1000) ) {
				REG_AND(MSM_CLK_CTL_BASE + 0xE8, (0x800 | 0x100) );
			}
			clk_regime_off_stateful(0xC);			
		break;

		case 82:
			REG_AND(MSM_CLK_CTL_BASE + 0xE8, (0x200 | 0x800 | 0x100) );		
			clk_regime_off_stateful(0xC);
		break;
		
		case 85:
			REG_AND(MSM_CLK_CTL_BASE, 0xC2000000);
			clk_regime_off_stateful(0xC);			
		break;
		
		case 88:
			msm_clk_regime_amss_vdc_rail_switch(0);
		break;
		
		default:
			printk("clk_regime_msm_off: Clock branch out of range (%d)", clock);
            rc = -1;
		break;
	}	

    return rc;
}

static int clk_regime_enable(int clock_id, int paramR1)
{
    int rc = -EIO;

	switch(clock_id) {
		case 89:
			rc = clk_regime_enable(73, paramR1);
		break;
		
		case 90:
			rc = clk_regime_enable(48, paramR1);
			rc = clk_regime_enable(49, paramR1);
			rc = clk_regime_enable(50, paramR1);
		break;
		
		case 91:
			rc = clk_regime_enable(37, paramR1);
			rc = clk_regime_enable(35, paramR1);
			rc = clk_regime_enable(33, paramR1);
			rc = clk_regime_enable(36, paramR1);
		break;
		
		case 92:
			rc = clk_regime_enable(81, paramR1);
			rc = clk_regime_enable(80, paramR1);
		break;
		
		case 93:
                        rc = 0;
		break;
		
		default:
			//TODO : clk_regime_msm_src_request(clock, 1);
			rc = clk_regime_on(clock_id);
		break;
	}

    return rc;
}

int clk_regime_disable(int clock_id, int paramR1)
{
    int rc = -EIO;

	switch(clock_id) {
		case 89:
			rc = clk_regime_disable(73, paramR1);
		break;
		
		case 90:
			rc = clk_regime_disable(48, paramR1);
			rc = clk_regime_disable(49, paramR1);
			rc = clk_regime_disable(50, paramR1);
		break;
		
		case 91:
			rc = clk_regime_disable(37, paramR1);
			rc = clk_regime_disable(35, paramR1);
			rc = clk_regime_disable(33, paramR1);
			rc = clk_regime_disable(36, paramR1);
		break;
		
		case 92:
			rc = clk_regime_disable(81, paramR1);
			rc = clk_regime_disable(80, paramR1);
		break;
		
		case 93:
                        rc = -1;
		break;
		
		default:
			rc = clk_regime_off(clock_id);			
			// TODO : clk_regime_msm_src_request(clock, 0);
		break;
	}

    return rc;
}

void clk_regime_amss_vfe_set_speed_normalized(int normalized_freq_index)
{
	int clock_0x60_status, clock_0x37_status;

	clock_0x60_status = clk_regime_is_on(96);
	clock_0x37_status = clk_regime_is_on(55);

	if( (clock_0x60_status | clock_0x37_status)  != 0 ) {
		if ( !(readl(MSM_CAM_NS_REG) & 0x4000) ) {
			REG_AND(MSM_CAM_NS_REG, (0x200 | 0x800 | 0x2000 | 0x100) );
			mdelay(100);
		}
	}
	
	REG_OR(MSM_CAM_NS_REG, 0x80);	
	REG_AND_OR(MSM_CAM_NS_REG, 0x7, (clocks_params[normalized_freq_index].source & 0x7));
	REG_AND_OR(MSM_CAM_NS_REG, 0x18, (clocks_params[normalized_freq_index].src_div & 0x3) << 3);		
	REG_ANDM_OR(MSM_CAM_NS_REG, 0x0FFFF, (clocks_params[normalized_freq_index].ns << 0x10));
	REG_SET(MSM_CAM_MD_REG, clocks_params[normalized_freq_index].md);

	if( clocks_params[normalized_freq_index].ns == 0 ) {
		REG_AND(MSM_CAM_NS_REG, 0x60);	
	} else {
		REG_AND_ADD(MSM_CAM_NS_REG, 0x60, 0x40);
	}

	REG_AND(MSM_CAM_NS_REG, 0x80);	    

	clk_40_41_freq_ndx = normalized_freq_index;
	if( (clock_0x60_status | clock_0x37_status) == 0 ) {
		REG_OR(MSM_CAM_NS_REG, 0x7);	
	} else {
		// Related to pll switch on / off.

		if( (readl(MSM_CAM_NS_REG) & 0x60) != 0 ) {
			REG_OR(MSM_CAM_NS_REG, 0x100);	
		}
			
		REG_OR(MSM_CAM_NS_REG, 0x2000);
		
		if( clock_0x60_status != 0 ) {
			REG_OR(MSM_CAM_NS_REG, 0x200);
		}
		if( clock_0x37_status != 0 ) {		
			REG_OR(MSM_CAM_NS_REG, 0x800);
		}
	}
}

/***************************************************************************************************
 *  Exported functions
 ***************************************************************************************************/
int msm_clk_regime_amss_enable(int id)
{
    int rc = -EIO;

    if( id == 0x14 ) {
        // TODO : set_clocks_bits(0, 1);
    } else if ( id == 0x36 ) {
        printk("Turn-on GRP clock...\r\n");
        // TODO : set_clocks_bits(0xC, 1);
    } else if ( id == 0x38 ) {
        printk("Turn-on MDDI host clock...\r\n");
        // TODO : set_clocks_bits(0xA, 1);
        // TODO : set_clocks_bits(0, 1);
     } else {

        int amss_clk_id = msm_rpc_clk_regime_get_id(id);

        if( amss_clk_id == -1 ) {
            printk(KERN_ERR "%s : unknown clock ID %d for use with clock regime\n", __func__, id);
            return -EIO;
        }

        rc = clk_regime_enable(amss_clk_id, 1);
    }
    
    return rc;
}
EXPORT_SYMBOL(msm_clk_regime_amss_enable);

int msm_clk_regime_amss_disable(int id)
{
    int rc = -EIO;
    int amss_clk_id = msm_rpc_clk_regime_get_id(id);

    if( amss_clk_id == -1 ) {
        printk(KERN_ERR "%s : unknown clock ID %d for use with clock regime\n", __func__, id);
        return -EIO;
    }

    rc = clk_regime_disable(amss_clk_id, 1);

    return rc;
}
EXPORT_SYMBOL(msm_clk_regime_amss_disable);

int msm_clk_regime_amss_is_on(int id)
{
    int rc = -EIO;
    int amss_clk_id = msm_rpc_clk_regime_get_id(id);

    if( amss_clk_id == -1 ) {
        printk(KERN_ERR "%s : unknown clock ID %d for use with clock regime\n", __func__, id);
        return -EIO;
    }

    CRGM_DBG("%s not yet fully tested. Clock id used by clk_regime_is_on is not the same as enable/disable\n", __func__);
    clk_regime_is_on(amss_clk_id);

    return rc;
}
EXPORT_SYMBOL(msm_clk_regime_amss_is_on);



/***************************************************************************************************
 *  Specific Clk regime ops
 ***************************************************************************************************/
int msm_clk_regime_amss_vfe_rail_switch(unsigned int bOn)
{
    int i = 0;
    int status = 0;

    if( bOn ) {
        REG_ANDM_OR(MSM_AXI_RESET, 0x3FFF, 1);
        REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x210, 0x1FFF, 1);
        REG_SET(MSM_VDD_VFE_GFS_CTL, 0x11F);
        mdelay(20);
        REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 4);
        if( readl(MSM_CAM_NS_REG) & 0x060 ) {
            REG_OR(MSM_CAM_NS_REG, 0x100);
        }
        REG_OR(MSM_CAM_NS_REG, 0x2200);
        REG_AND_MASK(MSM_RAIL_CLAMP_IO, 0x37);       

        REG_AND_MASK(MSM_AXI_BASE + 0x20080, 0xFE);

        REG_AND_MASK(MSM_AXI_RESET, 0x3FFE);
        REG_AND_MASK(MSM_APPS_RESET, 0x1FFE);   
    } else {
        REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 4);
        if( readl(MSM_CAM_NS_REG) & 0x060 ) {
            REG_OR(MSM_CAM_NS_REG, 0x200);
        }        
        REG_OR(MSM_CAM_NS_REG, 0x2200);

        REG_OR( MSM_AXI_BASE + 0x20080, 0x1 );
		while ( status == 0 && i < 100) {
			i++;
			status = readl(MSM_AXI_BASE + 0x20084) & 0x1;
		}

        REG_ANDM_OR(MSM_AXI_RESET, 0x3FFF, 1);
        REG_ANDM_OR(MSM_APPS_RESET, 0x1FFF, 1);
        REG_AND(MSM_CAM_NS_REG, 0x200);
        if( msm_clk_regime_amss_is_on(0x37) ) {
            REG_AND(MSM_CAM_NS_REG, 0x2100);
        }
        REG_ANDM_OR(MSM_RAIL_CLAMP_IO, 0x3F, 8);
        REG_SET(MSM_VDD_VFE_GFS_CTL, 0x1F);
        clk_regime_off_stateful(0xC);
    }

    return 0;
}
EXPORT_SYMBOL(msm_clk_regime_amss_vfe_rail_switch);

int msm_clk_regime_amss_vfe_sel_src(unsigned int flags)
{
    int src = 0;
    int clk_status = 0;

    if (flags & 0x100) {               //External clock
        src = 1;
    } else if (flags & 0x200) {        //Internal clock
        src = 0;
    }     

    clk_status = clk_regime_is_on(0x60);
    if( clk_status != 0 ) {
        REG_AND(MSM_CAM_NS_REG, 0x200);
        mdelay(100);
    }

    REG_AND(MSM_CAM_NS_REG, 0x4000);
    REG_OR(MSM_CAM_NS_REG, (src << 0xE) );

    if ( clk_status != 0 ) {
        REG_OR(MSM_CAM_NS_REG, 0x200);
    } 

    return 0;
}
EXPORT_SYMBOL(msm_clk_regime_amss_vfe_sel_src);

int msm_clk_regime_amss_set_flags(int id, int flags)
{
    int rc = -EIO;
    int amss_clk_id = msm_rpc_clk_regime_get_id(id);

    if( amss_clk_id == -1 ) {
        printk(KERN_ERR "%s : unknown clock ID %d for use with clock regime\n", __func__, id);
        return -EIO;
    }

    switch( amss_clk_id ) {
        case VFE_CLK:
            rc = msm_clk_regime_amss_vfe_sel_src(flags);
        break;

        default:
            printk(KERN_ERR "%s : clock ID %d not yet supported\n", __func__, amss_clk_id);
        break;
    }

    return rc;
}
EXPORT_SYMBOL(msm_clk_regime_amss_set_flags);

int msm_clk_regime_amss_vfe_set_rate(unsigned rate)
{
	int freq_diff = 0xFFFFFFF;	// difference between requested frequency and most close "normalized" frequency
	int tmp_freq_diff;          // tmp var for calculus
	int normalized_freq_index = 18; // default frequency index
	int index = 0;

	while( (index < 36) && (freq_diff != 0) )
	{
		tmp_freq_diff = abs(rate - clocks_params[index].frequency);
        //CRGM_DBG("req : %d, tested : %d, diff = %d\n", rate, clocks_params[index].frequency, tmp_freq_diff);
	
		if ( tmp_freq_diff < freq_diff ) {
			freq_diff = tmp_freq_diff;
			normalized_freq_index = index;
		}
		index += 1;		
	}
	
	if ( freq_diff != 0 ) {
		printk("VFE speed %d Hz not supported, using %d Hz\n", rate, clocks_params[normalized_freq_index].frequency);			
	} 
	clk_regime_amss_vfe_set_speed_normalized(normalized_freq_index);

    CRGM_DBG("%s : Frequency was set to %d Hz\n", __func__, clocks_params[clk_40_41_freq_ndx].frequency);

	return clocks_params[clk_40_41_freq_ndx].frequency;
}
EXPORT_SYMBOL(msm_clk_regime_amss_vfe_set_rate);

int msm_clk_regime_amss_set_rate(int id, int rate)
{
    int rc = -EIO;
    int amss_clk_id = msm_rpc_clk_regime_get_id(id);

    if( amss_clk_id == -1 ) {
        printk(KERN_ERR "%s : unknown clock ID %d for use with clock regime\n", __func__, id);
        return -EIO;
    }

    switch( amss_clk_id ) {
        case VFE_CLK:
            rc = msm_clk_regime_amss_vfe_set_rate(rate);
        break;

        default:
            printk(KERN_ERR "%s : clock ID %d not yet supported\n", __func__, amss_clk_id);
        break;
    }

    return rc;
}
EXPORT_SYMBOL(msm_clk_regime_amss_set_rate);

/* Control VDC rail */
int msm_clk_regime_amss_vdc_rail_switch(unsigned int bOn)
{
	int i = 0;
	int status = 0;
    
	if ( bOn != 0 ) {
		REG_OR( MSM_CLK_CTL_BASE + 0xF0, 0x800);		
		if (readl(MSM_CLK_CTL_BASE + 0xF0) & 0x60 ) {
			REG_OR( MSM_CLK_CTL_BASE + 0xF0, 0x100);
		}
		REG_OR( MSM_CLK_CTL_BASE + 0xF0, 0x200);
		
		REG_ANDM_OR( MSM_CLK_CTL_BASE + 0x34, 0x7FFFFF, 0x100000);
		REG_OR( MSM_CLK_CTL_BASE, 0x8);
		REG_ANDM_OR( MSM_AXI_RESET, 0x3FFF, 0x2);
		REG_ANDM_OR( MSM_ROW_RESET, 0x7FFFFFF, 0x1);
		REG_ANDM_OR( MSM_APPS_RESET, 0x1FFF, 0x80);
		
		REG_SET( MSM_VDD_VDC_GFS_CTL, 0x11f );
		mdelay(2);
		
		REG_AND( MSM_RAIL_CLAMP_IO, 0x2);
		
		REG_AND( MSM_AXI_BASE + 0x10080, 0x4 );
		
		REG_AND( MSM_AXI_RESET, 0x2 );
		REG_AND( MSM_ROW_RESET, 1);
		REG_AND( MSM_APPS_RESET, 0x80);
	} else {
		REG_OR( MSM_CLK_CTL_BASE + 0xF0, 0x800);
		if (readl(MSM_CLK_CTL_BASE + 0xF0) & 0x60 ) {
			REG_OR( MSM_CLK_CTL_BASE + 0xF0, 0x100);
		}
		REG_OR( MSM_CLK_CTL_BASE + 0xF0, 0x200);
		REG_OR( MSM_CLK_CTL_BASE, 0x8);

		REG_OR( MSM_AXI_BASE + 0x10080, 0x4 );
		
		while ( status == 0 && i < 100) {
			i++;
			status = readl( MSM_AXI_BASE + 0x10084 ) & 0x4;
		}
		
		REG_OR( MSM_AXI_RESET, 0x2 );
		REG_OR( MSM_ROW_RESET, 0x1 );

		REG_AND( MSM_AXI_BASE + 0xF0, 0x200 );
		REG_AND( MSM_AXI_BASE + 0xF0, 0x100 );
		REG_AND( MSM_AXI_BASE + 0xF0, 0x800 );
		
		REG_OR( MSM_RAIL_CLAMP_IO, 0x2 );

		REG_SET( MSM_VDD_VDC_GFS_CTL, 0x1f );

		clk_regime_off_stateful(12);
	}

    return 0;
}
EXPORT_SYMBOL(msm_clk_regime_amss_vdc_rail_switch);

/* Control GRP rail */
int msm_clk_regime_amss_grp_rail_switch(unsigned int bOn)
{
	int i = 0;
	int status = 0;

	if ( bOn != 0 ) {
		REG_OR( MSM_AXI_RESET, 0x20 );
		REG_OR( MSM_ROW_RESET, 0x20000 );
		REG_SET( MSM_VDD_GRP_GFS_CTL, 0x11f );
		mdelay( 20 );                               // very rough delay

		REG_OR( MSM_GRP_NS_REG, 0x800 );
		REG_OR( MSM_GRP_NS_REG, 0x80 );
		REG_OR( MSM_GRP_NS_REG, 0x200 );

		REG_OR( MSM_CLK_CTL_BASE, 0x8 );            // grp idx

		REG_AND( MSM_RAIL_CLAMP_IO, 0x4 );

        REG_AND( MSM_AXI_BASE + 0x10080, 0x1 );

		REG_AND( MSM_AXI_RESET, 0x20 );
		REG_AND( MSM_ROW_RESET, 0x20000 );
	} else {
		REG_OR( MSM_GRP_NS_REG, 0x800 );
		REG_OR( MSM_GRP_NS_REG, 0x80 );
		REG_OR( MSM_GRP_NS_REG, 0x200 );

		REG_OR(  MSM_CLK_CTL_BASE, 0x8 );           // grp idx

		REG_OR( MSM_AXI_BASE + 0x10080, 0x1 );

		while ( status == 0 && i < 100) {
			i++;
			status = readl( MSM_AXI_BASE + 0x10084 ) & 0x1;
		}

		REG_OR( MSM_AXI_RESET, 0x20 );
		REG_OR( MSM_ROW_RESET, 0x20000 );

		REG_AND( MSM_GRP_NS_REG, 0x800 );
		REG_AND( MSM_GRP_NS_REG, 0x80 );
		REG_AND( MSM_GRP_NS_REG, 0x200 );

		REG_OR( MSM_RAIL_CLAMP_IO, 0x4 );           // grp clk ramp

		REG_SET( MSM_VDD_GRP_GFS_CTL, 0x1f );

		/*
		control = readl( MSM_VDD_VDC_GFS_CTL );

		if ( control & 0x100 ) {
			REG_AND( MSM_CLK_CTL_BASE, 0x8 );       // grp idx
		}
		*/
		clk_regime_off_stateful(12);
	}

    return 0;
}
EXPORT_SYMBOL(msm_clk_regime_amss_grp_rail_switch);



/***************************************************************************************************
 *  Debug FS
 ***************************************************************************************************/
int debug_read_amss_clk(char *buf, int max)
{
    int n, i = 0;

    for (n=0; n < 90; n++) {
#if 0
        if( msm_rpc_clk_regime_cmd_simple_reply_simple(CLK_REGIME_RPC_AMSS_IS_ON, n) ) {
            i += scnprintf(buf + i, max - i, "Clock %d : freq = %d KHz\n", n, 
                msm_rpc_clk_regime_cmd_simple_reply_simple(CLK_REGIME_RPC_AMSS_MSM_GET_CLK_FREQ_KHZ, n) );
        }
#endif
    }

	return i;
}

ssize_t debug_write_amss(struct file *file, const char __user *buf,
		     size_t count, loff_t *ppos)
{
    char *pbuffer = kzalloc(count, GFP_KERNEL);

    if ( copy_from_user(pbuffer, buf, count) ) {
		return -EFAULT;
	}
    
    while( (pbuffer[0] != 0) && (pbuffer[0] != '\n') ){ 
        switch(pbuffer[0]) {      
            case '0':
                msm_clk_regime_amss_vfe_rail_switch(0);
            break;

            case '1':
                msm_clk_regime_amss_vfe_rail_switch(1);
            break;

            case '2':
                msm_clk_regime_amss_grp_rail_switch(0);
            break;

            case '3':
                msm_clk_regime_amss_grp_rail_switch(1);
            break;

            case 's':
                msm_clk_regime_amss_vfe_set_rate(23500000);
            break;

            case 't':
                msm_clk_regime_amss_vfe_set_rate(16300000);
            break;

            default:
        	printk(KERN_ERR "msm_clocks: cmd '%c' not supported\n", pbuffer[0]);
            break;
	    }
        pbuffer++;
    };

	return count;
}
