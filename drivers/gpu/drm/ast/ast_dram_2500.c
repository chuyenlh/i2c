// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>

#include <drm/drm_print.h>

#include "ast_dram_tables.h"
#include "ast_drv.h"

#define TIMEOUT		5000000

u32 ast_mindwm(struct ast_private *ast, u32 r)
{
	u32 data;

	ast_write32(ast, 0xf004, r & 0xffff0000);
	ast_write32(ast, 0xf000, 0x1);

	do {
		data = ast_read32(ast, 0xf004) & 0xffff0000;
	} while (data != (r & 0xffff0000));
	return ast_read32(ast, 0x10000 + (r & 0x0000ffff));
}

void ast_moutdwm(struct ast_private *ast, u32 r, u32 v)
{
	u32 data;

	ast_write32(ast, 0xf004, r & 0xffff0000);
	ast_write32(ast, 0xf000, 0x1);
	do {
		data = ast_read32(ast, 0xf004) & 0xffff0000;
	} while (data != (r & 0xffff0000));
	ast_write32(ast, 0x10000 + (r & 0x0000ffff), v);
}

static bool mmc_test(struct ast_private *ast, u32 datagen, u8 test_ctl)
{
	u32 data, timeout;

	ast_moutdwm(ast, 0x1e6e0070, 0x00000000);
	ast_moutdwm(ast, 0x1e6e0070, (datagen << 3) | test_ctl);
	timeout = 0;
	do {
		data = ast_mindwm(ast, 0x1e6e0070) & 0x3000;
		if (data & 0x2000)
			return false;
		if (++timeout > TIMEOUT) {
			ast_moutdwm(ast, 0x1e6e0070, 0x00000000);
			return false;
		}
	} while (!data);
	ast_moutdwm(ast, 0x1e6e0070, 0x0);
	return true;
}

static bool mmc_test_burst(struct ast_private *ast, u32 datagen)
{
	return mmc_test(ast, datagen, 0xc1);
}

static bool mmc_test_single_2500(struct ast_private *ast, u32 datagen)
{
	return mmc_test(ast, datagen, 0x85);
}

static bool cbr_test_2500(struct ast_private *ast)
{
	ast_moutdwm(ast, 0x1E6E0074, 0x0000FFFF);
	ast_moutdwm(ast, 0x1E6E007C, 0xFF00FF00);
	if (!mmc_test_burst(ast, 0))
		return false;
	if (!mmc_test_single_2500(ast, 0))
		return false;
	return true;
}

static bool ddr_test_2500(struct ast_private *ast)
{
	ast_moutdwm(ast, 0x1E6E0074, 0x0000FFFF);
	ast_moutdwm(ast, 0x1E6E007C, 0xFF00FF00);
	if (!mmc_test_burst(ast, 0))
		return false;
	if (!mmc_test_burst(ast, 1))
		return false;
	if (!mmc_test_burst(ast, 2))
		return false;
	if (!mmc_test_burst(ast, 3))
		return false;
	if (!mmc_test_single_2500(ast, 0))
		return false;
	return true;
}

static void ddr_init_common_2500(struct ast_private *ast)
{
	ast_moutdwm(ast, 0x1E6E0034, 0x00020080);
	ast_moutdwm(ast, 0x1E6E0008, 0x2003000F);
	ast_moutdwm(ast, 0x1E6E0038, 0x00000FFF);
	ast_moutdwm(ast, 0x1E6E0040, 0x88448844);
	ast_moutdwm(ast, 0x1E6E0044, 0x24422288);
	ast_moutdwm(ast, 0x1E6E0048, 0x22222222);
	ast_moutdwm(ast, 0x1E6E004C, 0x22222222);
	ast_moutdwm(ast, 0x1E6E0050, 0x80000000);
	ast_moutdwm(ast, 0x1E6E0208, 0x00000000);
	ast_moutdwm(ast, 0x1E6E0218, 0x00000000);
	ast_moutdwm(ast, 0x1E6E0220, 0x00000000);
	ast_moutdwm(ast, 0x1E6E0228, 0x00000000);
	ast_moutdwm(ast, 0x1E6E0230, 0x00000000);
	ast_moutdwm(ast, 0x1E6E02A8, 0x00000000);
	ast_moutdwm(ast, 0x1E6E02B0, 0x00000000);
	ast_moutdwm(ast, 0x1E6E0240, 0x86000000);
	ast_moutdwm(ast, 0x1E6E0244, 0x00008600);
	ast_moutdwm(ast, 0x1E6E0248, 0x80000000);
	ast_moutdwm(ast, 0x1E6E024C, 0x80808080);
}

static void ddr_phy_init_2500(struct ast_private *ast)
{
	u32 data, pass, timecnt;

	pass = 0;
	ast_moutdwm(ast, 0x1E6E0060, 0x00000005);
	while (!pass) {
		for (timecnt = 0; timecnt < TIMEOUT; timecnt++) {
			data = ast_mindwm(ast, 0x1E6E0060) & 0x1;
			if (!data)
				break;
		}
		if (timecnt != TIMEOUT) {
			data = ast_mindwm(ast, 0x1E6E0300) & 0x000A0000;
			if (!data)
				pass = 1;
		}
		if (!pass) {
			ast_moutdwm(ast, 0x1E6E0060, 0x00000000);
			usleep_range(10, 20); /* delay 10 us */
			ast_moutdwm(ast, 0x1E6E0060, 0x00000005);
		}
	}

	ast_moutdwm(ast, 0x1E6E0060, 0x00000006);
}

/*
 * Check DRAM Size
 * 1Gb : 0x80000000 ~ 0x87FFFFFF
 * 2Gb : 0x80000000 ~ 0x8FFFFFFF
 * 4Gb : 0x80000000 ~ 0x9FFFFFFF
 * 8Gb : 0x80000000 ~ 0xBFFFFFFF
 */
static void check_dram_size_2500(struct ast_private *ast, u32 tRFC)
{
	u32 reg_04, reg_14;

	reg_04 = ast_mindwm(ast, 0x1E6E0004) & 0xfffffffc;
	reg_14 = ast_mindwm(ast, 0x1E6E0014) & 0xffffff00;

	ast_moutdwm(ast, 0xA0100000, 0x41424344);
	ast_moutdwm(ast, 0x90100000, 0x35363738);
	ast_moutdwm(ast, 0x88100000, 0x292A2B2C);
	ast_moutdwm(ast, 0x80100000, 0x1D1E1F10);

	/* Check 8Gbit */
	if (ast_mindwm(ast, 0xA0100000) == 0x41424344) {
		reg_04 |= 0x03;
		reg_14 |= (tRFC >> 24) & 0xFF;
		/* Check 4Gbit */
	} else if (ast_mindwm(ast, 0x90100000) == 0x35363738) {
		reg_04 |= 0x02;
		reg_14 |= (tRFC >> 16) & 0xFF;
		/* Check 2Gbit */
	} else if (ast_mindwm(ast, 0x88100000) == 0x292A2B2C) {
		reg_04 |= 0x01;
		reg_14 |= (tRFC >> 8) & 0xFF;
	} else {
		reg_14 |= tRFC & 0xFF;
	}
	ast_moutdwm(ast, 0x1E6E0004, reg_04);
	ast_moutdwm(ast, 0x1E6E0014, reg_14);
}

static void enable_cache_2500(struct ast_private *ast)
{
	u32 reg_04, data;

	reg_04 = ast_mindwm(ast, 0x1E6E0004);
	ast_moutdwm(ast, 0x1E6E0004, reg_04 | 0x1000);

	do
		data = ast_mindwm(ast, 0x1E6E0004);
	while (!(data & 0x80000));
	ast_moutdwm(ast, 0x1E6E0004, reg_04 | 0x400);
}

static void set_mpll_2500(struct ast_private *ast)
{
	u32 addr, data, param;

	/* Reset MMC */
	ast_moutdwm(ast, 0x1E6E0000, 0xFC600309);
	ast_moutdwm(ast, 0x1E6E0034, 0x00020080);
	for (addr = 0x1e6e0004; addr < 0x1e6e0090;) {
		ast_moutdwm(ast, addr, 0x0);
		addr += 4;
	}
	ast_moutdwm(ast, 0x1E6E0034, 0x00020000);

	ast_moutdwm(ast, 0x1E6E2000, 0x1688A8A8);
	data = ast_mindwm(ast, 0x1E6E2070) & 0x00800000;
	if (data) {
		/* CLKIN = 25MHz */
		param = 0x930023E0;
		ast_moutdwm(ast, 0x1E6E2160, 0x00011320);
	} else {
		/* CLKIN = 24MHz */
		param = 0x93002400;
	}
	ast_moutdwm(ast, 0x1E6E2020, param);
	usleep_range(100, 150);
}

static void reset_mmc_2500(struct ast_private *ast)
{
	u32 data;

	ast_moutdwm(ast, 0x1E78505C, 0x00000004);
	ast_moutdwm(ast, 0x1E785044, 0x00000001);
	ast_moutdwm(ast, 0x1E785048, 0x00004755);
	ast_moutdwm(ast, 0x1E78504C, 0x00000013);
	mdelay(100);
	ast_moutdwm(ast, 0x1E78505c, 0x023FFFF3);
	ast_moutdwm(ast, 0x1E785054, 0x00000077);
	do {
		ast_moutdwm(ast, 0x1E6E0000, 0xFC600309);
		data = ast_mindwm(ast, 0x1E6E0000);
	} while (data == 0);
	ast_moutdwm(ast, 0x1E6E0034, 0x00020000);
}

static void ddr3_init_2500(struct ast_private *ast, const u32 *ddr_table)
{
	ast_moutdwm(ast, 0x1E6E0004, 0x00000303);
	ast_moutdwm(ast, 0x1E6E0010, ddr_table[REGIDX_010]);
	ast_moutdwm(ast, 0x1E6E0014, ddr_table[REGIDX_014]);
	ast_moutdwm(ast, 0x1E6E0018, ddr_table[REGIDX_018]);
	ast_moutdwm(ast, 0x1E6E0020, ddr_table[REGIDX_020]);	     /* MODEREG4/6 */
	ast_moutdwm(ast, 0x1E6E0024, ddr_table[REGIDX_024]);	     /* MODEREG5 */
	ast_moutdwm(ast, 0x1E6E002C, ddr_table[REGIDX_02C] | 0x100); /* MODEREG0/2 */
	ast_moutdwm(ast, 0x1E6E0030, ddr_table[REGIDX_030]);	     /* MODEREG1/3 */

	/* DDR PHY Setting */
	ast_moutdwm(ast, 0x1E6E0200, 0x02492AAE);
	ast_moutdwm(ast, 0x1E6E0204, 0x00001001);
	ast_moutdwm(ast, 0x1E6E020C, 0x55E00B0B);
	ast_moutdwm(ast, 0x1E6E0210, 0x20000000);
	ast_moutdwm(ast, 0x1E6E0214, ddr_table[REGIDX_214]);
	ast_moutdwm(ast, 0x1E6E02E0, ddr_table[REGIDX_2E0]);
	ast_moutdwm(ast, 0x1E6E02E4, ddr_table[REGIDX_2E4]);
	ast_moutdwm(ast, 0x1E6E02E8, ddr_table[REGIDX_2E8]);
	ast_moutdwm(ast, 0x1E6E02EC, ddr_table[REGIDX_2EC]);
	ast_moutdwm(ast, 0x1E6E02F0, ddr_table[REGIDX_2F0]);
	ast_moutdwm(ast, 0x1E6E02F4, ddr_table[REGIDX_2F4]);
	ast_moutdwm(ast, 0x1E6E02F8, ddr_table[REGIDX_2F8]);
	ast_moutdwm(ast, 0x1E6E0290, 0x00100008);
	ast_moutdwm(ast, 0x1E6E02C0, 0x00000006);

	/* Controller Setting */
	ast_moutdwm(ast, 0x1E6E0034, 0x00020091);

	/* Wait DDR PHY init done */
	ddr_phy_init_2500(ast);

	ast_moutdwm(ast, 0x1E6E0120, ddr_table[REGIDX_PLL]);
	ast_moutdwm(ast, 0x1E6E000C, 0x42AA5C81);
	ast_moutdwm(ast, 0x1E6E0034, 0x0001AF93);

	check_dram_size_2500(ast, ddr_table[REGIDX_RFC]);
	enable_cache_2500(ast);
	ast_moutdwm(ast, 0x1E6E001C, 0x00000008);
	ast_moutdwm(ast, 0x1E6E0038, 0xFFFFFF00);
}

static void ddr4_init_2500(struct ast_private *ast, const u32 *ddr_table)
{
	u32 data, data2, pass, retrycnt;
	u32 ddr_vref, phy_vref;
	u32 min_ddr_vref = 0, min_phy_vref = 0;
	u32 max_ddr_vref = 0, max_phy_vref = 0;

	ast_moutdwm(ast, 0x1E6E0004, 0x00000313);
	ast_moutdwm(ast, 0x1E6E0010, ddr_table[REGIDX_010]);
	ast_moutdwm(ast, 0x1E6E0014, ddr_table[REGIDX_014]);
	ast_moutdwm(ast, 0x1E6E0018, ddr_table[REGIDX_018]);
	ast_moutdwm(ast, 0x1E6E0020, ddr_table[REGIDX_020]);	     /* MODEREG4/6 */
	ast_moutdwm(ast, 0x1E6E0024, ddr_table[REGIDX_024]);	     /* MODEREG5 */
	ast_moutdwm(ast, 0x1E6E002C, ddr_table[REGIDX_02C] | 0x100); /* MODEREG0/2 */
	ast_moutdwm(ast, 0x1E6E0030, ddr_table[REGIDX_030]);	     /* MODEREG1/3 */

	/* DDR PHY Setting */
	ast_moutdwm(ast, 0x1E6E0200, 0x42492AAE);
	ast_moutdwm(ast, 0x1E6E0204, 0x09002800);					/* modify at V1.3 */
	ast_moutdwm(ast, 0x1E6E020C, 0x55E00B0B);
	ast_moutdwm(ast, 0x1E6E0210, 0x20000000);
	ast_moutdwm(ast, 0x1E6E0214, ddr_table[REGIDX_214]);
	ast_moutdwm(ast, 0x1E6E02E0, ddr_table[REGIDX_2E0]);
	ast_moutdwm(ast, 0x1E6E02E4, ddr_table[REGIDX_2E4]);
	ast_moutdwm(ast, 0x1E6E02E8, ddr_table[REGIDX_2E8]);
	ast_moutdwm(ast, 0x1E6E02EC, ddr_table[REGIDX_2EC]);
	ast_moutdwm(ast, 0x1E6E02F0, ddr_table[REGIDX_2F0]);
	ast_moutdwm(ast, 0x1E6E02F4, ddr_table[REGIDX_2F4]);
	ast_moutdwm(ast, 0x1E6E02F8, ddr_table[REGIDX_2F8]);
	ast_moutdwm(ast, 0x1E6E0290, 0x00100008);
	ast_moutdwm(ast, 0x1E6E02C4, 0x3C183C3C);
	ast_moutdwm(ast, 0x1E6E02C8, 0x00631E0E);

	/* Controller Setting */
	ast_moutdwm(ast, 0x1E6E0034, 0x0001A991);

	/* Train PHY Vref first */
	pass = 0;

	for (retrycnt = 0; retrycnt < 4 && pass == 0; retrycnt++) {
		max_phy_vref = 0x0;
		pass = 0;
		ast_moutdwm(ast, 0x1E6E02C0, 0x00001C06);
		for (phy_vref = 0x40; phy_vref < 0x80; phy_vref++) {
			ast_moutdwm(ast, 0x1E6E000C, 0x00000000);
			ast_moutdwm(ast, 0x1E6E0060, 0x00000000);
			ast_moutdwm(ast, 0x1E6E02CC, phy_vref | (phy_vref << 8));
			/* Fire DFI Init */
			ddr_phy_init_2500(ast);
			ast_moutdwm(ast, 0x1E6E000C, 0x00005C01);
			if (cbr_test_2500(ast)) {
				pass++;
				data = ast_mindwm(ast, 0x1E6E03D0);
				data2 = data >> 8;
				data  = data & 0xff;
				if (data > data2)
					data = data2;
				if (max_phy_vref < data) {
					max_phy_vref = data;
					min_phy_vref = phy_vref;
				}
			} else if (pass > 0) {
				break;
			}
		}
	}
	ast_moutdwm(ast, 0x1E6E02CC, min_phy_vref | (min_phy_vref << 8));

	/* Train DDR Vref next */
	pass = 0;

	for (retrycnt = 0; retrycnt < 4 && pass == 0; retrycnt++) {
		min_ddr_vref = 0xFF;
		max_ddr_vref = 0x0;
		pass = 0;
		for (ddr_vref = 0x00; ddr_vref < 0x40; ddr_vref++) {
			ast_moutdwm(ast, 0x1E6E000C, 0x00000000);
			ast_moutdwm(ast, 0x1E6E0060, 0x00000000);
			ast_moutdwm(ast, 0x1E6E02C0, 0x00000006 | (ddr_vref << 8));
			/* Fire DFI Init */
			ddr_phy_init_2500(ast);
			ast_moutdwm(ast, 0x1E6E000C, 0x00005C01);
			if (cbr_test_2500(ast)) {
				pass++;
				if (min_ddr_vref > ddr_vref)
					min_ddr_vref = ddr_vref;
				if (max_ddr_vref < ddr_vref)
					max_ddr_vref = ddr_vref;
			} else if (pass != 0) {
				break;
			}
		}
	}

	ast_moutdwm(ast, 0x1E6E000C, 0x00000000);
	ast_moutdwm(ast, 0x1E6E0060, 0x00000000);
	ddr_vref = (min_ddr_vref + max_ddr_vref + 1) >> 1;
	ast_moutdwm(ast, 0x1E6E02C0, 0x00000006 | (ddr_vref << 8));

	/* Wait DDR PHY init done */
	ddr_phy_init_2500(ast);

	ast_moutdwm(ast, 0x1E6E0120, ddr_table[REGIDX_PLL]);
	ast_moutdwm(ast, 0x1E6E000C, 0x42AA5C81);
	ast_moutdwm(ast, 0x1E6E0034, 0x0001AF93);

	check_dram_size_2500(ast, ddr_table[REGIDX_RFC]);
	enable_cache_2500(ast);
	ast_moutdwm(ast, 0x1E6E001C, 0x00000008);
	ast_moutdwm(ast, 0x1E6E0038, 0xFFFFFF00);
}

static bool ast_dram_init_2500(struct ast_private *ast)
{
	u32 data;
	u32 max_tries = 5;

	do {
		if (max_tries-- == 0)
			return false;
		set_mpll_2500(ast);
		reset_mmc_2500(ast);
		ddr_init_common_2500(ast);

		data = ast_mindwm(ast, 0x1E6E2070);
		if (data & 0x01000000)
			ddr4_init_2500(ast, ast2500_ddr4_1600_timing_table);
		else
			ddr3_init_2500(ast, ast2500_ddr3_1600_timing_table);
	} while (!ddr_test_2500(ast));

	ast_moutdwm(ast, 0x1E6E2040, ast_mindwm(ast, 0x1E6E2040) | 0x41);

	/* Patch code */
	data = ast_mindwm(ast, 0x1E6E200C) & 0xF9FFFFFF;
	ast_moutdwm(ast, 0x1E6E200C, data | 0x10000000);
	/* Version Number */
	data = ast_mindwm(ast, 0x1E6E0004);			/* add at V1.3 */
	ast_moutdwm(ast, 0x1E6E0004, data | 0x08300000);	/* add at V1.3 */
	ast_moutdwm(ast, 0x1E6E0088, 0x20161229);		/* add at V1.3 */

	return true;
}

void ast_patch_ahb_2500(struct ast_private *ast)
{
	u32	data;

	/* Clear bus lock condition */
	ast_moutdwm(ast, 0x1e600000, 0xAEED1A03);
	ast_moutdwm(ast, 0x1e600084, 0x00010000);
	ast_moutdwm(ast, 0x1e600088, 0x00000000);
	ast_moutdwm(ast, 0x1e6e2000, 0x1688A8A8);
	data = ast_mindwm(ast, 0x1e6e2070);
	if (data & 0x08000000) {					/* check fast reset */
		/*
		 * If "Fast restet" is enabled for ARM-ICE debugger,
		 * then WDT needs to enable, that
		 * WDT04 is WDT#1 Reload reg.
		 * WDT08 is WDT#1 counter restart reg to avoid system deadlock
		 * WDT0C is WDT#1 control reg
		 *	[6:5]:= 01:Full chip
		 *	[4]:= 1:1MHz clock source
		 *	[1]:= 1:WDT will be cleeared and disabled after timeout occurs
		 *	[0]:= 1:WDT enable
		 */
		ast_moutdwm(ast, 0x1E785004, 0x00000010);
		ast_moutdwm(ast, 0x1E785008, 0x00004755);
		ast_moutdwm(ast, 0x1E78500c, 0x00000033);
		usleep_range(1000, 1200);
	}
	do {
		ast_moutdwm(ast, 0x1e6e2000, 0x1688A8A8);
		data = ast_mindwm(ast, 0x1e6e2000);
	} while (data != 1);
	ast_moutdwm(ast, 0x1e6e207c, 0x08000000);	/* clear fast reset */
}

void ast_post_chip_2500(struct drm_device *dev)
{
	struct ast_private *ast = to_ast_private(dev);
	u32 temp;
	u8 reg;

	reg = ast_get_index_reg_mask(ast, AST_IO_CRTC_PORT, 0xd0, 0xff);
	if ((reg & AST_VRAM_INIT_STATUS_MASK) == 0) {/* vga only */
		/* Clear bus lock condition */
		ast_patch_ahb_2500(ast);

		/* Disable watchdog */
		ast_moutdwm(ast, 0x1E78502C, 0x00000000);
		ast_moutdwm(ast, 0x1E78504C, 0x00000000);

		/*
		 * Reset USB port to patch USB unknown device issue
		 * SCU90 is Multi-function Pin Control #5
		 *	[29]:= 1:Enable USB2.0 Host port#1 (that the mutually shared USB2.0 Hub
		 *				port).
		 * SCU94 is Multi-function Pin Control #6
		 *	[14:13]:= 1x:USB2.0 Host2 controller
		 * SCU70 is Hardware Strap reg
		 *	[23]:= 1:CLKIN is 25MHz and USBCK1 = 24/48 MHz (determined by
		 *				[18]: 0(24)/1(48) MHz)
		 * SCU7C is Write clear reg to SCU70
		 *	[23]:= write 1 and then SCU70[23] will be clear as 0b.
		 */
		ast_moutdwm(ast, 0x1E6E2090, 0x20000000);
		ast_moutdwm(ast, 0x1E6E2094, 0x00004000);
		if (ast_mindwm(ast, 0x1E6E2070) & 0x00800000) {
			ast_moutdwm(ast, 0x1E6E207C, 0x00800000);
			mdelay(100);
			ast_moutdwm(ast, 0x1E6E2070, 0x00800000);
		}
		/* Modify eSPI reset pin */
		temp = ast_mindwm(ast, 0x1E6E2070);
		if (temp & 0x02000000)
			ast_moutdwm(ast, 0x1E6E207C, 0x00004000);

		/* Slow down CPU/AHB CLK in VGA only mode */
		temp = ast_read32(ast, 0x12008);
		temp |= 0x73;
		ast_write32(ast, 0x12008, temp);

		if (!ast_dram_init_2500(ast))
			drm_err(dev, "DRAM init failed !\n");

		temp = ast_mindwm(ast, 0x1e6e2040);
		ast_moutdwm(ast, 0x1e6e2040, temp | 0x40);
	}

	/* wait ready */
	do {
		reg = ast_get_index_reg_mask(ast, AST_IO_CRTC_PORT, 0xd0, 0xff);
	} while ((reg & 0x40) == 0);
}
