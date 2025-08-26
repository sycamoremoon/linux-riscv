// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023-2025 SpacemiT (Hangzhou) Technology Co. Ltd
 * Copyright (c) 2025 Yixun Lan <dlan@gentoo.org>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/init.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "sdhci.h"
#include "sdhci-pltfm.h"

#define SPACEMIT_SDHC_OP_EXT_REG	0x108
#define  SDHC_OVRRD_CLK_OEN		BIT(11)
#define  SDHC_FORCE_CLK_ON		BIT(12)

#define SPACEMIT_SDHC_MMC_CTRL_REG	0x114
#define  SDHC_MISC_INT_EN		BIT(1)
#define  SDHC_MISC_INT			BIT(2)
#define  SDHC_ENHANCE_STROBE_EN		BIT(8)
#define  SDHC_MMC_HS400			BIT(9)
#define  SDHC_MMC_HS200			BIT(10)
#define  SDHC_MMC_CARD_MODE		BIT(12)

#define SDHC_TX_CFG_REG                 0x11C
#define TX_INT_CLK_SEL                  0x40000000
#define TX_MUX_SEL                      0x80000000

#define SPACEMIT_SDHC_TX_CFG_REG	0x11C
#define  SDHC_TX_INT_CLK_SEL		BIT(30)
#define  SDHC_TX_MUX_SEL		BIT(31)

#define SPACEMIT_SDHC_PHY_CTRL_REG	0x160
#define  SDHC_PHY_FUNC_EN		BIT(0)
#define  SDHC_PHY_PLL_LOCK		BIT(1)
#define  SDHC_HOST_LEGACY_MODE		BIT(31)

#define SPACEMIT_SDHC_PHY_FUNC_REG	0x164
#define  SDHC_PHY_TEST_EN		BIT(7)
#define  SDHC_HS200_USE_RFIFO		BIT(15)

#define SPACEMIT_SDHC_PHY_DLLCFG	0x168
#define  SDHC_DLL_PREDLY_NUM		GENMASK(3, 2)
#define  SDHC_DLL_FULLDLY_RANGE		GENMASK(5, 4)
#define  SDHC_DLL_VREG_CTRL		GENMASK(7, 6)
#define  SDHC_DLL_ENABLE		BIT(31)

#define SPACEMIT_SDHC_PHY_DLLCFG1	0x16C
#define  SDHC_DLL_REG1_CTRL		GENMASK(7, 0)
#define  SDHC_DLL_REG2_CTRL		GENMASK(15, 8)
#define  SDHC_DLL_REG3_CTRL		GENMASK(23, 16)
#define  SDHC_DLL_REG4_CTRL		GENMASK(31, 24)

#define SPACEMIT_SDHC_PHY_DLLSTS	0x170
#define  SDHC_DLL_LOCK_STATE		BIT(0)

#define SPACEMIT_SDHC_PHY_PADCFG_REG	0x178
#define  SDHC_PHY_DRIVE_SEL		GENMASK(2, 0)
#define  SDHC_RX_BIAS_CTRL		BIT(5)

#define SPACEMIT_SDHC_RX_CFG_REG	0x118
#define  RX_SDCLK_SEL0			GENMASK(1, 0)
#define  RX_SDCLK_SEL1			GENMASK(3, 2)

#define SPACEMIT_SDHC_DLINE_CTRL_REG	0x130
#define  DLINE_PU			BIT(0)
#define  RX_DLINE_CODE			GENMASK(23, 16)
#define  TX_DLINE_CODE			GENMASK(31, 24)

#define SPACEMIT_SDHC_DLINE_CFG_REG	0x134
#define  RX_DLINE_REG			GENMASK(7, 0)
#define  RX_DLINE_GAIN			BIT(8)
#define  TX_DLINE_REG			GENMASK(23, 16)

#define SDHC_RX_TUNE_DELAY_MIN		0x0
#define SDHC_RX_TUNE_DELAY_MAX		0xFF
#define SDHC_RX_TUNE_DELAY_STEP		0x1

#define CANDIDATE_WIN_NUM		3
#define SELECT_DELAY_NUM		9

#define RX_TUNING_WINDOW_THRESHOLD	80
#define RX_TUNING_DLINE_REG		0x00
#define TX_TUNING_DLINE_REG		0x00
#define TX_TUNING_DELAYCODE		0x5F

enum window_type {
        LEFT_WINDOW = 0,
        MIDDLE_WINDOW = 1,
        RIGHT_WINDOW = 2,
};

struct tuning_window {
        u8 type;
        u8 min_delay;
        u8 max_delay;
};

struct rx_tuning {
	u8 tx_dline_reg;
	u8 tx_delaycode;
        u8 rx_dline_reg;
        u8 select_delay_num;
        u8 current_delay_index;
        /* 0: biggest window, 1: bigger, 2:  small */
        struct tuning_window windows[CANDIDATE_WIN_NUM];
        u8 select_delay[SELECT_DELAY_NUM];

        u32 card_cid[4];
        u8 window_limit;
        u8 tuning_fail;
        u8 window_type;
};

struct spacemit_sdhci_host {
	struct clk *clk_core;
	struct clk *clk_io;
	wait_queue_head_t wait_queue;
	atomic_t ref_count;
	struct rx_tuning rxtuning;
};

static const u32 tuning_patten4[16] = {
	0x00ff0fff, 0xccc3ccff, 0xffcc3cc3, 0xeffefffe,
	0xddffdfff, 0xfbfffbff, 0xff7fffbf, 0xefbdf777,
	0xf0fff0ff, 0x3cccfc0f, 0xcfcc33cc, 0xeeffefff,
	0xfdfffdff, 0xffbfffdf, 0xfff7ffbb, 0xde7b7ff7,
};

static const u32 tuning_patten8[32] = {
	0xff00ffff, 0x0000ffff, 0xccccffff, 0xcccc33cc,
	0xcc3333cc, 0xffffcccc, 0xffffeeff, 0xffeeeeff,
	0xffddffff, 0xddddffff, 0xbbffffff, 0xbbffffff,
	0xffffffbb, 0xffffff77, 0x77ff7777, 0xffeeddbb,
	0x00ffffff, 0x00ffffff, 0xccffff00, 0xcc33cccc,
	0x3333cccc, 0xffcccccc, 0xffeeffff, 0xeeeeffff,
	0xddffffff, 0xddffffff, 0xffffffdd, 0xffffffbb,
	0xffffbbbb, 0xffff77ff, 0xff7777ff, 0xeeddbb77,
};

/* All helper functions will update clr/set while preserve rest bits */
static inline void spacemit_sdhci_setbits(struct sdhci_host *host, u32 val, int reg)
{
	sdhci_writel(host, sdhci_readl(host, reg) | val, reg);
}

static inline void spacemit_sdhci_clrbits(struct sdhci_host *host, u32 val, int reg)
{
	sdhci_writel(host, sdhci_readl(host, reg) & ~val, reg);
}

static inline void spacemit_sdhci_clrsetbits(struct sdhci_host *host, u32 clr, u32 set, int reg)
{
	u32 val = sdhci_readl(host, reg);

	val = (val & ~clr) | set;
	sdhci_writel(host, val, reg);
}

static void spacemit_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	sdhci_reset(host, mask);

	if (mask != SDHCI_RESET_ALL)
		return;

	spacemit_sdhci_setbits(host, SDHC_PHY_FUNC_EN | SDHC_PHY_PLL_LOCK,
			       SPACEMIT_SDHC_PHY_CTRL_REG);

	spacemit_sdhci_clrsetbits(host, SDHC_PHY_DRIVE_SEL,
				  SDHC_RX_BIAS_CTRL | FIELD_PREP(SDHC_PHY_DRIVE_SEL, 4),
				  SPACEMIT_SDHC_PHY_PADCFG_REG);

	if (!(host->mmc->caps2 & MMC_CAP2_NO_MMC))
		spacemit_sdhci_setbits(host, SDHC_MMC_CARD_MODE, SPACEMIT_SDHC_MMC_CTRL_REG);
}

static void spacemit_sdhci_set_uhs_signaling(struct sdhci_host *host, unsigned int timing)
{
	if (timing == MMC_TIMING_MMC_HS200)
		spacemit_sdhci_setbits(host, SDHC_MMC_HS200, SPACEMIT_SDHC_MMC_CTRL_REG);

	if (timing == MMC_TIMING_MMC_HS400)
		spacemit_sdhci_setbits(host, SDHC_MMC_HS400, SPACEMIT_SDHC_MMC_CTRL_REG);

	sdhci_set_uhs_signaling(host, timing);
}

static void spacemit_sdhci_set_clk_gate(struct sdhci_host *host, unsigned int auto_gate)
{
	unsigned int reg;

	reg = sdhci_readl(host, SPACEMIT_SDHC_OP_EXT_REG);
	if (auto_gate)
		reg &= ~(SDHC_OVRRD_CLK_OEN | SDHC_FORCE_CLK_ON);
	else
		reg |= (SDHC_OVRRD_CLK_OEN | SDHC_FORCE_CLK_ON);
	sdhci_writel(host, reg, SPACEMIT_SDHC_OP_EXT_REG);
}

static void spacemit_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct mmc_host *mmc = host->mmc;

	if (mmc->ios.timing <= MMC_TIMING_UHS_SDR50)
		spacemit_sdhci_setbits(host, SDHC_TX_INT_CLK_SEL, SPACEMIT_SDHC_TX_CFG_REG);
	else
		spacemit_sdhci_clrbits(host, SDHC_TX_INT_CLK_SEL, SPACEMIT_SDHC_TX_CFG_REG);

	sdhci_set_clock(host, clock);

	if (host->mmc->caps2 & MMC_CAP2_NO_MMC) {
		if (SDHCI_GET_CMD(sdhci_readw(host, SDHCI_COMMAND)) == SD_SWITCH_VOLTAGE)
			spacemit_sdhci_set_clk_gate(host, 0); /* disable auto clock */
		else
			spacemit_sdhci_set_clk_gate(host, 1); /* enable auto clock */

	}
};

static u32 spacemit_handle_interrupt(struct sdhci_host *host, u32 intmask)
{
	if ((intmask & SDHCI_INT_CARD_INT) && (host->ier & SDHCI_INT_CARD_INT)) {
		if (!(host->flags & SDHCI_DEVICE_DEAD)) {
			host->ier &= ~SDHCI_INT_CARD_INT;
			sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
			sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);
		}

		/* wakeup ksdioirqd thread */
		host->mmc->sdio_irq_pending = true;
		if (host->mmc->sdio_irq_thread)
			wake_up_process(host->mmc->sdio_irq_thread);
	}

	return intmask;
}

static void spacemit_sdhci_voltage_switch(struct sdhci_host *host)
{
#define MMC1_IO_V18EN		BIT(2)
#define AKEY_ASFAR		0xBABA
#define AKEY_ASSAR		0xEB10
        void __iomem *aib_mmc1_io, *apbc_asfar, *apbc_assar;
	int vol = host->mmc->ios.signal_voltage;
        u32 reg;
	if( vol != MMC_SIGNAL_VOLTAGE_180)
		return;
	else {
		/*
		 * v18en(MS) bit should meet TSMC's requirement
		 * when switch SOC SD IO voltage from 3.3(3.0)v to 1.8v
		 */
		aib_mmc1_io = ioremap(0xD401E81C, 4);
		apbc_asfar = ioremap(0xD4015050, 4);
		apbc_assar = ioremap(0xD4015054, 4);

		writel(AKEY_ASFAR, apbc_asfar);
		writel(AKEY_ASSAR, apbc_assar);
		reg = readl(aib_mmc1_io);
		reg |= MMC1_IO_V18EN;
		writel(AKEY_ASFAR, apbc_asfar);
		writel(AKEY_ASSAR, apbc_assar);
		writel(reg, aib_mmc1_io);

		iounmap(apbc_assar);
		iounmap(apbc_asfar);
		iounmap(aib_mmc1_io);
	}
}

static void spacemit_sdhci_phy_dll_init(struct sdhci_host *host)
{
	u32 state;
	int ret;

	spacemit_sdhci_clrsetbits(host, SDHC_DLL_PREDLY_NUM |
				  SDHC_DLL_FULLDLY_RANGE |
				  SDHC_DLL_VREG_CTRL,
				  FIELD_PREP(SDHC_DLL_PREDLY_NUM, 1) |
				  FIELD_PREP(SDHC_DLL_FULLDLY_RANGE, 1) |
				  FIELD_PREP(SDHC_DLL_VREG_CTRL, 1),
				  SPACEMIT_SDHC_PHY_DLLCFG);

	spacemit_sdhci_clrsetbits(host, SDHC_DLL_REG1_CTRL,
				  FIELD_PREP(SDHC_DLL_REG1_CTRL, 0x92),
				  SPACEMIT_SDHC_PHY_DLLCFG1);

	spacemit_sdhci_setbits(host, SDHC_DLL_ENABLE, SPACEMIT_SDHC_PHY_DLLCFG);

	ret = readl_poll_timeout(host->ioaddr + SPACEMIT_SDHC_PHY_DLLSTS, state,
				 state & SDHC_DLL_LOCK_STATE, 2, 100);
	if (ret == -ETIMEDOUT)
		dev_warn(mmc_dev(host->mmc), "fail to lock phy dll in 100us!\n");
}

static void spacemit_sdhci_hs400_enhanced_strobe(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);

	if (!ios->enhanced_strobe) {
		spacemit_sdhci_clrbits(host, SDHC_ENHANCE_STROBE_EN, SPACEMIT_SDHC_MMC_CTRL_REG);
		return;
	}

	spacemit_sdhci_setbits(host, SDHC_ENHANCE_STROBE_EN, SPACEMIT_SDHC_MMC_CTRL_REG);
	spacemit_sdhci_phy_dll_init(host);
}

static unsigned int spacemit_sdhci_clk_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return clk_get_rate(pltfm_host->clk);
}

static int spacemit_sdhci_pre_select_hs400(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);

	spacemit_sdhci_setbits(host, SDHC_MMC_HS400, SPACEMIT_SDHC_MMC_CTRL_REG);

	return 0;
}

static void spacemit_sdhci_post_select_hs400(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);

	spacemit_sdhci_phy_dll_init(host);
}

static void spacemit_sdhci_pre_hs400_to_hs200(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);

	spacemit_sdhci_clrbits(host, SDHC_PHY_FUNC_EN | SDHC_PHY_PLL_LOCK,
			       SPACEMIT_SDHC_PHY_CTRL_REG);
	spacemit_sdhci_clrbits(host, SDHC_MMC_HS400 | SDHC_MMC_HS200 | SDHC_ENHANCE_STROBE_EN,
			       SPACEMIT_SDHC_MMC_CTRL_REG);
	spacemit_sdhci_clrbits(host, SDHC_HS200_USE_RFIFO, SPACEMIT_SDHC_PHY_FUNC_REG);

	udelay(5);

	spacemit_sdhci_setbits(host, SDHC_PHY_FUNC_EN | SDHC_PHY_PLL_LOCK,
			       SPACEMIT_SDHC_PHY_CTRL_REG);
}

static inline int spacemit_sdhci_get_clocks(struct device *dev,
					    struct sdhci_pltfm_host *pltfm_host)
{
	struct spacemit_sdhci_host *sdhst = sdhci_pltfm_priv(pltfm_host);

	sdhst->clk_core = devm_clk_get_enabled(dev, "core");
	if (IS_ERR(sdhst->clk_core))
		return -EINVAL;

	sdhst->clk_io = devm_clk_get_enabled(dev, "io");
	if (IS_ERR(sdhst->clk_io))
		return -EINVAL;

	pltfm_host->clk = sdhst->clk_io;

	return 0;
}

static void spacemit_sdhci_enable_sdio_irq_nolock(struct sdhci_host *host, int enable)
{
	if (!(host->flags & SDHCI_DEVICE_DEAD)) {
		if (enable)
			host->ier |= SDHCI_INT_CARD_INT;
		else
			host->ier &= ~SDHCI_INT_CARD_INT;

		sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
		sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);
	}
}

static void spacemit_sdhci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sdhci_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	spacemit_sdhci_enable_sdio_irq_nolock(host, enable);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void spacemit_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sdhci_host *host = mmc_priv(mmc);
	unsigned long flags;

	spacemit_sdhci_enable_sdio_irq(mmc, enable);

	/* avoid to read the SDIO_CCCR_INTx */
	spin_lock_irqsave(&host->lock, flags);
	mmc->sdio_irq_pending = true;
	spin_unlock_irqrestore(&host->lock, flags);
}

static void spacemit_sdhci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct spacemit_sdhci_host *sdhst;
	sdhst = sdhci_pltfm_priv(sdhci_priv(mmc_priv(mmc)));

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO)) {
	        while (atomic_inc_return(&sdhst->ref_count) > 1) {
	                atomic_dec(&sdhst->ref_count);
	                wait_event(sdhst->wait_queue, atomic_read(&sdhst->ref_count) == 0);
	        }
	}

	sdhci_request(mmc, mrq);
}

static void spacemit_sdhci_request_done(struct sdhci_host *host,
                                    struct mmc_request *mrq)
{
	struct spacemit_sdhci_host *sdhst;
	sdhst = sdhci_pltfm_priv(sdhci_priv(host));

	mmc_request_done(host->mmc, mrq);

	if (!(host->mmc->caps2 & MMC_CAP2_NO_SDIO)) {
	        atomic_dec(&sdhst->ref_count);
	        wake_up(&sdhst->wait_queue);
	}
}

static void spacemit_sw_rx_tuning_prepare(struct sdhci_host *host, u8 dline_reg)
{
	struct mmc_host *mmc = host->mmc;
	struct mmc_ios ios = mmc->ios;
	u32 reg;

	reg = sdhci_readl(host, SPACEMIT_SDHC_DLINE_CFG_REG);
	reg &= ~RX_DLINE_REG;
	reg |= FIELD_PREP(RX_DLINE_REG, dline_reg);
	reg &= ~RX_DLINE_GAIN;
	if ((ios.timing == MMC_TIMING_UHS_SDR50) && (reg & 0x40))
		reg |= FIELD_PREP(RX_DLINE_GAIN, 1);
	sdhci_writel(host, reg, SPACEMIT_SDHC_DLINE_CFG_REG);

	reg = sdhci_readl(host, SPACEMIT_SDHC_DLINE_CTRL_REG);
	reg |= DLINE_PU;
	sdhci_writel(host, reg, SPACEMIT_SDHC_DLINE_CTRL_REG);
	udelay(5);

	reg = sdhci_readl(host, SPACEMIT_SDHC_RX_CFG_REG);
	reg &= ~RX_SDCLK_SEL1;
	reg |= FIELD_PREP(RX_SDCLK_SEL1, 1);
	sdhci_writel(host, reg, SPACEMIT_SDHC_RX_CFG_REG);

	if ((mmc->ios.timing == MMC_TIMING_MMC_HS200)) {
		reg = sdhci_readl(host, SPACEMIT_SDHC_PHY_FUNC_REG);
		reg |= SDHC_HS200_USE_RFIFO;
		sdhci_writel(host, reg, SPACEMIT_SDHC_PHY_FUNC_REG);
	}
}

static void spacemit_sw_rx_set_delaycode(struct sdhci_host *host, u32 delay)
{
	u32 reg;

	reg = sdhci_readl(host, SPACEMIT_SDHC_DLINE_CTRL_REG);
	reg &= ~RX_DLINE_CODE;
	reg |= FIELD_PREP(RX_DLINE_CODE, delay);
	sdhci_writel(host, reg, SPACEMIT_SDHC_DLINE_CTRL_REG);
}

static void spacemit_sw_tx_tuning_prepare(struct sdhci_host *host)
{
	u32 reg;

	/* set TX_MUX_SEL */
	reg = sdhci_readl(host, SDHC_TX_CFG_REG);
	reg |= TX_MUX_SEL;
	sdhci_writel(host, reg, SDHC_TX_CFG_REG);

	reg = sdhci_readl(host, SPACEMIT_SDHC_DLINE_CTRL_REG);
	reg |= DLINE_PU;
	sdhci_writel(host, reg, SPACEMIT_SDHC_DLINE_CTRL_REG);
	udelay(5);
}

static void spacemit_sw_tx_set_dlinereg(struct sdhci_host *host, u8 dline_reg)
{
	u32 reg;

	reg = sdhci_readl(host, SPACEMIT_SDHC_DLINE_CFG_REG);
	reg &= ~TX_DLINE_REG;
	reg |= FIELD_PREP(TX_DLINE_REG, dline_reg);
	sdhci_writel(host, reg, SPACEMIT_SDHC_DLINE_CFG_REG);
}

static void spacemit_sw_tx_set_delaycode(struct sdhci_host *host, u32 delay)
{
	u32 reg;

	reg = sdhci_readl(host, SPACEMIT_SDHC_DLINE_CTRL_REG);
	reg &= ~TX_DLINE_CODE;
	reg |= FIELD_PREP(TX_DLINE_CODE, delay);
	sdhci_writel(host, reg, SPACEMIT_SDHC_DLINE_CTRL_REG);
}

static void spacemit_sdhci_clear_set_irqs(struct sdhci_host *host, u32 clr, u32 set)
{
	u32 ier;

	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	ier &= ~clr;
	ier |= set;
	sdhci_writel(host, ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, ier, SDHCI_SIGNAL_ENABLE);
}

static int spacemit_tuning_patten_check(struct sdhci_host *host, int point)
{
	u32 read_patten;
	unsigned int i;
	u32 *tuning_patten;
	int patten_len;
	int err = 0;

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_8) {
		tuning_patten = (u32 *)tuning_patten8;
		patten_len = ARRAY_SIZE(tuning_patten8);
	} else {
		tuning_patten = (u32 *)tuning_patten4;
		patten_len = ARRAY_SIZE(tuning_patten4);
	}

	for (i = 0; i < patten_len; i++) {
		read_patten = sdhci_readl(host, SDHCI_BUFFER);
		if (read_patten != tuning_patten[i])
			err++;
	}

	return err;
}

static int spacemit_send_tuning_cmd(struct sdhci_host *host, u32 opcode,
					int point, unsigned long flags)
{
	int err = 0;

	spin_unlock_irqrestore(&host->lock, flags);

	sdhci_send_tuning(host, opcode);

	spin_lock_irqsave(&host->lock, flags);
	if (!host->tuning_done) {
		pr_err("%s: Timeout waiting for Buffer Read Ready interrupt "
			"during tuning procedure, resetting CMD and DATA\n",
			mmc_hostname(host->mmc));
		sdhci_reset(host, SDHCI_RESET_CMD|SDHCI_RESET_DATA);
		/* err = -EIO; */
	} else
		err = spacemit_tuning_patten_check(host, point);

	host->tuning_done = 0;
	return err;
}

static int spacemit_sw_rx_select_window(struct sdhci_host *host, u32 opcode)
{
	int min;
	int max;
	u16 ctrl;
	u32 ier;
	unsigned long flags = 0;
	int err = 0;
	int i, j, len;
	struct tuning_window tmp;
	struct mmc_host *mmc = host->mmc;
	struct spacemit_sdhci_host *sdhst = sdhci_pltfm_priv(sdhci_priv(host));
	struct rx_tuning *rxtuning = &sdhst->rxtuning;

	/* change to pio mode during the tuning stage */
	spin_lock_irqsave(&host->lock, flags);
	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	spacemit_sdhci_clear_set_irqs(host, ier, SDHCI_INT_DATA_AVAIL);

	min = SDHC_RX_TUNE_DELAY_MIN;
	do {
		/* find the mininum delay first which can pass tuning */
		while (min < SDHC_RX_TUNE_DELAY_MAX) {
			spacemit_sw_rx_set_delaycode(host, min);
			if (!mmc->ops->get_cd(mmc)) {
				spin_unlock_irqrestore(&host->lock, flags);
				return -ENODEV;
			}
			err = spacemit_send_tuning_cmd(host, opcode, min, flags);
			if (err == -EIO) {
				spin_unlock_irqrestore(&host->lock, flags);
				return -EIO;
			}
			if (!err)
				break;
			ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
			ctrl &= ~(SDHCI_CTRL_TUNED_CLK | SDHCI_CTRL_EXEC_TUNING);
			sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
			min += SDHC_RX_TUNE_DELAY_STEP;
		}

		/* find the maxinum delay which can not pass tuning */
		max = min + SDHC_RX_TUNE_DELAY_STEP;
		while (max < SDHC_RX_TUNE_DELAY_MAX) {
			spacemit_sw_rx_set_delaycode(host, max);
			if (!mmc->ops->get_cd(mmc)) {
				spin_unlock_irqrestore(&host->lock, flags);
				return -ENODEV;
			}
			err = spacemit_send_tuning_cmd(host, opcode, max, flags);
			if (err) {
				ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
				ctrl &= ~(SDHCI_CTRL_TUNED_CLK | SDHCI_CTRL_EXEC_TUNING);
				sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
				if (err == -EIO) {
					spin_unlock_irqrestore(&host->lock, flags);
					return -EIO;
				}
				break;
			}
			max += SDHC_RX_TUNE_DELAY_STEP;
		}

		pr_notice("%s: pass window [%d %d) \n", mmc_hostname(host->mmc), min, max);
		/* store the top 3 window */
		if ((max - min) >= rxtuning->window_limit) {
			tmp.max_delay = max;
			tmp.min_delay = min;
			tmp.type = rxtuning->window_type;
			for (i = 0; i < CANDIDATE_WIN_NUM; i++) {
				len = rxtuning->windows[i].max_delay - rxtuning->windows[i].min_delay;
				if ((tmp.max_delay - tmp.min_delay) > len) {
					for (j = CANDIDATE_WIN_NUM - 1; j > i; j--) {
						rxtuning->windows[j] = rxtuning->windows[j-1];
					}
					rxtuning->windows[i] = tmp;
					break;
				}
			}
		}
		min = max + SDHC_RX_TUNE_DELAY_STEP;
	} while (min < SDHC_RX_TUNE_DELAY_MAX);

	spacemit_sdhci_clear_set_irqs(host, SDHCI_INT_DATA_AVAIL, ier);
	spin_unlock_irqrestore(&host->lock, flags);
	return 0;
}

static int spacemit_sw_rx_select_delay(struct sdhci_host *host)
{
	int i;
	int win_len, min, max, mid;
	struct tuning_window *window;

	struct spacemit_sdhci_host *sdhst = sdhci_pltfm_priv(sdhci_priv(host));
	struct rx_tuning *rxtuning = &sdhst->rxtuning;

	for (i = 0; i < CANDIDATE_WIN_NUM; i++) {
		window = &rxtuning->windows[i];
		min = window->min_delay;
		max = window->max_delay;
		mid = (min + max - 1) / 2;
		win_len = max - min;
		if (win_len < rxtuning->window_limit)
			continue;

		if (window->type == LEFT_WINDOW) {
			rxtuning->select_delay[rxtuning->select_delay_num++] = min + win_len / 4;
			rxtuning->select_delay[rxtuning->select_delay_num++] = min + win_len / 3;
		} else if (window->type == RIGHT_WINDOW) {
			rxtuning->select_delay[rxtuning->select_delay_num++] = max - win_len / 4;
			rxtuning->select_delay[rxtuning->select_delay_num++] = max - win_len / 3;
		} else {
			rxtuning->select_delay[rxtuning->select_delay_num++] = mid;
			rxtuning->select_delay[rxtuning->select_delay_num++] = mid + win_len / 4;
			rxtuning->select_delay[rxtuning->select_delay_num++] = mid - win_len / 4;
		}
	}

	return rxtuning->select_delay_num;
}

static void spacemit_sw_rx_card_store(struct sdhci_host *host, struct rx_tuning *rxtuning)
{
	struct mmc_card *card = host->mmc->card;

	if (card)
		memcpy(rxtuning->card_cid, card->raw_cid, sizeof(card->raw_cid));
}

static int spacemit_sw_rx_card_pretuned(struct sdhci_host *host, struct rx_tuning *rxtuning)
{
	struct mmc_card *card = host->mmc->card;

	if (!card)
		return 0;

	return !memcmp(rxtuning->card_cid, card->raw_cid, sizeof(card->raw_cid));
}

static int spacemit_sdhci_execute_sw_tuning(struct sdhci_host *host, u32 opcode)
{
	int ret;
	int index;
	struct mmc_host *mmc = host->mmc;
	struct mmc_ios ios = mmc->ios;
	struct spacemit_sdhci_host *sdhst = sdhci_pltfm_priv(sdhci_priv(host));
	struct rx_tuning *rxtuning = &sdhst->rxtuning;

	/*
	 * Tuning is required for SDR50/SDR104, HS200/HS400 cards and
	 * if clock frequency is greater than 100MHz in these modes.
	 */
	if (host->clock < 100 * 1000 * 1000 ||
	    !((ios.timing == MMC_TIMING_MMC_HS200) ||
	      (ios.timing == MMC_TIMING_UHS_SDR50) ||
	      (ios.timing == MMC_TIMING_UHS_SDR104)))
		return 0;

	if (!(mmc->caps2 & MMC_CAP2_NO_SD) && !mmc->ops->get_cd(mmc)) {
		return 0;
	}

	/* TX tuning config */
	if (host->mmc->caps2 & MMC_CAP2_NO_MMC) {
		spacemit_sw_tx_set_dlinereg(host, sdhst->rxtuning.tx_dline_reg);
		spacemit_sw_tx_set_delaycode(host, sdhst->rxtuning.tx_delaycode);
		pr_info("%s: set tx_delaycode: %d\n", mmc_hostname(mmc), sdhst->rxtuning.tx_delaycode);
		spacemit_sw_tx_tuning_prepare(host);
	}

	/* step 1: check pretuned card */
	if (spacemit_sw_rx_card_pretuned(host, rxtuning) &&
	    rxtuning->select_delay_num) {
		index = rxtuning->current_delay_index;
		if (mmc->doing_retune)
			index++;
		if (index == rxtuning->select_delay_num) {
			pr_info("%s: all select delay failed, re-init to DDR50\n", mmc_hostname(mmc));
			rxtuning->select_delay_num = 0;
			rxtuning->current_delay_index = 0;
			memset(rxtuning->windows, 0, sizeof(rxtuning->windows));
			memset(rxtuning->select_delay, 0xFF, sizeof(rxtuning->select_delay));
			memset(rxtuning->card_cid, 0, sizeof(rxtuning->card_cid));
			rxtuning->tuning_fail = 1;
			return -EIO;
		}

		spacemit_sw_rx_tuning_prepare(host, rxtuning->rx_dline_reg);
		spacemit_sw_rx_set_delaycode(host, rxtuning->select_delay[index]);
		pr_info("%s: pretuned card, use select_delay[%d]:%d\n",
			mmc_hostname(mmc), index, rxtuning->select_delay[index]);
		rxtuning->current_delay_index = index;
		return 0;
	}

	/* specify cpu freq during tuning rx windows if current cpufreq exceed 1.6G */
	rxtuning->select_delay_num = 0;
	rxtuning->current_delay_index = 0;
	memset(rxtuning->windows, 0, sizeof(rxtuning->windows));
	memset(rxtuning->select_delay, 0xFF, sizeof(rxtuning->select_delay));
	memset(rxtuning->card_cid, 0, sizeof(rxtuning->card_cid));

	/* step 2: get pass window and caculate the select_delay */
	spacemit_sw_rx_tuning_prepare(host, rxtuning->rx_dline_reg);
	ret = spacemit_sw_rx_select_window(host, opcode);

	if (ret) {
		pr_warn("%s: abort tuning, err:%d\n", mmc_hostname(mmc), ret);
		rxtuning->tuning_fail = 1;
		return ret;
	}

	if (!spacemit_sw_rx_select_delay(host)) {
		pr_warn("%s: fail to get delaycode\n", mmc_hostname(mmc));
		rxtuning->tuning_fail = 1;
		ret = -EIO;
		return ret;
	}

	/* step 3: set the delay code and store card cid */
	spacemit_sw_rx_set_delaycode(host, rxtuning->select_delay[0]);
	spacemit_sw_rx_card_store(host, rxtuning);
	rxtuning->tuning_fail = 0;
	pr_info("%s: tuning done, use the firstly delay_code:%d\n",
		mmc_hostname(mmc), rxtuning->select_delay[0]);

	return ret;
}
static const struct sdhci_ops spacemit_sdhci_ops = {
	.get_max_clock		= spacemit_sdhci_clk_get_max_clock,
	.reset			= spacemit_sdhci_reset,
	.set_bus_width		= sdhci_set_bus_width,
	.set_clock		= spacemit_sdhci_set_clock,
	.set_uhs_signaling	= spacemit_sdhci_set_uhs_signaling,
	.voltage_switch		= spacemit_sdhci_voltage_switch,
	.irq			= spacemit_handle_interrupt,
	.set_power		= sdhci_set_power_and_bus_voltage,
	.request_done		= spacemit_sdhci_request_done,
	.platform_execute_tuning= spacemit_sdhci_execute_sw_tuning,
};

static const struct sdhci_pltfm_data spacemit_sdhci_k1_pdata = {
	.ops = &spacemit_sdhci_ops,
	.quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC |
		  SDHCI_QUIRK_32BIT_ADMA_SIZE |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_BROKEN_TIMEOUT_VAL,
	.quirks2 = SDHCI_QUIRK2_BROKEN_64_BIT_DMA |
		   SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static const struct of_device_id spacemit_sdhci_of_match[] = {
	{ .compatible = "spacemit,k1-sdhci" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spacemit_sdhci_of_match);

static int spacemit_sdhci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spacemit_sdhci_host *sdhst;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct mmc_host_ops *mops;
	int ret;

	host = sdhci_pltfm_init(pdev, &spacemit_sdhci_k1_pdata, sizeof(*sdhst));

	if (IS_ERR(host))
		return PTR_ERR(host);

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_pltfm;
	sdhci_get_of_property(pdev);

	pltfm_host = sdhci_priv(host);

	sdhst = sdhci_pltfm_priv(pltfm_host);
	sdhst->rxtuning.rx_dline_reg = RX_TUNING_DLINE_REG;
	sdhst->rxtuning.window_limit = RX_TUNING_WINDOW_THRESHOLD;
	sdhst->rxtuning.window_type  = MIDDLE_WINDOW;
	sdhst->rxtuning.tx_dline_reg = TX_TUNING_DLINE_REG;
	sdhst->rxtuning.tx_delaycode = TX_TUNING_DELAYCODE;
	init_waitqueue_head(&sdhst->wait_queue);
	atomic_set(&sdhst->ref_count, 0);

	mops = &host->mmc_host_ops;
	if (!(host->mmc->caps2 & MMC_CAP2_NO_MMC)) {
		mops->hs400_prepare_ddr	= spacemit_sdhci_pre_select_hs400;
		mops->hs400_complete	= spacemit_sdhci_post_select_hs400;
		mops->hs400_downgrade	= spacemit_sdhci_pre_hs400_to_hs200;
		mops->hs400_enhanced_strobe = spacemit_sdhci_hs400_enhanced_strobe;
	}
	mops->enable_sdio_irq = spacemit_enable_sdio_irq;
	mops->request = spacemit_sdhci_request;

	host->mmc->caps |= MMC_CAP_NEED_RSP_BUSY;

	ret = spacemit_sdhci_get_clocks(dev, pltfm_host);
	if (ret)
		goto err_pltfm;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_pltfm;

	return 0;

err_pltfm:
	return ret;
}

static struct platform_driver spacemit_sdhci_driver = {
	.driver		= {
		.name	= "sdhci-spacemit",
		.of_match_table = spacemit_sdhci_of_match,
	},
	.probe		= spacemit_sdhci_probe,
	.remove		= sdhci_pltfm_remove,
};
module_platform_driver(spacemit_sdhci_driver);

MODULE_DESCRIPTION("SpacemiT SDHCI platform driver");
MODULE_LICENSE("GPL");
