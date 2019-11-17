// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2009
 * Vipin Kumar, ST Micoelectronics, vipin.kumar@st.com.
 */

#include <clk.h>
#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <pci.h>
#include <reset.h>
#include <asm/io.h>
#include "designware_i2c.h"

#ifdef CONFIG_SYS_I2C_DW_ENABLE_STATUS_UNSUPPORTED
static int  dw_i2c_enable(struct i2c_regs *i2c_base, bool enable)
{
	u32 ena = enable ? IC_ENABLE_0B : 0;

	writel(ena, &i2c_base->ic_enable);

	return 0;
}
#else
static int dw_i2c_enable(struct i2c_regs *i2c_base, bool enable)
{
	u32 ena = enable ? IC_ENABLE_0B : 0;
	int timeout = 100;

	do {
		writel(ena, &i2c_base->ic_enable);
		if ((readl(&i2c_base->ic_enable_status) & IC_ENABLE_0B) == ena)
			return 0;

		/*
		 * Wait 10 times the signaling period of the highest I2C
		 * transfer supported by the driver (for 400KHz this is
		 * 25us) as described in the DesignWare I2C databook.
		 */
		udelay(25);
	} while (timeout--);
	printf("timeout in %sabling I2C adapter\n", enable ? "en" : "dis");

	return -ETIMEDOUT;
}
#endif

/* Frequency represented as ticks per ns. Can also be used to calculate
 * the number of ticks to meet a time target or the period. */
struct freq {
	uint32_t ticks;
	uint32_t ns;
};

enum i2c_speed {
	I2C_SPEED_STANDARD	= 100000,
	I2C_SPEED_FAST		= 400000,
	I2C_SPEED_FAST_PLUS	= 1000000,
	I2C_SPEED_HIGH		= 3400000,
	I2C_SPEED_FAST_ULTRA	= 5000000,
};

/* High and low times in different speed modes (in ns) */
enum {
	/* SDA Hold Time */
	DEFAULT_SDA_HOLD_TIME		= 300,
	/* Fast Plus Speed */
	MIN_FP_SCL_HIGHTIME		= 260,
	MIN_FP_SCL_LOWTIME		= 500,
};

static const struct i2c_descriptor {
	int speed;
	struct freq freq;
	int min_thigh_ns;
	int min_tlow_ns;
} speed_descriptors[] = {
	{
		.speed = I2C_SPEED_STANDARD,
		.freq = {
			.ticks = 100,
			.ns = 1000*1000,
		},
		.min_thigh_ns = MIN_SS_SCL_HIGHTIME,
		.min_tlow_ns = MIN_SS_SCL_LOWTIME,
	},
	{
		.speed = I2C_SPEED_FAST,
		.freq = {
			.ticks = 400,
			.ns = 1000*1000,
		},
		.min_thigh_ns = MIN_FS_SCL_HIGHTIME,
		.min_tlow_ns = MIN_FS_SCL_LOWTIME,
	},
	{
		.speed = I2C_SPEED_FAST_PLUS,
		.freq = {
			.ticks = 1,
			.ns = 1000,
		},
		.min_thigh_ns = MIN_FP_SCL_HIGHTIME,
		.min_tlow_ns = MIN_FP_SCL_LOWTIME,
	},
	{
		/* 100pF max capacitance */
		.speed = I2C_SPEED_HIGH,
		.freq = {
			.ticks = 3400,
			.ns = 1000*1000,
		},
		.min_thigh_ns = MIN_HS_SCL_HIGHTIME,
		.min_tlow_ns = MIN_HS_SCL_LOWTIME,
	},
};

static const struct soc_clock {
	int clk_speed_mhz;
	struct freq freq;
} soc_clocks[] = {
	{
		.clk_speed_mhz = 120,
		.freq = {
			.ticks = 120,
			.ns = 1000,
		},
	},
	{
		.clk_speed_mhz = 133,
		.freq = {
			.ticks = 400,
			.ns = 3000,
		},
	},
	{
		.clk_speed_mhz = 216,
		.freq = {
			.ticks = 1080,
			.ns = 5000,
		},
	},
};

static const struct i2c_descriptor *get_bus_descriptor(enum i2c_speed speed)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(speed_descriptors); i++)
		if (speed == speed_descriptors[i].speed)
			return &speed_descriptors[i];

	return NULL;
}

static const struct soc_clock *get_soc_descriptor(int ic_clk)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(soc_clocks); i++)
		if (ic_clk == soc_clocks[i].clk_speed_mhz)
			return &soc_clocks[i];

	return NULL;
}

struct dw_i2c_bus_config {
	/* Bus should be enabled prior to ramstage with temporary base */
	int early_init;
	/* Bus speed in Hz, default is I2C_SPEED_FAST (400 KHz) */
	int speed;
	/* If rise_time_ns is non-zero the calculations for lcnt and hcnt
	 * registers take into account the times of the bus. However, if
	 * there is a match in speed_config those register values take
	 * precedence. */
	int rise_time_ns;
	int fall_time_ns;
	int data_hold_time_ns;
};

struct dw_i2c_speed_config {
	int speed;
	/* SCL high and low period count */
	uint16_t scl_lcnt;
	uint16_t scl_hcnt;
	/*
	 * SDA hold time should be 300ns in standard and fast modes
	 * and long enough for deterministic logic level change in
	 * fast-plus and high speed modes.
	 *
	 *  [15:0] SDA TX Hold Time
	 * [23:16] SDA RX Hold Time
	 */
	uint32_t sda_hold;
};

static int counts_from_time(const struct freq *f, int ns)
{
	return DIV_ROUND_UP(f->ticks * ns, f->ns);
}

static int counts_from_freq(const struct freq *fast, const struct freq *slow)
{
	return DIV_ROUND_UP(fast->ticks * slow->ns, fast->ns * slow->ticks);
}

static int dw_i2c_gen_config_rise_fall_time(struct i2c_regs *regs,
					int speed,
					const struct dw_i2c_bus_config *bcfg,
					int ic_clk,
					struct dw_i2c_speed_config *config)
{
	const struct i2c_descriptor *bus;
	const struct soc_clock *soc;
	int fall_cnt, rise_cnt, min_tlow_cnt, min_thigh_cnt, spk_cnt;
	int hcnt, lcnt, period_cnt, diff, tot;
	int data_hold_time_ns;

	bus = get_bus_descriptor(speed);
	soc = get_soc_descriptor(ic_clk);

	/* Get the proper spike suppression count based on target speed. */
	if (speed >= I2C_MAX_SPEED)
		spk_cnt = readl(&regs->hs_spklen);
	else
		spk_cnt = readl(&regs->fs_spklen);

	/* Find the period, rise, fall, min tlow, and min thigh in terms of
	 * counts of SoC clock. */
	period_cnt = counts_from_freq(&soc->freq, &bus->freq);
	rise_cnt = counts_from_time(&soc->freq, bcfg->rise_time_ns);
	fall_cnt = counts_from_time(&soc->freq, bcfg->fall_time_ns);
	min_tlow_cnt = counts_from_time(&soc->freq, bus->min_tlow_ns);
	min_thigh_cnt = counts_from_time(&soc->freq, bus->min_thigh_ns);

	printf("dw_i2c: SoC %d/%d ns Bus: %d/%d ns\n",
		soc->freq.ticks, soc->freq.ns, bus->freq.ticks, bus->freq.ns);
	printf(
"		dw_i2c: period %d rise %d fall %d tlow %d thigh %d spk %d\n",
		period_cnt, rise_cnt, fall_cnt, min_tlow_cnt, min_thigh_cnt,
		spk_cnt);

	/*
	 * Back solve for hcnt and lcnt according to the following equations.
	 * SCL_High_time = [(HCNT + IC_*_SPKLEN + 7) * ic_clk] + SCL_Fall_time
	 * SCL_Low_time = [(LCNT + 1) * ic_clk] - SCL_Fall_time + SCL_Rise_time
	 */
	hcnt = min_thigh_cnt - fall_cnt - 7 - spk_cnt;
	lcnt = min_tlow_cnt - rise_cnt + fall_cnt - 1;

	if (hcnt < 0 || lcnt < 0) {
		printf("dw_i2c: bad counts. hcnt = %d lcnt = %d\n", hcnt, lcnt);
		return -1;
	}

	/* Now add things back up to ensure the period is hit. If off,
	 * split the difference and bias to lcnt for remainder. */
	tot = hcnt + lcnt + 7 + spk_cnt + rise_cnt + 1;

	if (tot < period_cnt) {
		diff = (period_cnt - tot) / 2;
		hcnt += diff;
		lcnt += diff;
		tot = hcnt + lcnt + 7 + spk_cnt + rise_cnt + 1;
		lcnt += period_cnt - tot;
	}

	config->speed = speed;
	config->scl_lcnt = lcnt;
	config->scl_hcnt = hcnt;

	/* Use internal default unless other value is specified. */
	data_hold_time_ns = DEFAULT_SDA_HOLD_TIME;
	if (bcfg->data_hold_time_ns)
		data_hold_time_ns = bcfg->data_hold_time_ns;

	config->sda_hold = counts_from_time(&soc->freq, data_hold_time_ns);

	printk("dw_i2c: hcnt = %d lcnt = %d sda hold = %d\n",
	       hcnt, lcnt, config->sda_hold);

	return 0;
}

/*
 * i2c_set_bus_speed - Set the i2c speed
 * @speed:	required i2c speed
 *
 * Set the i2c speed.
 */
static unsigned int __dw_i2c_set_bus_speed(struct i2c_regs *i2c_base,
					   struct dw_scl_sda_cfg *scl_sda_cfg,
					   unsigned int speed,
					   unsigned int bus_mhz)
{
	struct dw_i2c_bus_config cfg, *bcfg = &cfg;
	unsigned int cntl;
	unsigned int hcnt, lcnt;
	unsigned int ena;
	int i2c_spd;

	printf("bus_mhz = %d\n", bus_mhz);
	/* Allow max speed if there is no config , or the config allows it */
	if (speed >= I2C_MAX_SPEED &&
	    (!scl_sda_cfg || scl_sda_cfg->has_max_speed))
		i2c_spd = IC_SPEED_MODE_MAX;
	else if (speed >= I2C_FAST_SPEED)
		i2c_spd = IC_SPEED_MODE_FAST;
	else
		i2c_spd = IC_SPEED_MODE_STANDARD;

	/* Get enable setting for restore later */
	ena = readl(&i2c_base->ic_enable) & IC_ENABLE_0B;

	/* to set speed cltr must be disabled */
	dw_i2c_enable(i2c_base, false);

	cntl = (readl(&i2c_base->ic_con) & (~IC_CON_SPD_MSK));

	bcfg->data_hold_time_ns = 0;
	bcfg->rise_time_ns = 57;
	bcfg->fall_time_ns = 28;
	if (1 && bcfg->rise_time_ns) {
		struct dw_i2c_speed_config config;
		int ret;

		ret = dw_i2c_gen_config_rise_fall_time(i2c_base, speed, bcfg,
						       bus_mhz, &config);
		if (ret)
			return log_msg_ret("gen_confg", ret);
		writel(config.scl_hcnt, &i2c_base->ic_fs_scl_hcnt);
		writel(config.scl_lcnt, &i2c_base->ic_fs_scl_lcnt);
		cntl |= IC_CON_SPD_FS;
		writel(cntl, &i2c_base->ic_con);
		writel(config.sda_hold, &i2c_base->ic_sda_hold);
		printf("now %d, %d, %d\n", config.scl_hcnt, config.scl_lcnt,
		       config.sda_hold);
		goto done;
	}

	switch (i2c_spd) {
	case IC_SPEED_MODE_MAX:
		cntl |= IC_CON_SPD_SS;
		if (scl_sda_cfg) {
			hcnt = scl_sda_cfg->fs_hcnt;
			lcnt = scl_sda_cfg->fs_lcnt;
		} else {
			hcnt = (bus_mhz * MIN_HS_SCL_HIGHTIME) / NANO_TO_MICRO;
			lcnt = (bus_mhz * MIN_HS_SCL_LOWTIME) / NANO_TO_MICRO;
		}
		writel(hcnt, &i2c_base->ic_hs_scl_hcnt);
		writel(lcnt, &i2c_base->ic_hs_scl_lcnt);
		break;

	case IC_SPEED_MODE_STANDARD:
		cntl |= IC_CON_SPD_SS;
		if (scl_sda_cfg) {
			hcnt = scl_sda_cfg->ss_hcnt;
			lcnt = scl_sda_cfg->ss_lcnt;
		} else {
			hcnt = (bus_mhz * MIN_SS_SCL_HIGHTIME) / NANO_TO_MICRO;
			lcnt = (bus_mhz * MIN_SS_SCL_LOWTIME) / NANO_TO_MICRO;
		}
		writel(hcnt, &i2c_base->ic_ss_scl_hcnt);
		writel(lcnt, &i2c_base->ic_ss_scl_lcnt);
		break;

	case IC_SPEED_MODE_FAST:
	default:
		cntl |= IC_CON_SPD_FS;
		if (scl_sda_cfg) {
			hcnt = scl_sda_cfg->fs_hcnt;
			lcnt = scl_sda_cfg->fs_lcnt;
		} else {
			hcnt = (bus_mhz * MIN_FS_SCL_HIGHTIME) / NANO_TO_MICRO;
			lcnt = (bus_mhz * MIN_FS_SCL_LOWTIME) / NANO_TO_MICRO;
		}
		printf("was %d, %d\n", hcnt, lcnt);
#if 0
		printf("was %d, %d\n", hcnt, lcnt);
		hcnt = (bus_mhz * MIN_FS_SCL_HIGHTIME) / NANO_TO_MICRO;
		lcnt = (bus_mhz * MIN_FS_SCL_LOWTIME) / NANO_TO_MICRO;

// 		.rise_time_ns = 57,
// 			.fall_time_ns = 28,
#endif
		writel(hcnt, &i2c_base->ic_fs_scl_hcnt);
		writel(lcnt, &i2c_base->ic_fs_scl_lcnt);
		break;
	}
	writel(cntl, &i2c_base->ic_con);

	/* Configure SDA Hold Time if required */
	if (scl_sda_cfg)
		writel(scl_sda_cfg->sda_hold, &i2c_base->ic_sda_hold);

done:
	/* Restore back i2c now speed set */
	if (ena == IC_ENABLE_0B)
		dw_i2c_enable(i2c_base, true);

	return 0;
}

/*
 * i2c_setaddress - Sets the target slave address
 * @i2c_addr:	target i2c address
 *
 * Sets the target slave address.
 */
static void i2c_setaddress(struct i2c_regs *i2c_base, unsigned int i2c_addr)
{
	/* Disable i2c */
	dw_i2c_enable(i2c_base, false);

	writel(i2c_addr, &i2c_base->ic_tar);

	/* Enable i2c */
	dw_i2c_enable(i2c_base, true);
}

/*
 * i2c_flush_rxfifo - Flushes the i2c RX FIFO
 *
 * Flushes the i2c RX FIFO
 */
static void i2c_flush_rxfifo(struct i2c_regs *i2c_base)
{
	while (readl(&i2c_base->ic_status) & IC_STATUS_RFNE)
		readl(&i2c_base->ic_cmd_data);
}

/*
 * i2c_wait_for_bb - Waits for bus busy
 *
 * Waits for bus busy
 */
static int i2c_wait_for_bb(struct i2c_regs *i2c_base)
{
	unsigned long start_time_bb = get_timer(0);

	while ((readl(&i2c_base->ic_status) & IC_STATUS_MA) ||
	       !(readl(&i2c_base->ic_status) & IC_STATUS_TFE)) {

		/* Evaluate timeout */
		if (get_timer(start_time_bb) > (unsigned long)(I2C_BYTE_TO_BB))
			return 1;
	}

	return 0;
}

static int i2c_xfer_init(struct i2c_regs *i2c_base, uchar chip, uint addr,
			 int alen)
{
	if (i2c_wait_for_bb(i2c_base))
		return 1;

	i2c_setaddress(i2c_base, chip);
	while (alen) {
		alen--;
		/* high byte address going out first */
		writel((addr >> (alen * 8)) & 0xff,
		       &i2c_base->ic_cmd_data);
	}
	return 0;
}

static int i2c_xfer_finish(struct i2c_regs *i2c_base)
{
	ulong start_stop_det = get_timer(0);

	while (1) {
		if ((readl(&i2c_base->ic_raw_intr_stat) & IC_STOP_DET)) {
			readl(&i2c_base->ic_clr_stop_det);
			break;
		} else if (get_timer(start_stop_det) > I2C_STOPDET_TO) {
			break;
		}
	}

	if (i2c_wait_for_bb(i2c_base)) {
		printf("Timed out waiting for bus\n");
		return 1;
	}

	i2c_flush_rxfifo(i2c_base);

	return 0;
}

/*
 * i2c_read - Read from i2c memory
 * @chip:	target i2c address
 * @addr:	address to read from
 * @alen:
 * @buffer:	buffer for read data
 * @len:	no of bytes to be read
 *
 * Read from i2c memory.
 */
static int __dw_i2c_read(struct i2c_regs *i2c_base, u8 dev, uint addr,
			 int alen, u8 *buffer, int len)
{
	unsigned long start_time_rx;
	unsigned int active = 0;

#ifdef CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW
	/*
	 * EEPROM chips that implement "address overflow" are ones
	 * like Catalyst 24WC04/08/16 which has 9/10/11 bits of
	 * address and the extra bits end up in the "chip address"
	 * bit slots. This makes a 24WC08 (1Kbyte) chip look like
	 * four 256 byte chips.
	 *
	 * Note that we consider the length of the address field to
	 * still be one byte because the extra address bits are
	 * hidden in the chip address.
	 */
	dev |= ((addr >> (alen * 8)) & CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW);
	addr &= ~(CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW << (alen * 8));

	debug("%s: fix addr_overflow: dev %02x addr %02x\n", __func__, dev,
	      addr);
#endif

	if (i2c_xfer_init(i2c_base, dev, addr, alen))
		return 1;

	start_time_rx = get_timer(0);
	while (len) {
		if (!active) {
			/*
			 * Avoid writing to ic_cmd_data multiple times
			 * in case this loop spins too quickly and the
			 * ic_status RFNE bit isn't set after the first
			 * write. Subsequent writes to ic_cmd_data can
			 * trigger spurious i2c transfer.
			 */
			if (len == 1)
				writel(IC_CMD | IC_STOP, &i2c_base->ic_cmd_data);
			else
				writel(IC_CMD, &i2c_base->ic_cmd_data);
			active = 1;
		}

		if (readl(&i2c_base->ic_status) & IC_STATUS_RFNE) {
			*buffer++ = (uchar)readl(&i2c_base->ic_cmd_data);
			len--;
			start_time_rx = get_timer(0);
			active = 0;
		} else if (get_timer(start_time_rx) > I2C_BYTE_TO) {
			return 1;
		}
	}

	return i2c_xfer_finish(i2c_base);
}

/*
 * i2c_write - Write to i2c memory
 * @chip:	target i2c address
 * @addr:	address to read from
 * @alen:
 * @buffer:	buffer for read data
 * @len:	no of bytes to be read
 *
 * Write to i2c memory.
 */
static int __dw_i2c_write(struct i2c_regs *i2c_base, u8 dev, uint addr,
			  int alen, u8 *buffer, int len)
{
	int nb = len;
	unsigned long start_time_tx;

#ifdef CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW
	/*
	 * EEPROM chips that implement "address overflow" are ones
	 * like Catalyst 24WC04/08/16 which has 9/10/11 bits of
	 * address and the extra bits end up in the "chip address"
	 * bit slots. This makes a 24WC08 (1Kbyte) chip look like
	 * four 256 byte chips.
	 *
	 * Note that we consider the length of the address field to
	 * still be one byte because the extra address bits are
	 * hidden in the chip address.
	 */
	dev |= ((addr >> (alen * 8)) & CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW);
	addr &= ~(CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW << (alen * 8));

	debug("%s: fix addr_overflow: dev %02x addr %02x\n", __func__, dev,
	      addr);
#endif

	if (i2c_xfer_init(i2c_base, dev, addr, alen))
		return 1;

	start_time_tx = get_timer(0);
	while (len) {
		if (readl(&i2c_base->ic_status) & IC_STATUS_TFNF) {
			if (--len == 0) {
				writel(*buffer | IC_STOP,
				       &i2c_base->ic_cmd_data);
			} else {
				writel(*buffer, &i2c_base->ic_cmd_data);
			}
			buffer++;
			start_time_tx = get_timer(0);

		} else if (get_timer(start_time_tx) > (nb * I2C_BYTE_TO)) {
				printf("Timed out. i2c write Failed\n");
				return 1;
		}
	}

	return i2c_xfer_finish(i2c_base);
}

/*
 * __dw_i2c_init - Init function
 * @speed:	required i2c speed
 * @slaveaddr:	slave address for the device
 *
 * Initialization function.
 */
static int __dw_i2c_init(struct i2c_regs *i2c_base, int speed, int slaveaddr)
{
	int ret;

	/* Disable i2c */
	ret = dw_i2c_enable(i2c_base, false);
	if (ret)
		return ret;

	writel(IC_CON_SD | IC_CON_RE | IC_CON_SPD_FS | IC_CON_MM,
	       &i2c_base->ic_con);
	writel(IC_RX_TL, &i2c_base->ic_rx_tl);
	writel(IC_TX_TL, &i2c_base->ic_tx_tl);
	writel(IC_STOP_DET, &i2c_base->ic_intr_mask);
#ifndef CONFIG_DM_I2C
	__dw_i2c_set_bus_speed(i2c_base, NULL, speed, IC_CLK);
	writel(slaveaddr, &i2c_base->ic_sar);
#endif

	/* Enable i2c */
	ret = dw_i2c_enable(i2c_base, true);
	if (ret)
		return ret;

	return 0;
}

#ifndef CONFIG_DM_I2C
/*
 * The legacy I2C functions. These need to get removed once
 * all users of this driver are converted to DM.
 */
static struct i2c_regs *i2c_get_base(struct i2c_adapter *adap)
{
	switch (adap->hwadapnr) {
#if CONFIG_SYS_I2C_BUS_MAX >= 4
	case 3:
		return (struct i2c_regs *)CONFIG_SYS_I2C_BASE3;
#endif
#if CONFIG_SYS_I2C_BUS_MAX >= 3
	case 2:
		return (struct i2c_regs *)CONFIG_SYS_I2C_BASE2;
#endif
#if CONFIG_SYS_I2C_BUS_MAX >= 2
	case 1:
		return (struct i2c_regs *)CONFIG_SYS_I2C_BASE1;
#endif
	case 0:
		return (struct i2c_regs *)CONFIG_SYS_I2C_BASE;
	default:
		printf("Wrong I2C-adapter number %d\n", adap->hwadapnr);
	}

	return NULL;
}

static unsigned int dw_i2c_set_bus_speed(struct i2c_adapter *adap,
					 unsigned int speed)
{
	adap->speed = speed;
	return __dw_i2c_set_bus_speed(i2c_get_base(adap), NULL, speed, IC_CLK);
}

static void dw_i2c_init(struct i2c_adapter *adap, int speed, int slaveaddr)
{
	__dw_i2c_init(i2c_get_base(adap), speed, slaveaddr);
}

static int dw_i2c_read(struct i2c_adapter *adap, u8 dev, uint addr,
		       int alen, u8 *buffer, int len)
{
	return __dw_i2c_read(i2c_get_base(adap), dev, addr, alen, buffer, len);
}

static int dw_i2c_write(struct i2c_adapter *adap, u8 dev, uint addr,
			int alen, u8 *buffer, int len)
{
	return __dw_i2c_write(i2c_get_base(adap), dev, addr, alen, buffer, len);
}

/* dw_i2c_probe - Probe the i2c chip */
static int dw_i2c_probe(struct i2c_adapter *adap, u8 dev)
{
	struct i2c_regs *i2c_base = i2c_get_base(adap);
	u32 tmp;
	int ret;

	/*
	 * Try to read the first location of the chip.
	 */
	ret = __dw_i2c_read(i2c_base, dev, 0, 1, (uchar *)&tmp, 1);
	if (ret)
		dw_i2c_init(adap, adap->speed, adap->slaveaddr);

	return ret;
}

U_BOOT_I2C_ADAP_COMPLETE(dw_0, dw_i2c_init, dw_i2c_probe, dw_i2c_read,
			 dw_i2c_write, dw_i2c_set_bus_speed,
			 CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE, 0)

#if CONFIG_SYS_I2C_BUS_MAX >= 2
U_BOOT_I2C_ADAP_COMPLETE(dw_1, dw_i2c_init, dw_i2c_probe, dw_i2c_read,
			 dw_i2c_write, dw_i2c_set_bus_speed,
			 CONFIG_SYS_I2C_SPEED1, CONFIG_SYS_I2C_SLAVE1, 1)
#endif

#if CONFIG_SYS_I2C_BUS_MAX >= 3
U_BOOT_I2C_ADAP_COMPLETE(dw_2, dw_i2c_init, dw_i2c_probe, dw_i2c_read,
			 dw_i2c_write, dw_i2c_set_bus_speed,
			 CONFIG_SYS_I2C_SPEED2, CONFIG_SYS_I2C_SLAVE2, 2)
#endif

#if CONFIG_SYS_I2C_BUS_MAX >= 4
U_BOOT_I2C_ADAP_COMPLETE(dw_3, dw_i2c_init, dw_i2c_probe, dw_i2c_read,
			 dw_i2c_write, dw_i2c_set_bus_speed,
			 CONFIG_SYS_I2C_SPEED3, CONFIG_SYS_I2C_SLAVE3, 3)
#endif

#else /* CONFIG_DM_I2C */
/* The DM I2C functions */

static int designware_i2c_xfer(struct udevice *bus, struct i2c_msg *msg,
			       int nmsgs)
{
	struct dw_i2c *i2c = dev_get_priv(bus);
	int ret;

	debug("i2c_xfer: %d messages\n", nmsgs);
	for (; nmsgs > 0; nmsgs--, msg++) {
		debug("i2c_xfer: chip=0x%x, len=0x%x\n", msg->addr, msg->len);
		if (msg->flags & I2C_M_RD) {
			ret = __dw_i2c_read(i2c->regs, msg->addr, 0, 0,
					    msg->buf, msg->len);
		} else {
			ret = __dw_i2c_write(i2c->regs, msg->addr, 0, 0,
					     msg->buf, msg->len);
		}
		if (ret) {
			debug("i2c_write: error sending\n");
			return -EREMOTEIO;
		}
	}

	return 0;
}

static int designware_i2c_set_bus_speed(struct udevice *bus, unsigned int speed)
{
	struct dw_i2c *i2c = dev_get_priv(bus);
	ulong rate;

#if CONFIG_IS_ENABLED(CLK)
	rate = clk_get_rate(&i2c->clk);
	if (IS_ERR_VALUE(rate))
		return -EINVAL;

	/* Convert to MHz */
	rate /= 1000000;
#else
	rate = IC_CLK;
#endif
	return __dw_i2c_set_bus_speed(i2c->regs, i2c->scl_sda_cfg, speed,
				      rate);
}

static int designware_i2c_probe_chip(struct udevice *bus, uint chip_addr,
				     uint chip_flags)
{
	struct dw_i2c *i2c = dev_get_priv(bus);
	struct i2c_regs *i2c_base = i2c->regs;
	u32 tmp;
	int ret;

	/* Try to read the first location of the chip */
	ret = __dw_i2c_read(i2c_base, chip_addr, 0, 1, (uchar *)&tmp, 1);
	if (ret)
		__dw_i2c_init(i2c_base, 0, 0);

	return ret;
}

static int designware_i2c_ofdata_to_platdata(struct udevice *bus)
{
	struct dw_i2c *priv = dev_get_priv(bus);

	priv->regs = (struct i2c_regs *)devfdt_get_addr_ptr(bus);

	return 0;
}

int designware_i2c_probe(struct udevice *bus)
{
	struct dw_i2c *priv = dev_get_priv(bus);
	int ret;

	ret = reset_get_bulk(bus, &priv->resets);
	if (ret)
		dev_warn(bus, "Can't get reset: %d\n", ret);
	else
		reset_deassert_bulk(&priv->resets);

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_get_by_index(bus, 0, &priv->clk);
	if (ret)
		return ret;

	ret = clk_enable(&priv->clk);
	if (ret && ret != -ENOSYS && ret != -ENOTSUPP) {
		clk_free(&priv->clk);
		dev_err(bus, "failed to enable clock\n");
		return ret;
	}
#endif

	return __dw_i2c_init(priv->regs, 0, 0);
}

int designware_i2c_remove(struct udevice *dev)
{
	struct dw_i2c *priv = dev_get_priv(dev);

#if CONFIG_IS_ENABLED(CLK)
	clk_disable(&priv->clk);
	clk_free(&priv->clk);
#endif

	return reset_release_bulk(&priv->resets);
}

const struct dm_i2c_ops designware_i2c_ops = {
	.xfer		= designware_i2c_xfer,
	.probe_chip	= designware_i2c_probe_chip,
	.set_bus_speed	= designware_i2c_set_bus_speed,
};

static const struct udevice_id designware_i2c_ids[] = {
	{ .compatible = "snps,designware-i2c" },
	{ }
};

U_BOOT_DRIVER(i2c_designware) = {
	.name	= "i2c_designware",
	.id	= UCLASS_I2C,
	.of_match = designware_i2c_ids,
	.ofdata_to_platdata = designware_i2c_ofdata_to_platdata,
	.probe	= designware_i2c_probe,
	.priv_auto_alloc_size = sizeof(struct dw_i2c),
	.remove = designware_i2c_remove,
	.flags	= DM_FLAG_OS_PREPARE,
	.ops	= &designware_i2c_ops,
};

#endif /* CONFIG_DM_I2C */
