// SPDX-License-Identifier: GPL-2.0
/*
 * Cr50 / H1 TPM support
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY UCLASS_TPM
#define LOG_DEBUG

#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <tpm-v2.h>
#include <asm/gpio.h>

enum {
	TIMEOUT_LONG_US		= 2 * 1000 * 1000,
	TIMEOUT_SHORT_US	= 2 * 1000,
	TIMEOUT_NO_IRQ_US	= 20 * 1000,
	TIMEOUT_IRQ_US		= 100 * 1000,
};

enum {
	CR50_DID_VID = 0x00281ae0L
};

enum {
	CR50_MAX_BUF_SIZE = 63,
};

struct cr50_priv {
	struct gpio_desc ready_gpio;
	int locality;
	uint vendor;
};

/* Wait for interrupt to indicate TPM is ready */
static int cr50_i2c_wait_tpm_ready(struct udevice *dev)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	ulong timeout;
	int i;

	if (!dm_gpio_is_valid(&priv->ready_gpio)) {
		/* Fixed delay if interrupt not supported */
		udelay(TIMEOUT_NO_IRQ_US);
		return 0;
	}

	timeout = timer_get_us() + TIMEOUT_IRQ_US;

	i = 0;
	while (!dm_gpio_get_value(&priv->ready_gpio)) {
		i++;
		if (timer_get_us() > timeout) {
			printf("Timeout\n");
			/*
			 * Use this instead of -ETIMEDOUT which is used by i2c
			 */
			return -ETIME;
		}
	}

	return 0;
}

/* Clear pending interrupts */
static void cr50_i2c_clear_tpm_irq(struct udevice *dev)
{
	/* This is not really an interrupt, just a GPIO, so we can't clear it */
}

/*
 * cr50_i2c_read() - read from TPM register
 *
 * @tpm: TPM chip information
 * @addr: register address to read from
 * @buffer: provided by caller
 * @len: number of bytes to read
 *
 * 1) send register address byte 'addr' to the TPM
 * 2) wait for TPM to indicate it is ready
 * 3) read 'len' bytes of TPM response into the provided 'buffer'
 *
 * Return 0 on success. -ve on error
 */
static int cr50_i2c_read(struct udevice *dev, u8 addr, u8 *buffer,
			 size_t len)
{
	int ret;

	/* Clear interrupt before starting transaction */
	cr50_i2c_clear_tpm_irq(dev);

	/* Send the register address byte to the TPM */
	ret = dm_i2c_write(dev, 0, &addr, 1);
	if (ret) {
		log_err("Address write failed (err=%d)\n", ret);
		return ret;
	}

	/* Wait for TPM to be ready with response data */
	ret = cr50_i2c_wait_tpm_ready(dev);
	if (ret)
		return ret;

	/* Read response data frrom the TPM */
	ret = dm_i2c_read(dev, 0, buffer, len);
	if (ret) {
		log_err("Read response failed (err=%d)\n", ret);
		return ret;
	}

	return 0;
}

/*
 * cr50_i2c_write() - write to TPM register
 *
 * @tpm: TPM chip information
 * @addr: register address to write to
 * @buffer: data to write
 * @len: number of bytes to write
 *
 * 1) prepend the provided address to the provided data
 * 2) send the address+data to the TPM
 * 3) wait for TPM to indicate it is done writing
 *
 * Returns -1 on error, 0 on success.
 */
static int cr50_i2c_write(struct udevice *dev, u8 addr, const u8 *buffer,
			  size_t len)
{
	u8 buf[len + 1];
	int ret;

	if (len > CR50_MAX_BUF_SIZE) {
		log_err("Length %zd is too large\n", len);
		return -E2BIG;
	}

	/* Prepend the 'register address' to the buffer */
	buf[0] = addr;
	memcpy(buf + 1, buffer, len);

	/* Clear interrupt before starting transaction */
	cr50_i2c_clear_tpm_irq(dev);

	/* Send write request buffer with address */
	ret = dm_i2c_write(dev, 0, buf, len + 1);
	if (ret) {
		log_err("Error writing to TPM (err=%d)\n", ret);
		return ret;
	}

	/* Wait for TPM to be ready */
	return cr50_i2c_wait_tpm_ready(dev);
}

#define TPM_HEADER_SIZE 10

static inline u8 tpm_access(u8 locality)
{
	return 0x0 | (locality << 4);
}

static inline u8 tpm_sts(u8 locality)
{
	return 0x1 | (locality << 4);
}

static inline u8 tpm_data_fifo(u8 locality)
{
	return 0x5 | (locality << 4);
}

static inline u8 tpm_did_vid(u8 locality)
{
	return 0x6 | (locality << 4);
}

static int check_locality(struct udevice *dev, int loc)
{
	u8 mask = TPM_ACCESS_VALID | TPM_ACCESS_ACTIVE_LOCALITY;
	int ret;
	u8 buf;

	ret = cr50_i2c_read(dev, tpm_access(loc), &buf, 1);
	if (ret)
		return ret;

	if ((buf & mask) == mask)
		return loc;

	return -EPERM;
}

static int release_locality(struct udevice *dev, int force)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	u8 mask = TPM_ACCESS_VALID | TPM_ACCESS_REQUEST_PENDING;
	u8 addr = tpm_access(priv->locality);
	int ret;
	u8 buf;

	ret = cr50_i2c_read(dev, addr, &buf, 1);
	if (ret)
		return ret;

	if (force || (buf & mask) == mask) {
		buf = TPM_ACCESS_ACTIVE_LOCALITY;
		cr50_i2c_write(dev, addr, &buf, 1);
	}

	priv->locality = 0;

	return 0;
}

static int request_locality(struct udevice *dev, int loc)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	u8 buf = TPM_ACCESS_REQUEST_USE;
	ulong timeout;
	int ret;

	ret = check_locality(dev, loc);
	if (ret < 0)
		return ret;
	else if (ret == loc)
		return loc;

	ret = cr50_i2c_write(dev, tpm_access(loc), &buf, 1);
	if (ret)
		return ret;

	timeout = timer_get_us() + TIMEOUT_LONG_US;
	while (timer_get_us() < timeout) {
		ret = check_locality(dev, loc);
		if (ret < 0)
			return ret;
		if (ret == loc) {
			priv->locality = loc;
			log_debug("Set locality to %x\n", loc);
			return loc;
		}
		udelay(TIMEOUT_SHORT_US);
	}
	printf("Request locality failed\n");

	return -ETIMEDOUT;
}

/* cr50 requires all 4 bytes of status register to be read */
static int cr50_i2c_status(struct udevice *dev)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	u8 buf[4];
	int ret;

	ret = cr50_i2c_read(dev, tpm_sts(priv->locality), buf, sizeof(buf));
	if (ret)
		return ret;

	return buf[0];
}

/* cr50 requires all 4 bytes of status register to be written */
static int cr50_i2c_ready(struct udevice *dev)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	u8 buf[4] = { TPM_STS_COMMAND_READY };
	int ret;

	ret = cr50_i2c_write(dev, tpm_sts(priv->locality), buf, sizeof(buf));
	if (ret)
		return ret;

	udelay(TIMEOUT_SHORT_US);

	return 0;
}

static int cr50_i2c_wait_burststs(struct udevice *dev, u8 mask,
				  size_t *burst, int *status)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	ulong timeout;
	u32 buf;

	timeout = timer_get_us() + TIMEOUT_LONG_US;
	while (timer_get_us() < timeout) {
		if (cr50_i2c_read(dev, tpm_sts(priv->locality),
				  (u8 *)&buf, sizeof(buf)) < 0) {
			udelay(TIMEOUT_SHORT_US);
			continue;
		}

		*status = buf & 0xff;
		*burst = le16_to_cpu((buf >> 8) & 0xffff);

		if ((*status & mask) == mask &&
		    *burst > 0 && *burst <= CR50_MAX_BUF_SIZE)
			return 0;

		udelay(TIMEOUT_SHORT_US);
	}

	printf("Timeout reading burst and status\n");

	return -ETIMEDOUT;
}

static int cr50_i2c_recv(struct udevice *dev, u8 *buf, size_t buf_len)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	size_t burstcnt, expected, current, len;
	u8 addr = tpm_data_fifo(priv->locality);
	u8 mask = TPM_STS_VALID | TPM_STS_DATA_AVAIL;
	u32 expected_buf;
	int status;
	int ret;

	if (buf_len < TPM_HEADER_SIZE)
		return -E2BIG;

	if (cr50_i2c_wait_burststs(dev, mask, &burstcnt, &status) < 0) {
		printf("First chunk not available\n");
		goto out_err;
	}

	/* Read first chunk of burstcnt bytes */
	if (cr50_i2c_read(dev, addr, buf, burstcnt) < 0) {
		printf("Read failed\n");
		goto out_err;
	}

	/* Determine expected data in the return buffer */
	memcpy(&expected_buf, buf + TPM_CMD_COUNT_OFFSET, sizeof(expected_buf));
	expected = be32_to_cpu(expected_buf);
	if (expected > buf_len) {
		printf("Too much data: %zu > %zu\n",
		       expected, buf_len);
		goto out_err;
	}

	/* Now read the rest of the data */
	current = burstcnt;
	while (current < expected) {
		/* Read updated burst count and check status */
		if (cr50_i2c_wait_burststs(dev, mask, &burstcnt, &status) < 0)
			goto out_err;

		len = min(burstcnt, expected - current);
		if (cr50_i2c_read(dev, addr, buf + current, len) != 0) {
			printf("Read failed\n");
			goto out_err;
		}

		current += len;
	}

	if (cr50_i2c_wait_burststs(dev, TPM_STS_VALID, &burstcnt, &status) < 0)
		goto out_err;
	if (status & TPM_STS_DATA_AVAIL) {
		printf("Data still available\n");
		goto out_err;
	}

	return current;

out_err:
	/* Abort current transaction if still pending */
	ret = cr50_i2c_status(dev);
	if (ret)
		return ret;
	if (ret & TPM_STS_COMMAND_READY) {
		ret = cr50_i2c_ready(dev);
		if (ret)
			return ret;
	}

	return -EIO;
}

static int cr50_i2c_send(struct udevice *dev, const u8 *buf, size_t len)
{
	struct cr50_priv *priv = dev_get_priv(dev);

	int status;
	size_t burstcnt, limit, sent = 0;
	u8 tpm_go[4] = { TPM_STS_GO };
	ulong timeout;
	int ret;

	timeout = timer_get_us() + TIMEOUT_LONG_US;
	do {
		ret = cr50_i2c_status(dev);
		if (ret)
			goto out_err;
		if (!(ret & TPM_STS_COMMAND_READY))
			break;

		if (timer_get_us() > timeout)
			goto out_err;

		ret = cr50_i2c_ready(dev);
		if (ret)
			goto out_err;
	} while (1);

	while (len > 0) {
		u8 mask = TPM_STS_VALID;

		/* Wait for data if this is not the first chunk */
		if (sent > 0)
			mask |= TPM_STS_DATA_EXPECT;

		if (cr50_i2c_wait_burststs(dev, mask, &burstcnt, &status) < 0)
			goto out_err;

		/*
		 * Use burstcnt - 1 to account for the address byte
		 * that is inserted by cr50_i2c_write()
		 */
		limit = min(burstcnt - 1, len);
		if (cr50_i2c_write(dev, tpm_data_fifo(priv->locality),
				   &buf[sent], limit) != 0) {
			printf("Write failed\n");
			goto out_err;
		}

		sent += limit;
		len -= limit;
	}

	/* Ensure TPM is not expecting more data */
	if (cr50_i2c_wait_burststs(dev, TPM_STS_VALID, &burstcnt, &status) < 0)
		goto out_err;
	if (status & TPM_STS_DATA_EXPECT) {
		printf("Data still expected\n");
		goto out_err;
	}

	/* Start the TPM command */
	ret = cr50_i2c_write(dev, tpm_sts(priv->locality), tpm_go,
			     sizeof(tpm_go));
	if (ret) {
		printf("Start command failed\n");
		goto out_err;
	}
	return sent;

out_err:
	/* Abort current transaction if still pending */
	ret = cr50_i2c_status(dev);
	if (ret)
		return ret;

	if (ret & TPM_STS_COMMAND_READY) {
		ret = cr50_i2c_ready(dev);
		if (ret)
			return ret;
	}

	return -EIO;
}

static int cr50_i2c_get_desc(struct udevice *dev, char *buf, int size)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	struct cr50_priv *priv = dev_get_priv(dev);

	return snprintf(buf, size, "cr50 TPM 2.0 (i2c %02x id %x)",
			chip->chip_addr, priv->vendor >> 16);
}

static int cr50_i2c_open(struct udevice *dev)
{
	struct cr50_priv *priv = dev_get_priv(dev);
	char buf[80];
	u32 vendor;
	int ret;

	ret = request_locality(dev, 0);
	if (ret)
		return ret;

	/* Read four bytes from DID_VID register */
	ret = cr50_i2c_read(dev, tpm_did_vid(0), (u8 *)&vendor, 4);
	if (ret) {
		release_locality(dev, 1);
		return ret;
	}

	if (vendor != CR50_DID_VID) {
		printf("Vendor ID 0x%08x not recognized.\n", vendor);
		release_locality(dev, 1);
		return -EXDEV;
	}

	priv->vendor = vendor;
	cr50_i2c_get_desc(dev, buf, sizeof(buf));
	log_debug("%s\n", buf);

	return 0;
}

static int cr50_i2c_cleanup(struct udevice *dev)
{
	release_locality(dev, 1);

	return 0;
}

enum {
	TPM_TIMEOUT_MS		= 5,
	SHORT_TIMEOUT_MS	= 750,
	LONG_TIMEOUT_MS		= 2000,
};

static int cr50_i2c_probe(struct udevice *dev)
{
	struct tpm_chip_priv *upriv = dev_get_uclass_priv(dev);
	struct cr50_priv *priv = dev_get_priv(dev);
	int ret;

	upriv->version = TPM_V2;
	upriv->duration_ms[TPM_SHORT] = SHORT_TIMEOUT_MS;
	upriv->duration_ms[TPM_MEDIUM] = LONG_TIMEOUT_MS;
	upriv->duration_ms[TPM_LONG] = LONG_TIMEOUT_MS;
	upriv->retry_time_ms = TPM_TIMEOUT_MS;

	upriv->pcr_count = 32;
	upriv->pcr_select_min = 2;

	/* Optional GPIO to track when cr50 is ready */
	ret = gpio_request_by_name(dev, "ready-gpio", 0, &priv->ready_gpio,
				   GPIOD_IS_IN);
	if (ret)
		debug("Warning: Cr50 does not have a ready-gpio (err=%d)\n",
		      ret);

	return 0;
}

static const struct tpm_ops cr50_i2c_ops = {
	.open		= cr50_i2c_open,
	.get_desc	= cr50_i2c_get_desc,
	.send		= cr50_i2c_send,
	.recv		= cr50_i2c_recv,
	.cleanup	= cr50_i2c_cleanup,
};

static const struct udevice_id cr50_i2c_ids[] = {
	{ .compatible = "google,cr50" },
	{ }
};

U_BOOT_DRIVER(cr50_i2c) = {
	.name   = "cr50_i2c",
	.id     = UCLASS_TPM,
	.of_match = cr50_i2c_ids,
	.ops    = &cr50_i2c_ops,
	.probe	= cr50_i2c_probe,
	.priv_auto_alloc_size = sizeof(struct cr50_priv),
};
