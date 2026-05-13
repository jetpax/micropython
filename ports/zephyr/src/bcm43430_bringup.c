/*
 * BCM43430A1 SDIO bring-up shim.
 *
 * Runs at APPLICATION priority (after the BCM2835 SDHCI driver's
 * POST_KERNEL init) and exercises Zephyr's SDIO subsystem against
 * the on-module CYW43439 wireless chip on Pi Zero 2 W:
 *
 *   sd_init()        -- subsystem re-runs CMD0/5/3/7/52 enumeration
 *   sdio_read_byte() -- read CCCR rev via func0
 *   sdio_init_func() -- pull CIS tuples for backplane function (#1)
 *   sdio_enable_func() -- flip CCCR_IO_EN bit 1
 *
 * Throwaway: lives here until a proper brcmfmac-style WLAN driver
 * is built under drivers/wifi/. Drop once that lands.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>
#include <zephyr/sd/sd_spec.h>

LOG_MODULE_REGISTER(bcm43430_bringup, LOG_LEVEL_INF);

#if !DT_HAS_CHOSEN(zephyr_sdhc) && !DT_NODE_EXISTS(DT_ALIAS(sdhc0))
#error "bcm43430_bringup needs an sdhc0 alias in the board DT"
#endif

/* BCM brcmfmac function-1 misc-register layout (from Linux's
 * drivers/net/wireless/broadcom/brcm80211/brcmfmac/sdio.h). Function 1
 * exposes a 32 KiB sliding window into the chip's AXI/SB backplane;
 * the SBADDR* registers set the window base, then accesses to
 * func1[0..0x7FFF] map to backplane[base + offset]. Setting bit 15 in
 * the offset (SB_ACCESS_2_4B_FLAG) makes a 4-byte access at the chip,
 * not four single-byte accesses.
 */
#define SBSDIO_FUNC1_SBADDRLOW		0x1000A
#define SBSDIO_FUNC1_SBADDRMID		0x1000B
#define SBSDIO_FUNC1_SBADDRHIGH		0x1000C
#define SBSDIO_FUNC1_CHIPCLKCSR		0x1000E
#define SBSDIO_FUNC1_SDIOPULLUP		0x1000F
#define SBSDIO_SB_OFT_ADDR_MASK		0x07FFF
#define SBSDIO_SB_ACCESS_2_4B_FLAG	0x08000
#define SBSDIO_SBWINDOW_MASK		0xFFFF8000

/* CHIPCLKCSR bits (brcmfmac sdio.c). */
#define SBSDIO_FORCE_ALP		0x01
#define SBSDIO_ALP_AVAIL_REQ		0x08
#define SBSDIO_FORCE_HW_CLKREQ_OFF	0x20
#define SBSDIO_ALP_AVAIL		0x40
#define SBSDIO_HT_AVAIL			0x80
#define SBSDIO_AVBITS			(SBSDIO_ALP_AVAIL | SBSDIO_HT_AVAIL)
#define BRCMF_INIT_CLKCTL1		(SBSDIO_FORCE_HW_CLKREQ_OFF | \
					 SBSDIO_ALP_AVAIL_REQ)

/* Chipcommon core base on BCM43xx SDIO chips (SI_ENUM_BASE_DEFAULT). */
#define BRCMF_SI_ENUM_BASE		0x18000000
/* Chipcommon core[0] = chipid register layout. */
#define CID_ID_MASK			0x0000FFFF
#define CID_REV_MASK			0x000F0000
#define CID_REV_SHIFT			16
#define CID_TYPE_MASK			0xF0000000
#define CID_TYPE_SHIFT			28

static const struct device *const sdhc_dev = DEVICE_DT_GET(DT_ALIAS(sdhc0));
static struct sd_card card;
static struct sdio_func backplane;

/* Set the function-1 backplane window. Per brcmfmac's
 * brcmf_sdiod_set_backplane_window: write the 3 high bytes of the
 * target address to SBADDRLOW/MID/HIGH (each is a byte register).
 */
static int set_backplane_window(uint32_t addr)
{
	uint32_t bar0 = addr & SBSDIO_SBWINDOW_MASK;
	uint32_t v = bar0 >> 8;
	int ret;

	for (int i = 0; i < 3; i++, v >>= 8) {
		ret = sdio_write_byte(&backplane,
				      SBSDIO_FUNC1_SBADDRLOW + i,
				      (uint8_t)(v & 0xFF));
		if (ret != 0) {
			LOG_ERR("SBADDR[%d] write failed: %d", i, ret);
			return ret;
		}
	}
	return 0;
}

/* 32-bit read at a backplane address (mirrors brcmf_sdiod_readl). */
static int backplane_read32(uint32_t addr, uint32_t *out)
{
	uint8_t buf[4];
	int ret;

	ret = set_backplane_window(addr);
	if (ret != 0) {
		return ret;
	}

	uint32_t off = (addr & SBSDIO_SB_OFT_ADDR_MASK) | SBSDIO_SB_ACCESS_2_4B_FLAG;

	ret = sdio_read_addr(&backplane, off, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("sdio_read_addr(func1, 0x%05x, 4) failed: %d",
			off, ret);
		return ret;
	}

	*out = (uint32_t)buf[0] |
	       ((uint32_t)buf[1] << 8) |
	       ((uint32_t)buf[2] << 16) |
	       ((uint32_t)buf[3] << 24);
	return 0;
}

static int bcm43430_bringup(void)
{
	int ret;
	uint8_t reg;

	if (!device_is_ready(sdhc_dev)) {
		LOG_ERR("SDHC device %s not ready", sdhc_dev->name);
		return -ENODEV;
	}

	LOG_INF("--- BCM43430 SDIO subsystem bring-up ---");

	ret = sd_is_card_present(sdhc_dev);
	LOG_INF("sd_is_card_present: %d", ret);

	ret = sd_init(sdhc_dev, &card);
	if (ret != 0) {
		LOG_ERR("sd_init failed: %d", ret);
		return ret;
	}
	LOG_INF("sd_init ok: num_io=%u rca=0x%04x ocr=0x%08x bus_width=%u",
		(unsigned int)card.num_io, card.relative_addr, card.ocr,
		card.bus_width);

	ret = sdio_read_byte(&card.func0, SDIO_CCCR_CCCR, &reg);
	if (ret != 0) {
		LOG_ERR("CCCR rev read via subsys failed: %d", ret);
		return ret;
	}
	LOG_INF("CCCR rev byte (via subsys): 0x%02x", reg);

	/* Diagnostic: read CCCR caps + speed + current bus IF to verify
	 * the subsystem actually drove the chip to 4-bit. cccr_flags
	 * gates sdio_set_bus_width() on SDIO_SUPPORT_HS (from SHS bit in
	 * CCCR_SPEED) -- if that's missing, we stay at 1-bit even though
	 * the chip supports 4-bit.
	 */
	uint8_t caps_val = 0, speed_val = 0, busif_val = 0;

	(void)sdio_read_byte(&card.func0, SDIO_CCCR_CAPS, &caps_val);
	(void)sdio_read_byte(&card.func0, SDIO_CCCR_SPEED, &speed_val);
	(void)sdio_read_byte(&card.func0, SDIO_CCCR_BUS_IF, &busif_val);
	LOG_INF("CCCR diag: CAPS=0x%02x SPEED=0x%02x BUS_IF=0x%02x (chip width=%u-bit)",
		caps_val, speed_val, busif_val,
		(busif_val & SDIO_CCCR_BUS_IF_WIDTH_MASK) ==
			SDIO_CCCR_BUS_IF_WIDTH_4_BIT ? 4 : 1);
	LOG_INF("card.cccr_flags=0x%08x (HS=%d 4BIT_LS=%d MULTIBLK=%d)",
		card.cccr_flags,
		!!(card.cccr_flags & SDIO_SUPPORT_HS),
		!!(card.cccr_flags & SDIO_SUPPORT_4BIT_LS_BUS),
		!!(card.cccr_flags & SDIO_SUPPORT_MULTIBLOCK));
	LOG_INF("card.bus_io.bus_width=%u (1=1-bit 2=4-bit 3=8-bit) clock=%u",
		card.bus_io.bus_width, card.bus_io.clock);

	ret = sdio_init_func(&card, &backplane, SDIO_FUNC_NUM_1);
	if (ret != 0) {
		LOG_ERR("sdio_init_func(backplane=1) failed: %d", ret);
		return ret;
	}
	LOG_INF("sdio_init_func(backplane=1) ok: max_blk=%u",
		backplane.cis.max_blk_size);

	ret = sdio_enable_func(&backplane);
	if (ret != 0) {
		LOG_ERR("sdio_enable_func(backplane=1) failed: %d", ret);
		return ret;
	}
	LOG_INF("sdio_enable_func(backplane=1) ok");

	/* Mandatory: sdio_init_func leaves func->block_size = 0, which
	 * confuses sdio_io_rw_extended_helper into taking the block-mode
	 * branch (len > 0 always) and then dividing by zero. Set it
	 * explicitly to the brcmfmac-canonical 64 for function 1.
	 */
	ret = sdio_set_block_size(&backplane, 64);
	if (ret != 0) {
		LOG_ERR("sdio_set_block_size(backplane=1, 64) failed: %d", ret);
		return ret;
	}
	LOG_INF("sdio_set_block_size(backplane=1, 64) ok");

	uint32_t chipid_reg;
	ret = backplane_read32(BRCMF_SI_ENUM_BASE, &chipid_reg);
	if (ret != 0) {
		LOG_ERR("chipid readback failed: %d", ret);
		return ret;
	}
	uint32_t chip_id = chipid_reg & CID_ID_MASK;
	uint32_t chip_rev = (chipid_reg & CID_REV_MASK) >> CID_REV_SHIFT;
	uint32_t chip_type = (chipid_reg & CID_TYPE_MASK) >> CID_TYPE_SHIFT;

	LOG_INF("chipcommon[0] = 0x%08x  -> chip=%u (0x%04x) rev=%u type=%u (%s)",
		chipid_reg, chip_id, chip_id, chip_rev, chip_type,
		chip_type == 0 ? "SB" : (chip_type == 1 ? "AXI" : "?"));

	if (chip_id == 43430 && chip_rev == 1) {
		LOG_INF("  -> matches BCM43430A1");
	}

	/* PMU setup for downstream backplane access (firmware load, core
	 * enables). Runs after the chipid read -- chipcommon is always-on,
	 * doesn't need ALP forced.
	 */
	ret = sdio_write_byte(&backplane, SBSDIO_FUNC1_CHIPCLKCSR,
			      BRCMF_INIT_CLKCTL1);
	if (ret != 0) {
		LOG_ERR("CHIPCLKCSR write(0x%02x) failed: %d",
			BRCMF_INIT_CLKCTL1, ret);
		return ret;
	}

	uint8_t clkval = 0;
	for (int i = 0; i < 100; i++) {
		ret = sdio_read_byte(&backplane, SBSDIO_FUNC1_CHIPCLKCSR,
				     &clkval);
		if (ret != 0) {
			LOG_ERR("CHIPCLKCSR read failed: %d", ret);
			return ret;
		}
		if (clkval & SBSDIO_AVBITS) {
			break;
		}
		k_msleep(1);
	}
	if (!(clkval & SBSDIO_AVBITS)) {
		LOG_ERR("ALP/HT never available (CHIPCLKCSR=0x%02x)", clkval);
		return -ETIMEDOUT;
	}
	LOG_INF("CHIPCLKCSR=0x%02x (%s%s)", clkval,
		(clkval & SBSDIO_ALP_AVAIL) ? "ALP " : "",
		(clkval & SBSDIO_HT_AVAIL) ? "HT" : "");

	ret = sdio_write_byte(&backplane, SBSDIO_FUNC1_CHIPCLKCSR,
			      SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_FORCE_ALP);
	if (ret != 0) {
		LOG_ERR("CHIPCLKCSR force-ALP write failed: %d", ret);
		return ret;
	}
	k_busy_wait(65);

	ret = sdio_write_byte(&backplane, SBSDIO_FUNC1_SDIOPULLUP, 0);
	if (ret != 0) {
		LOG_ERR("SDIOPULLUP=0 write failed: %d", ret);
		return ret;
	}

	LOG_INF("--- subsystem bring-up complete ---");
	return 0;
}

SYS_INIT(bcm43430_bringup, APPLICATION, 99);
