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

#include <string.h>

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
static struct sdio_func backplane;  /* F1: backplane window + IOCTL register I/O */
static struct sdio_func radio;      /* F2: data + control IOCTL path */

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

/* 32-bit write at a backplane address (mirrors brcmf_sdiod_writel). */
static int backplane_write32(uint32_t addr, uint32_t val)
{
	uint8_t buf[4];
	int ret;

	ret = set_backplane_window(addr);
	if (ret != 0) {
		return ret;
	}

	uint32_t off = (addr & SBSDIO_SB_OFT_ADDR_MASK) | SBSDIO_SB_ACCESS_2_4B_FLAG;

	buf[0] = (uint8_t)(val & 0xFF);
	buf[1] = (uint8_t)((val >> 8) & 0xFF);
	buf[2] = (uint8_t)((val >> 16) & 0xFF);
	buf[3] = (uint8_t)((val >> 24) & 0xFF);

	ret = sdio_write_addr(&backplane, off, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("sdio_write_addr(func1, 0x%05x, 4) failed: %d",
			off, ret);
		return ret;
	}
	return 0;
}

/* Variable-length read/write at a backplane address. Transfer must fit
 * inside the current 32 KiB SBADDR window -- caller chooses the
 * address; this helper does not slide the window mid-burst.
 */
static int backplane_read_bytes(uint32_t addr, uint8_t *buf, uint32_t len)
{
	int ret = set_backplane_window(addr);
	if (ret != 0) {
		return ret;
	}

	uint32_t off = (addr & SBSDIO_SB_OFT_ADDR_MASK) | SBSDIO_SB_ACCESS_2_4B_FLAG;

	ret = sdio_read_addr(&backplane, off, buf, len);
	if (ret != 0) {
		LOG_ERR("sdio_read_addr(func1, 0x%05x, %u) failed: %d",
			off, len, ret);
	}
	return ret;
}

static int backplane_write_bytes(uint32_t addr, const uint8_t *buf, uint32_t len)
{
	int ret = set_backplane_window(addr);
	if (ret != 0) {
		return ret;
	}

	uint32_t off = (addr & SBSDIO_SB_OFT_ADDR_MASK) | SBSDIO_SB_ACCESS_2_4B_FLAG;

	/* Zephyr's sdio_write_addr takes a non-const buffer but does not
	 * modify it; cast is safe.
	 */
	ret = sdio_write_addr(&backplane, off, (uint8_t *)buf, len);
	if (ret != 0) {
		LOG_ERR("sdio_write_addr(func1, 0x%05x, %u) failed: %d",
			off, len, ret);
	}
	return ret;
}

/* === BCMA core enumeration (EROM scan) =====================================
 *
 * The BCM43430A1's AXI backplane has a hardware-described enumeration table
 * (DMP/EROM). chipcommon's `eromptr` register (offset 0xFC) points at the
 * table; we walk descriptors and harvest each core's slave-port base and
 * slave-wrap base. The wrap base is where each core's IOCTL/RESET_CTL
 * registers live -- a separate address space from the core's "data" base.
 *
 * Definitions mirror Linux's drivers/net/wireless/broadcom/brcm80211/
 * brcmfmac/chip.c (DMP/PL-368 descriptor spec).
 */

/* descriptor types */
#define DMP_DESC_TYPE_MSK       0x0000000F
#define DMP_DESC_EMPTY          0x00000000
#define DMP_DESC_VALID          0x00000001
#define DMP_DESC_COMPONENT      0x00000001
#define DMP_DESC_MASTER_PORT    0x00000003
#define DMP_DESC_ADDRESS        0x00000005
#define DMP_DESC_ADDRSIZE_GT32  0x00000008
#define DMP_DESC_EOT            0x0000000F

/* CompIdentA (first COMPONENT desc) */
#define DMP_COMP_PARTNUM        0x000FFF00
#define DMP_COMP_PARTNUM_S      8

/* CompIdentB (second COMPONENT desc) */
#define DMP_COMP_NUM_SWRAP      0x00F80000
#define DMP_COMP_NUM_SWRAP_S    19
#define DMP_COMP_NUM_MWRAP      0x0007C000
#define DMP_COMP_NUM_MWRAP_S    14

/* slave/wrap address descriptor */
#define DMP_SLAVE_ADDR_BASE     0xFFFFF000
#define DMP_SLAVE_TYPE          0x000000C0
#define DMP_SLAVE_TYPE_S        6
#define DMP_SLAVE_TYPE_SLAVE    0
#define DMP_SLAVE_TYPE_SWRAP    2
#define DMP_SLAVE_TYPE_MWRAP    3
#define DMP_SLAVE_SIZE_TYPE     0x00000030
#define DMP_SLAVE_SIZE_TYPE_S   4
#define DMP_SLAVE_SIZE_4K       0
#define DMP_SLAVE_SIZE_8K       1
#define DMP_SLAVE_SIZE_DESC     3

/* BCMA core IDs (subset). Full list in Linux's include/linux/bcma/bcma.h. */
#define BCMA_CORE_INTERNAL_MEM  0x80E    /* SOCRAM */
#define BCMA_CORE_80211         0x812    /* D11 MAC */
#define BCMA_CORE_PMU           0x827
#define BCMA_CORE_SDIO_DEV      0x829    /* SDIO device core */
#define BCMA_CORE_ARM_CM3       0x82A
#define BCMA_CORE_GCI           0x840

/* SDIO core register offsets within its data base. */
#define SDPCMD_INTSTATUS        0x20     /* W1C; write 0xFFFFFFFF to ack-all */

/* CHIPCLKCSR bit not already defined above. */
#define SBSDIO_HT_AVAIL_REQ     0x10     /* request HT clock from chip */

/* BCMA wrapper-register offsets (within a core's wrapbase) */
#define BCMA_IOCTL              0x0408
#define BCMA_IOCTL_CLK          0x0001
#define BCMA_IOCTL_FGC          0x0002
#define BCMA_RESET_CTL          0x0800
#define BCMA_RESET_CTL_RESET    0x0001

/* D11-specific IOCTL bits */
#define D11_BCMA_IOCTL_PHYCLOCKEN  0x0004
#define D11_BCMA_IOCTL_PHYRESET    0x0008

/* chipcommon offsets */
#define CC_EROMPTR_OFFSET       0xFC

/* SOCRAM register offsets within SOCRAM core base */
#define SOCRAM_BANKIDX_OFFSET   0x10
#define SOCRAM_BANKPDA_OFFSET   0x44

#define MAX_CORES 12

struct bcm_core {
	uint16_t id;
	uint32_t base;
	uint32_t wrapbase;
};

static struct bcm_core cores[MAX_CORES];
static unsigned int num_cores;

static int dmp_get_desc(uint32_t *erom_addr, uint32_t *val_out, uint8_t *type_out)
{
	int ret = backplane_read32(*erom_addr, val_out);
	if (ret != 0) {
		return ret;
	}
	*erom_addr += 4;

	if (type_out != NULL) {
		uint8_t t = (uint8_t)(*val_out & DMP_DESC_TYPE_MSK);
		/* an ADDRESS desc with ADDRSIZE_GT32 still classifies as ADDRESS */
		if ((t & ~DMP_DESC_ADDRSIZE_GT32) == DMP_DESC_ADDRESS) {
			t = DMP_DESC_ADDRESS;
		}
		*type_out = t;
	}
	return 0;
}

static int dmp_get_regaddr(uint32_t *erom_addr, uint32_t *regbase, uint32_t *wrapbase)
{
	uint8_t desc;
	uint32_t val;
	int ret;
	uint8_t wraptype;

	*regbase = 0;
	*wrapbase = 0;

	ret = dmp_get_desc(erom_addr, &val, &desc);
	if (ret != 0) {
		return ret;
	}

	if (desc == DMP_DESC_MASTER_PORT) {
		wraptype = DMP_SLAVE_TYPE_MWRAP;
	} else if (desc == DMP_DESC_ADDRESS) {
		/* no master port: this is the first slave-address desc.
		 * Revert so the loop below re-consumes it, and look for SWRAP.
		 */
		*erom_addr -= 4;
		wraptype = DMP_SLAVE_TYPE_SWRAP;
	} else {
		*erom_addr -= 4;
		return -EILSEQ;
	}

	do {
		/* find next ADDRESS desc or hit a component boundary */
		do {
			ret = dmp_get_desc(erom_addr, &val, &desc);
			if (ret != 0) {
				return ret;
			}
			if (desc == DMP_DESC_EOT) {
				*erom_addr -= 4;
				return -EFAULT;
			}
		} while (desc != DMP_DESC_ADDRESS && desc != DMP_DESC_COMPONENT);

		if (desc == DMP_DESC_COMPONENT) {
			*erom_addr -= 4;
			return 0;
		}

		/* skip upper 32 bits of 64-bit address */
		if (val & DMP_DESC_ADDRSIZE_GT32) {
			uint32_t tmp;
			ret = dmp_get_desc(erom_addr, &tmp, NULL);
			if (ret != 0) {
				return ret;
			}
		}

		uint8_t sztype = (val & DMP_SLAVE_SIZE_TYPE) >> DMP_SLAVE_SIZE_TYPE_S;
		if (sztype == DMP_SLAVE_SIZE_DESC) {
			uint32_t szdesc;
			ret = dmp_get_desc(erom_addr, &szdesc, NULL);
			if (ret != 0) {
				return ret;
			}
			if (szdesc & DMP_DESC_ADDRSIZE_GT32) {
				uint32_t tmp;
				ret = dmp_get_desc(erom_addr, &tmp, NULL);
				if (ret != 0) {
					return ret;
				}
			}
		}

		if (sztype != DMP_SLAVE_SIZE_4K && sztype != DMP_SLAVE_SIZE_8K) {
			continue;
		}

		uint8_t stype = (val & DMP_SLAVE_TYPE) >> DMP_SLAVE_TYPE_S;
		if (*regbase == 0 && stype == DMP_SLAVE_TYPE_SLAVE) {
			*regbase = val & DMP_SLAVE_ADDR_BASE;
		}
		if (*wrapbase == 0 && stype == wraptype) {
			*wrapbase = val & DMP_SLAVE_ADDR_BASE;
		}
	} while (*regbase == 0 || *wrapbase == 0);

	return 0;
}

static int erom_scan(void)
{
	uint32_t erom_addr;
	int ret;

	ret = backplane_read32(BRCMF_SI_ENUM_BASE + CC_EROMPTR_OFFSET, &erom_addr);
	if (ret != 0) {
		LOG_ERR("erom_scan: read eromptr failed: %d", ret);
		return ret;
	}
	LOG_INF("erom_scan: eromptr=0x%08x", erom_addr);

	num_cores = 0;
	uint8_t desc_type = 0;

	while (desc_type != DMP_DESC_EOT) {
		uint32_t val;
		ret = dmp_get_desc(&erom_addr, &val, &desc_type);
		if (ret != 0) {
			LOG_ERR("erom_scan: desc read failed: %d", ret);
			return ret;
		}
		if (!(val & DMP_DESC_VALID)) {
			continue;
		}
		if (desc_type == DMP_DESC_EMPTY) {
			continue;
		}
		if (desc_type != DMP_DESC_COMPONENT) {
			continue;
		}

		uint16_t id = (val & DMP_COMP_PARTNUM) >> DMP_COMP_PARTNUM_S;

		/* CompIdentB: nmw / nsw / rev */
		uint32_t ident_b;
		ret = dmp_get_desc(&erom_addr, &ident_b, &desc_type);
		if (ret != 0) {
			return ret;
		}
		if ((ident_b & DMP_DESC_TYPE_MSK) != DMP_DESC_COMPONENT) {
			LOG_ERR("erom_scan: expected CompIdentB, got 0x%08x", ident_b);
			return -EILSEQ;
		}

		uint8_t nmw = (ident_b & DMP_COMP_NUM_MWRAP) >> DMP_COMP_NUM_MWRAP_S;
		uint8_t nsw = (ident_b & DMP_COMP_NUM_SWRAP) >> DMP_COMP_NUM_SWRAP_S;

		if (nmw + nsw == 0 && id != BCMA_CORE_PMU && id != BCMA_CORE_GCI) {
			continue;
		}

		uint32_t base = 0, wrap = 0;
		ret = dmp_get_regaddr(&erom_addr, &base, &wrap);
		if (ret != 0) {
			continue;
		}

		if (num_cores >= MAX_CORES) {
			LOG_WRN("erom_scan: MAX_CORES overflow at id=0x%03x", id);
			continue;
		}
		cores[num_cores].id = id;
		cores[num_cores].base = base;
		cores[num_cores].wrapbase = wrap;
		LOG_INF("erom_scan: core[%u] id=0x%03x base=0x%08x wrap=0x%08x",
			num_cores, id, base, wrap);
		num_cores++;
	}

	LOG_INF("erom_scan: discovered %u cores", num_cores);
	return 0;
}

static const struct bcm_core *core_find(uint16_t id)
{
	for (unsigned int i = 0; i < num_cores; i++) {
		if (cores[i].id == id) {
			return &cores[i];
		}
	}
	return NULL;
}

/* === AI (AXI) core reset/enable ============================================
 *
 * Mirrors Linux's brcmf_chip_ai_{iscoreup,coredisable,resetcore}.
 * Each core has a "wrapper" address space (separate from its data base)
 * where IOCTL and RESET_CTL live.
 */

static bool ai_iscoreup(uint32_t wrap)
{
	uint32_t v;
	if (backplane_read32(wrap + BCMA_IOCTL, &v) != 0) {
		return false;
	}
	bool clk_ok = (v & (BCMA_IOCTL_FGC | BCMA_IOCTL_CLK)) == BCMA_IOCTL_CLK;
	if (backplane_read32(wrap + BCMA_RESET_CTL, &v) != 0) {
		return false;
	}
	return clk_ok && ((v & BCMA_RESET_CTL_RESET) == 0);
}

static int ai_coredisable(uint32_t wrap, uint32_t prereset, uint32_t reset)
{
	uint32_t v;
	int ret;

	ret = backplane_read32(wrap + BCMA_RESET_CTL, &v);
	if (ret != 0) {
		return ret;
	}

	if ((v & BCMA_RESET_CTL_RESET) == 0) {
		/* not in reset -- drive it in */
		(void)backplane_write32(wrap + BCMA_IOCTL,
					prereset | BCMA_IOCTL_FGC | BCMA_IOCTL_CLK);
		(void)backplane_read32(wrap + BCMA_IOCTL, &v); /* flush */

		(void)backplane_write32(wrap + BCMA_RESET_CTL, BCMA_RESET_CTL_RESET);
		k_busy_wait(20);

		for (int i = 0; i < 300; i++) {
			(void)backplane_read32(wrap + BCMA_RESET_CTL, &v);
			if (v == BCMA_RESET_CTL_RESET) {
				break;
			}
			k_busy_wait(1);
		}
	}

	(void)backplane_write32(wrap + BCMA_IOCTL,
				reset | BCMA_IOCTL_FGC | BCMA_IOCTL_CLK);
	(void)backplane_read32(wrap + BCMA_IOCTL, &v);  /* flush */
	return 0;
}

static int ai_resetcore(uint32_t wrap, uint32_t prereset, uint32_t reset,
			uint32_t postreset)
{
	uint32_t v;
	int ret = ai_coredisable(wrap, prereset, reset);
	if (ret != 0) {
		return ret;
	}

	/* take core out of reset */
	for (int count = 0; count < 50; count++) {
		(void)backplane_read32(wrap + BCMA_RESET_CTL, &v);
		if ((v & BCMA_RESET_CTL_RESET) == 0) {
			break;
		}
		(void)backplane_write32(wrap + BCMA_RESET_CTL, 0);
		k_busy_wait(50);
	}

	(void)backplane_write32(wrap + BCMA_IOCTL, postreset | BCMA_IOCTL_CLK);
	(void)backplane_read32(wrap + BCMA_IOCTL, &v);  /* flush */
	return 0;
}

/* === set_passive: prepare BCM43430A1 for firmware upload ===================
 *
 * Mirrors Linux's brcmf_chip_cm3_set_passive (BCM43430 uses ARM CM3):
 *   1. halt ARM CM3 (so it doesn't fight us during the upload)
 *   2. reset D11 MAC with PHYRESET + PHYCLOCKEN active
 *   3. reset SOCRAM (brings it out of POR to operational)
 *   4. disable bank-3 remap (BCM43430-specific quirk)
 */
static int set_passive(void)
{
	const struct bcm_core *arm = core_find(BCMA_CORE_ARM_CM3);
	const struct bcm_core *d11 = core_find(BCMA_CORE_80211);
	const struct bcm_core *sr  = core_find(BCMA_CORE_INTERNAL_MEM);

	if (arm == NULL || d11 == NULL || sr == NULL) {
		LOG_ERR("set_passive: missing core(s) arm=%p d11=%p sr=%p",
			(const void *)arm, (const void *)d11, (const void *)sr);
		return -ENODEV;
	}

	LOG_INF("set_passive: halt ARM CM3 (wrap=0x%08x)", arm->wrapbase);
	int ret = ai_coredisable(arm->wrapbase, 0, 0);
	if (ret != 0) {
		LOG_ERR("set_passive: CM3 disable failed: %d", ret);
		return ret;
	}

	LOG_INF("set_passive: reset D11 PHY (wrap=0x%08x)", d11->wrapbase);
	ret = ai_resetcore(d11->wrapbase,
			   D11_BCMA_IOCTL_PHYRESET | D11_BCMA_IOCTL_PHYCLOCKEN,
			   D11_BCMA_IOCTL_PHYCLOCKEN,
			   D11_BCMA_IOCTL_PHYCLOCKEN);
	if (ret != 0) {
		LOG_ERR("set_passive: D11 reset failed: %d", ret);
		return ret;
	}

	LOG_INF("set_passive: reset SOCRAM (wrap=0x%08x base=0x%08x)",
		sr->wrapbase, sr->base);
	ret = ai_resetcore(sr->wrapbase, 0, 0, 0);
	if (ret != 0) {
		LOG_ERR("set_passive: SOCRAM reset failed: %d", ret);
		return ret;
	}

	/* BCM43430 quirk: bank-3 remap aliases SOCRAM to a ROM region; turn
	 * it off so firmware writes land in actual RAM.
	 */
	(void)backplane_write32(sr->base + SOCRAM_BANKIDX_OFFSET, 3);
	(void)backplane_write32(sr->base + SOCRAM_BANKPDA_OFFSET, 0);
	LOG_INF("set_passive: bank-3 remap disabled");

	if (!ai_iscoreup(sr->wrapbase)) {
		LOG_ERR("set_passive: SOCRAM did not come up");
		return -EIO;
	}
	LOG_INF("set_passive: SOCRAM is up");
	return 0;
}

/* === set_active: release ARM CM3 + confirm firmware boot ===================
 *
 * Mirrors Linux's brcmf_chip_cm3_set_active + brcmf_sdio_clkctl(CLK_AVAIL):
 *   1. Activate the chip-side SDIO core: clear any pending interrupts so
 *      the firmware sees a fresh start.
 *   2. Release ARM CM3 from reset. The CM3 starts executing from its
 *      reset vector at SOCRAM[0], i.e. the byte we wrote first during the
 *      firmware upload.
 *   3. Request HT clock by writing SBSDIO_HT_AVAIL_REQ to CHIPCLKCSR, then
 *      poll for SBSDIO_HT_AVAIL. The chip's PMU only brings HT up after the
 *      firmware boots far enough to enable its DPLL -- so HT_AVAIL = "fw
 *      is alive and running."
 *   4. Sanity-read chipcommon[0] to confirm the SDIO bridge still works.
 *      (chipcommon is always-on; this is a "chip didn't die" check, not a
 *      "firmware is up" check.)
 */
static int set_active(void)
{
	const struct bcm_core *sdio_dev = core_find(BCMA_CORE_SDIO_DEV);
	const struct bcm_core *arm = core_find(BCMA_CORE_ARM_CM3);

	if (sdio_dev == NULL || arm == NULL) {
		LOG_ERR("set_active: missing core(s) sdio=%p arm=%p",
			(const void *)sdio_dev, (const void *)arm);
		return -ENODEV;
	}

	LOG_INF("set_active: clear SDIO core intstatus (base=0x%08x)",
		sdio_dev->base);
	int ret = backplane_write32(sdio_dev->base + SDPCMD_INTSTATUS,
				    0xFFFFFFFFu);
	if (ret != 0) {
		LOG_ERR("set_active: intstatus clear failed: %d", ret);
		return ret;
	}

	LOG_INF("set_active: release ARM CM3 (wrap=0x%08x)", arm->wrapbase);
	ret = ai_resetcore(arm->wrapbase, 0, 0, 0);
	if (ret != 0) {
		LOG_ERR("set_active: ARM resetcore failed: %d", ret);
		return ret;
	}

	LOG_INF("set_active: request HT clock");
	ret = sdio_write_byte(&backplane, SBSDIO_FUNC1_CHIPCLKCSR,
			      SBSDIO_HT_AVAIL_REQ);
	if (ret != 0) {
		LOG_ERR("set_active: CHIPCLKCSR write failed: %d", ret);
		return ret;
	}

	uint8_t clkcsr = 0;
	int64_t t0 = k_uptime_get();
	for (int i = 0; i < 500; i++) {
		ret = sdio_read_byte(&backplane, SBSDIO_FUNC1_CHIPCLKCSR,
				     &clkcsr);
		if (ret != 0) {
			LOG_ERR("set_active: CHIPCLKCSR read failed: %d", ret);
			return ret;
		}
		if (clkcsr & SBSDIO_HT_AVAIL) {
			break;
		}
		k_msleep(1);
	}
	int64_t t_wait = k_uptime_get() - t0;

	if (!(clkcsr & SBSDIO_HT_AVAIL)) {
		LOG_ERR("set_active: HT_AVAIL timeout (CHIPCLKCSR=0x%02x after %lld ms)",
			clkcsr, (long long)t_wait);
		return -ETIMEDOUT;
	}
	LOG_INF("set_active: HT_AVAIL after %lld ms (CHIPCLKCSR=0x%02x)",
		(long long)t_wait, clkcsr);

	uint32_t chipid;
	ret = backplane_read32(BRCMF_SI_ENUM_BASE, &chipid);
	if (ret != 0) {
		LOG_ERR("set_active: post-boot chipid read failed: %d", ret);
		return ret;
	}
	LOG_INF("set_active: post-boot chipid = 0x%08x (chip alive, fw running)",
		chipid);
	return 0;
}

/* === Spike C: first BCDC IOCTL to running firmware =========================
 *
 * Wire format for an SDIO IOCTL (mirrors zerowi's IOCTL_MSG, which matches
 * Linux brcmfmac's SDPCM + BCDC stack):
 *   [4 B  SDPCM frame:  uint16 len, uint16 ~len]
 *   [8 B  SDPCM sw hdr: seq, chan, nextlen, hdrlen, flow, credit, rsv x2]
 *   [16 B BCDC/CDC hdr: cmd, outlen, inlen, flags, status]
 *   [payload: variable name (incl. NUL) for GET, or data for SET]
 *   [padded to 4-byte alignment]
 *
 * For GET_VAR("cur_etheraddr"): payload = name+NUL on the way out, chip
 * replies with the 6-byte MAC in the response payload. This is a one-shot
 * probe -- proves F2 + the BCDC protocol stack work; doesn't try to model
 * the full driver yet.
 */

#define WLC_GET_VAR             262
#define SDPCM_CHAN_CTRL         0
#define BCDC_FLAG_ERROR         0x01
#define BCDC_FLAG_SET           0x02
#define BCDC_REQ_ID_SHIFT       16

#define F2_FIFO_ADDR            0x8000      /* with SB_ACCESS_2_4B_FLAG bit */
#define F2_BLOCK_SIZE           512

struct sdpcm_frame_hdr {
	uint16_t len;
	uint16_t notlen;
} __packed;

struct sdpcm_sw_hdr {
	uint8_t seq;
	uint8_t chan;
	uint8_t nextlen;
	uint8_t hdrlen;
	uint8_t flow;
	uint8_t credit;
	uint8_t reserved[2];
} __packed;

struct cdc_hdr {
	uint32_t cmd;
	uint16_t outlen;
	uint16_t inlen;
	uint32_t flags;
	uint32_t status;
} __packed;

static uint8_t  sdpcm_txseq;
static uint16_t bcdc_reqid;

static int spike_c_first_ioctl(void)
{
	int ret;
	const char *var = "cur_etheraddr";
	const size_t var_len = strlen(var) + 1;  /* include trailing NUL */

	LOG_INF("--- Spike C: F2 setup + first IOCTL ---");

	ret = sdio_init_func(&card, &radio, SDIO_FUNC_NUM_2);
	if (ret != 0) {
		LOG_ERR("Spike C: sdio_init_func(F2) failed: %d", ret);
		return ret;
	}
	ret = sdio_enable_func(&radio);
	if (ret != 0) {
		LOG_ERR("Spike C: sdio_enable_func(F2) failed: %d", ret);
		return ret;
	}
	ret = sdio_set_block_size(&radio, F2_BLOCK_SIZE);
	if (ret != 0) {
		LOG_ERR("Spike C: sdio_set_block_size(F2, %u) failed: %d",
			F2_BLOCK_SIZE, ret);
		return ret;
	}
	LOG_INF("Spike C: F2 enabled, block_size=%u", F2_BLOCK_SIZE);

	static uint8_t tx_buf[128] __aligned(4);
	static uint8_t rx_buf[256] __aligned(4);

	memset(tx_buf, 0, sizeof(tx_buf));

	struct sdpcm_frame_hdr *frame = (void *)tx_buf;
	struct sdpcm_sw_hdr *sw = (void *)(tx_buf + sizeof(*frame));
	struct cdc_hdr *cdc = (void *)((uint8_t *)sw + sizeof(*sw));
	uint8_t *payload = (uint8_t *)cdc + sizeof(*cdc);

	const size_t hdr_len = sizeof(*frame) + sizeof(*sw) + sizeof(*cdc);
	const size_t payload_len = var_len;  /* GET: send just the name */
	const size_t total = hdr_len + payload_len;
	const size_t padded = (total + 3) & ~(size_t)3;

	frame->len = (uint16_t)total;
	frame->notlen = (uint16_t)~frame->len;
	sw->seq = sdpcm_txseq++;
	sw->chan = SDPCM_CHAN_CTRL;
	sw->hdrlen = (uint8_t)(sizeof(*frame) + sizeof(*sw));   /* = 12 */

	cdc->cmd = WLC_GET_VAR;
	cdc->outlen = (uint16_t)payload_len;
	cdc->inlen = 0;
	bcdc_reqid++;
	cdc->flags = ((uint32_t)bcdc_reqid << BCDC_REQ_ID_SHIFT);

	memcpy(payload, var, var_len);

	LOG_INF("Spike C: TX GET_VAR(\"%s\") len=%u padded=%u reqid=%u",
		var, (unsigned)total, (unsigned)padded, bcdc_reqid);

	ret = sdio_write_addr(&radio, F2_FIFO_ADDR, tx_buf, padded);
	if (ret != 0) {
		LOG_ERR("Spike C: F2 write failed: %d", ret);
		return ret;
	}

	/* Give the firmware time to handle the IOCTL. */
	k_msleep(50);

	/* Drain F2 until we find a control-channel frame (chan=0) whose CDC
	 * request-ID matches ours. The chip can have queued event frames
	 * (chan=1) and/or credit-grant signaling frames from the post-boot
	 * window before our IOCTL response. A real driver would poll the
	 * SDIO core's INT_STATUS bits to know what's available; for the spike
	 * we just read in a loop with a brief sleep.
	 */
	const int max_retries = 20;
	bool matched = false;
	uint16_t rsp_outlen = 0;
	uint8_t *rpayload = NULL;
	struct cdc_hdr *rcdc = NULL;

	for (int retry = 0; retry < max_retries && !matched; retry++) {
		memset(rx_buf, 0, sizeof(rx_buf));
		ret = sdio_read_addr(&radio, F2_FIFO_ADDR, rx_buf, sizeof(rx_buf));
		if (ret != 0) {
			LOG_ERR("Spike C: F2 read #%d failed: %d", retry, ret);
			return ret;
		}

		struct sdpcm_frame_hdr *rfrm = (void *)rx_buf;
		struct sdpcm_sw_hdr *rsw = (void *)(rx_buf + sizeof(*rfrm));

		uint16_t hdr_xor = rfrm->len ^ rfrm->notlen;
		if (hdr_xor != 0xFFFF) {
			LOG_WRN("Spike C: read #%d: bad frame xor 0x%04x (len=%u notlen=0x%04x)",
				retry, hdr_xor, rfrm->len, rfrm->notlen);
			LOG_HEXDUMP_INF(rx_buf, 32, "rx[:32]");
			k_msleep(10);
			continue;
		}

		LOG_INF("Spike C: rx #%d: len=%u seq=%u chan=%u credit=%u hdrlen=%u",
			retry, rfrm->len, rsw->seq, rsw->chan,
			rsw->credit, rsw->hdrlen);

		if (rsw->chan != SDPCM_CHAN_CTRL) {
			/* event (chan=1), data (chan=2), or signalling: skip */
			k_msleep(10);
			continue;
		}

		rcdc = (void *)((uint8_t *)rsw + sizeof(*rsw));
		uint16_t rsp_reqid = (uint16_t)(rcdc->flags >> BCDC_REQ_ID_SHIFT);
		LOG_INF("Spike C: rx #%d cdc: cmd=%u outlen=%u flags=0x%08x status=%u reqid=%u",
			retry, rcdc->cmd, rcdc->outlen, rcdc->flags,
			rcdc->status, rsp_reqid);

		if (rsp_reqid != bcdc_reqid) {
			LOG_WRN("Spike C: rx #%d: control frame but reqid mismatch (got %u, want %u)",
				retry, rsp_reqid, bcdc_reqid);
			k_msleep(10);
			continue;
		}

		if (rcdc->flags & BCDC_FLAG_ERROR) {
			LOG_ERR("Spike C: chip returned BCDC error (status=%u)",
				rcdc->status);
			return -EIO;
		}

		rpayload = (uint8_t *)rcdc + sizeof(*rcdc);
		rsp_outlen = rcdc->outlen;
		matched = true;
	}

	if (!matched) {
		LOG_ERR("Spike C: no IOCTL response after %d retries", max_retries);
		return -ETIMEDOUT;
	}

	LOG_INF("Spike C: chip MAC = %02x:%02x:%02x:%02x:%02x:%02x",
		rpayload[0], rpayload[1], rpayload[2],
		rpayload[3], rpayload[4], rpayload[5]);
	uint16_t pl_show = MIN((uint16_t)32, rsp_outlen);
	LOG_HEXDUMP_INF(rpayload, pl_show, "response payload");

	LOG_INF("--- Spike C complete ---");
	return 0;
}

/* === Bulk RAM read/write across the F1 backplane window ====================
 *
 * Mirrors Linux's brcmf_sdiod_ramrw. Each chunk:
 *   1. Slide the SBADDR window to cover the current chip-side address.
 *   2. Transfer up to 32 KiB (one F1 window worth) via CMD53 (Zephyr's
 *      sdio_(read|write)_addr picks block mode when len > block_size).
 *   3. Continue across windows until size bytes done.
 */
#define SBSDIO_SB_OFT_ADDR_LIMIT  0x8000  /* 32 KiB F1 window size */

/* Cap per-CMD53 block-mode transfers at 511 blocks (not the spec's max 512).
 * The 9-bit block-count field encodes 0 as 512, but on BCM43430A1 + BCM2835
 * Arasan, the 512-block CMD53 wedges the chip-side state machine -- after
 * the write completes (no controller-side error), the chip becomes
 * unresponsive (DATA_TIMEOUT on the next CMD53). Other working drivers
 * for this chip family also cap at 511 empirically. With block_size=64
 * that's 511*64 = 32704 bytes per CMD53; per F1 window (32 KiB = 32768)
 * we issue 511 blocks + one 64-byte byte-mode tail.
 */
#define MAX_CMD53_BLOCK_BYTES  (511 * 64)

#define BCM43430_VERIFY_UPLOAD     1   /* post-upload readback verify (cheap) */
#define BCM43430_VERIFY_PER_WINDOW 0   /* readback after each window (debug only) */

/* forward decl so the per-window verify hook can call into verify_memory */
static int verify_memory(uint32_t chip_addr, const uint8_t *expected, uint32_t size);

static int ramrw(bool write, uint32_t chip_addr, uint8_t *buf, uint32_t size)
{
	while (size > 0) {
		uint32_t window_chip_addr = chip_addr;
		const uint8_t *window_buf = buf;

		uint32_t sdaddr = chip_addr & SBSDIO_SB_OFT_ADDR_MASK;
		uint32_t window_left = SBSDIO_SB_OFT_ADDR_LIMIT - sdaddr;
		uint32_t chunk = MIN(size, window_left);
		/* Cap at 511 blocks per CMD53 -- see MAX_CMD53_BLOCK_BYTES note. */
		if (chunk > MAX_CMD53_BLOCK_BYTES) {
			chunk = MAX_CMD53_BLOCK_BYTES;
		}
		/* The F1->backplane bridge with the SB_ACCESS_2_4B_FLAG bit
		 * set requires 4-aligned transfer counts -- each 4-byte run
		 * is one 32-bit backplane access. Linux's brcmf_sdiod_ramrw
		 * rounds up via skb->len + 3 padding; we split off the 1-3
		 * byte tail and write it through a 4-byte staging buffer
		 * (pad bytes land in unused chip-RAM past the logical end).
		 */
		uint32_t aligned_chunk = chunk & ~3u;
		uint32_t tail = chunk - aligned_chunk;

		int ret = set_backplane_window(chip_addr);
		if (ret != 0) {
			LOG_ERR("ramrw: SBADDR set @ chip 0x%08x failed: %d",
				chip_addr, ret);
			return ret;
		}

		uint32_t off = sdaddr | SBSDIO_SB_ACCESS_2_4B_FLAG;

		if (aligned_chunk > 0) {
			ret = write
				? sdio_write_addr(&backplane, off, buf, aligned_chunk)
				: sdio_read_addr(&backplane, off, buf, aligned_chunk);
			if (ret != 0) {
				LOG_ERR("ramrw: %s @ chip 0x%08x len %u failed: %d",
					write ? "write" : "read",
					chip_addr, aligned_chunk, ret);
				return ret;
			}
		}

		if (tail > 0) {
			uint8_t tail_buf[4] __aligned(4) = {0};
			uint32_t tail_off = off + aligned_chunk;

			if (write) {
				memcpy(tail_buf, buf + aligned_chunk, tail);
				ret = sdio_write_addr(&backplane, tail_off,
						      tail_buf, sizeof(tail_buf));
			} else {
				ret = sdio_read_addr(&backplane, tail_off,
						     tail_buf, sizeof(tail_buf));
				if (ret == 0) {
					memcpy(buf + aligned_chunk, tail_buf, tail);
				}
			}
			if (ret != 0) {
				LOG_ERR("ramrw: %s tail @ chip 0x%08x len %u failed: %d",
					write ? "write" : "read",
					chip_addr + aligned_chunk, tail, ret);
				return ret;
			}
		}

#if BCM43430_VERIFY_PER_WINDOW
		if (write) {
			int v = verify_memory(window_chip_addr, window_buf, chunk);
			if (v != 0) {
				LOG_ERR("ramrw: per-window verify FAIL @ chip 0x%08x len %u",
					window_chip_addr, chunk);
				return v;
			}
			LOG_INF("ramrw: per-window verify OK @ chip 0x%08x len %u",
				window_chip_addr, chunk);
		}
#endif

		buf += chunk;
		chip_addr += chunk;
		size -= chunk;
	}
	return 0;
}

/* Read back from chip RAM and compare against `expected`. On the first
 * mismatch, log the divergent byte and return -EIO. Chunked through a
 * static buffer to avoid stack pressure on the long firmware verify.
 */
static int verify_memory(uint32_t chip_addr, const uint8_t *expected,
			 uint32_t size)
{
	static uint8_t readback[1024];
	uint32_t verified = 0;

	while (size > 0) {
		uint32_t chunk = MIN(size, (uint32_t)sizeof(readback));
		int ret = ramrw(false, chip_addr, readback, chunk);
		if (ret != 0) {
			return ret;
		}
		for (uint32_t i = 0; i < chunk; i++) {
			if (readback[i] != expected[i]) {
				LOG_ERR("verify: mismatch @ offset %u (chip 0x%08x): exp 0x%02x got 0x%02x",
					verified + i, chip_addr + i,
					expected[i], readback[i]);
				return -EIO;
			}
		}
		chip_addr += chunk;
		expected += chunk;
		size -= chunk;
		verified += chunk;
	}
	return 0;
}

/* === NVRAM strip ===========================================================
 *
 * Convert key=value text NVRAM into the chip-format the firmware expects:
 *   - Skip '#' comments, empty lines, leading/trailing whitespace.
 *   - Concat valid lines NUL-separated.
 *   - Pad to 4-byte alignment with NULs.
 *   - Append 4-byte LE "token" footer: (~n << 16) | (n & 0xFFFF), where
 *     n = padded_len/4. Firmware reads this footer at boot to locate the
 *     NVRAM table at the top of SOCRAM.
 *
 * Minimal port of Linux's brcmf_fw_nvram_strip -- the upstream version also
 * handles multi-device (devpath/PCIe) selection and MAC-from-platform
 * overrides; our NVRAM is single-device + has a placeholder MAC, so we
 * skip both. Returns bytes written to `out`, or negative errno.
 */
static int nvram_strip(const uint8_t *in, uint32_t in_len,
		       uint8_t *out, uint32_t out_capacity)
{
	uint32_t op = 0;
	uint32_t ip = 0;
	const uint32_t need_tail = 4 + 3;  /* token + worst-case alignment pad */

	while (ip < in_len) {
		while (ip < in_len &&
		       (in[ip] == ' ' || in[ip] == '\t' ||
			in[ip] == '\r' || in[ip] == '\n')) {
			ip++;
		}
		if (ip >= in_len) {
			break;
		}
		if (in[ip] == '#') {
			while (ip < in_len && in[ip] != '\n') {
				ip++;
			}
			continue;
		}

		uint32_t line_start = op;
		while (ip < in_len && in[ip] != '\n' && in[ip] != '\r') {
			if (op + need_tail >= out_capacity) {
				LOG_ERR("nvram_strip: out overflow @ in %u out %u",
					ip, op);
				return -EOVERFLOW;
			}
			out[op++] = in[ip++];
		}
		while (op > line_start &&
		       (out[op - 1] == ' ' || out[op - 1] == '\t')) {
			op--;
		}
		if (op > line_start) {
			out[op++] = '\0';
		}
	}

	uint32_t padded = (op + 3) & ~3u;
	if (padded + 4 > out_capacity) {
		return -EOVERFLOW;
	}
	while (op < padded) {
		out[op++] = '\0';
	}

	uint32_t n = padded / 4;
	uint32_t token = (~n << 16) | (n & 0xFFFF);
	out[op++] = (uint8_t)(token & 0xFF);
	out[op++] = (uint8_t)((token >> 8) & 0xFF);
	out[op++] = (uint8_t)((token >> 16) & 0xFF);
	out[op++] = (uint8_t)((token >> 24) & 0xFF);

	return (int)op;
}

/* Embedded blobs, generated by `xxd -i` from the linux-firmware files
 * staged at ports/zephyr/firmware/brcm/. See brcmfmac_fw.c, brcmfmac_nvram.c.
 */
extern const unsigned char brcmfmac_fw[];
extern const unsigned int brcmfmac_fw_len;
extern const unsigned char brcmfmac_nvram[];
extern const unsigned int brcmfmac_nvram_len;

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

	/* Discover chip cores via the EROM table, then put the chip into
	 * "passive" state (ARM halted, SOCRAM out of reset, bank-3 remap
	 * disabled). After this, SOCRAM-addressed backplane reads/writes
	 * succeed -- without it they fail with DATA_CRC at the SDIO bus
	 * (Wall #4, isolated 2026-05-13).
	 */
	ret = erom_scan();
	if (ret != 0) {
		LOG_ERR("erom_scan failed: %d", ret);
		return ret;
	}

	ret = set_passive();
	if (ret != 0) {
		LOG_ERR("set_passive failed: %d", ret);
		return ret;
	}

	/* ---- Spike A: backplane write-path probe ----
	 * Tests whether byte-mode CMD53 (windowed) writes reach the chip
	 * and whether SBADDR sliding works for writes. The probes target
	 * chip-side SOCRAM addresses; SOCRAM has NOT been reset out of POR
	 * (no set_passive yet), so writes may be silently dropped -- that
	 * is informative either way:
	 *   OK            -- readback matches the pattern.
	 *   WRITE_IGNORED -- readback == before-value (write didn't stick).
	 *   MISMATCH      -- readback != before and != pattern.
	 */
	LOG_INF("--- Spike A: backplane write probe ---");

	const uint32_t probe_pat = 0xDEADBEEFu;
	static const struct {
		uint32_t addr;
		const char *label;
	} probes_32b[] = {
		{ 0x00000000u, "SOCRAM[0]" },
		{ 0x00000100u, "SOCRAM[0x100]" },
		{ 0x00004000u, "SOCRAM[0x4000]" },
		{ 0x00008004u, "SOCRAM[0x8004] (win+1)" },
	};

	for (size_t i = 0; i < ARRAY_SIZE(probes_32b); i++) {
		uint32_t before = 0, after = 0;
		int rc;

		rc = backplane_read32(probes_32b[i].addr, &before);
		if (rc != 0) {
			LOG_ERR("probe %s: pre-read failed: %d",
				probes_32b[i].label, rc);
			continue;
		}
		rc = backplane_write32(probes_32b[i].addr, probe_pat);
		if (rc != 0) {
			LOG_ERR("probe %s: write failed: %d",
				probes_32b[i].label, rc);
			continue;
		}
		rc = backplane_read32(probes_32b[i].addr, &after);
		if (rc != 0) {
			LOG_ERR("probe %s: post-read failed: %d",
				probes_32b[i].label, rc);
			continue;
		}
		LOG_INF("probe32 %-22s before=0x%08x after=0x%08x %s",
			probes_32b[i].label, before, after,
			(after == probe_pat) ? "OK" :
			(after == before)    ? "WRITE_IGNORED" : "MISMATCH");
	}

	/* 16-byte byte-mode burst at SOCRAM[0x40]. Exercises multi-byte
	 * CMD53 byte-mode (len=16 < block_size=64 -> Zephyr's helper stays
	 * in byte mode, single CMD53 transaction). Pattern is incrementing
	 * bytes so byte-order bugs are visible in the hex dump.
	 */
	{
		const uint32_t base = 0x00000040u;
		uint8_t pat[16];
		uint8_t before[16] = {0};
		uint8_t after[16]  = {0};
		int rc;

		for (size_t i = 0; i < sizeof(pat); i++) {
			pat[i] = 0xA0u + (uint8_t)i;
		}

		rc = backplane_read_bytes(base, before, sizeof(before));
		if (rc == 0) {
			rc = backplane_write_bytes(base, pat, sizeof(pat));
		}
		if (rc == 0) {
			rc = backplane_read_bytes(base, after, sizeof(after));
		}
		if (rc != 0) {
			LOG_ERR("burst16 @SOCRAM[0x%02x] aborted: %d", base, rc);
		} else {
			bool match = true;
			for (size_t i = 0; i < sizeof(pat); i++) {
				if (after[i] != pat[i]) {
					match = false;
					break;
				}
			}
			LOG_INF("burst16 @SOCRAM[0x%02x]: %s",
				base, match ? "OK" : "MISMATCH");
			LOG_HEXDUMP_INF(before, sizeof(before), "before");
			LOG_HEXDUMP_INF(pat,    sizeof(pat),    "wrote ");
			LOG_HEXDUMP_INF(after,  sizeof(after),  "after ");
		}
	}

	LOG_INF("--- Spike A complete ---");

	/* ---- Spike B: block-mode CMD53 smoke test ----
	 * 128-byte transfer (= 2 blocks of 64) at a fresh SOCRAM address.
	 * Zephyr's sdio_io_rw_extended_helper switches to block-mode CMD53
	 * when len > block_size, so this exercises a single CMD53 with
	 * blocks=2. Confirms the SDHCI driver handles multi-block PIO
	 * correctly before we attempt the ~432 KiB firmware upload.
	 */
	LOG_INF("--- Spike B: block-mode CMD53 (128B) ---");
	{
		const uint32_t base = 0x00000200u;
		uint8_t pat[128];
		uint8_t before[128] = {0};
		uint8_t after[128]  = {0};
		int rc;

		/* Distinct, non-uniform pattern (0x80..0xFF) — won't be
		 * confused with POR garbage on readback.
		 */
		for (size_t i = 0; i < sizeof(pat); i++) {
			pat[i] = (uint8_t)(0x80u + i);
		}

		rc = backplane_read_bytes(base, before, sizeof(before));
		if (rc == 0) {
			rc = backplane_write_bytes(base, pat, sizeof(pat));
		}
		if (rc == 0) {
			rc = backplane_read_bytes(base, after, sizeof(after));
		}
		if (rc != 0) {
			LOG_ERR("Spike B aborted @ SOCRAM[0x%03x]: %d", base, rc);
		} else {
			size_t first_mismatch = sizeof(pat);
			for (size_t i = 0; i < sizeof(pat); i++) {
				if (after[i] != pat[i]) {
					first_mismatch = i;
					break;
				}
			}
			if (first_mismatch == sizeof(pat)) {
				LOG_INF("Spike B: 128B block-mode round-trip OK @ SOCRAM[0x%03x]",
					base);
			} else {
				LOG_ERR("Spike B: MISMATCH at byte %u (expected 0x%02x got 0x%02x)",
					(unsigned)first_mismatch,
					pat[first_mismatch], after[first_mismatch]);
				LOG_HEXDUMP_INF(before, 32, "before[:32]");
				LOG_HEXDUMP_INF(pat,    32, "wrote[:32] ");
				LOG_HEXDUMP_INF(after,  32, "after[:32] ");
			}
		}
	}
	LOG_INF("--- Spike B complete ---");

	/* === Firmware + NVRAM upload =======================================
	 * Bring the chip from "passive" to "ready to boot": write firmware
	 * to chip-address 0 (SOCRAM start), strip + write NVRAM at top of
	 * SOCRAM. ARM CM3 stays halted -- releasing it is the boot trigger
	 * and lives in a separate later step.
	 */
	LOG_INF("--- firmware upload: %u bytes @ chip 0x%08x ---",
		brcmfmac_fw_len, 0u);
	{
		int64_t t0 = k_uptime_get();
		ret = ramrw(true, 0u, (uint8_t *)brcmfmac_fw, brcmfmac_fw_len);
		if (ret != 0) {
			LOG_ERR("firmware upload failed: %d", ret);
			return ret;
		}
		int64_t t1 = k_uptime_get();
		LOG_INF("firmware upload OK in %lld ms (%u bytes)",
			(long long)(t1 - t0), brcmfmac_fw_len);

#if BCM43430_VERIFY_UPLOAD
		ret = verify_memory(0u, brcmfmac_fw, brcmfmac_fw_len);
		int64_t t2 = k_uptime_get();
		if (ret != 0) {
			LOG_ERR("firmware verify failed: %d", ret);
			return ret;
		}
		LOG_INF("firmware verify OK in %lld ms",
			(long long)(t2 - t1));
#endif
	}

	{
		static uint8_t nvram_buf[1024];
		int stripped = nvram_strip(brcmfmac_nvram, brcmfmac_nvram_len,
					   nvram_buf, sizeof(nvram_buf));
		if (stripped < 0) {
			LOG_ERR("nvram_strip failed: %d", stripped);
			return stripped;
		}
		LOG_INF("nvram strip: %u -> %d bytes (incl. 4-byte token footer)",
			brcmfmac_nvram_len, stripped);
		LOG_HEXDUMP_DBG(nvram_buf, (size_t)stripped, "nvram (stripped)");

		const uint32_t ramsize = 0x80000u;  /* SOCRAM = 512 KiB on BCM43430A1 */
		const uint32_t nvram_addr = ramsize - (uint32_t)stripped;

		int64_t t0 = k_uptime_get();
		ret = ramrw(true, nvram_addr, nvram_buf, (uint32_t)stripped);
		if (ret != 0) {
			LOG_ERR("nvram upload failed: %d", ret);
			return ret;
		}
		int64_t t1 = k_uptime_get();
		LOG_INF("nvram upload OK @ chip 0x%08x in %lld ms",
			nvram_addr, (long long)(t1 - t0));

#if BCM43430_VERIFY_UPLOAD
		ret = verify_memory(nvram_addr, nvram_buf, (uint32_t)stripped);
		int64_t t2 = k_uptime_get();
		if (ret != 0) {
			LOG_ERR("nvram verify failed: %d", ret);
			return ret;
		}
		LOG_INF("nvram verify OK in %lld ms",
			(long long)(t2 - t1));
#endif
	}

	LOG_INF("--- ARM release + boot ---");
	ret = set_active();
	if (ret != 0) {
		LOG_ERR("set_active failed: %d", ret);
		return ret;
	}
	LOG_INF("--- ARM release + boot complete ---");

	/* Spike C is a probe: log success/failure but don't abort the rest of
	 * the bring-up so the REPL still comes up.
	 */
	(void)spike_c_first_ioctl();

	LOG_INF("--- subsystem bring-up complete ---");
	return 0;
}

SYS_INIT(bcm43430_bringup, APPLICATION, 99);
