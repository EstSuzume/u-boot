// SPDX-License-Identifier: GPL-2.0+
/*
 * GR-MANGO (Renesas RZ/A2M R7S921058VCBG) board support
 */

#include <command.h>
#include <console.h>
#include <init.h>
#include <bootm.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <vsprintf.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * In the Mbed OS OctaRAM initialization (octaram_init.c),
 * the MDTR (Memory Delay Trim Register) is set to OCTA.MDTR.BIT.DV1DEL = 40,
 * but when actually running at 132 MHz with that value, a few error occurs.
 * On the board I have, the range where no errors occurred was from 49 to 151.
 * Since it is unclear whether this is due to individual variations in the board,
 * the calibration is performed every time the board starts up.
 */
#define GR_MANGO_OCTARAM_CALIBRATION	1

#define CPG_SCLKSEL		0xFCFE0104	/* 16 bit */
#define  SCLKSEL_OCTCR		GENMASK(9, 8)
#define  SCLKSEL_OCTCR_G	(3 << 8)	/* OM_SCLK = G/2 = 132 MHz */
#define GPIO_PHMOM0		0xFCFFE980
#define  PHMOM0_HOSEL		BIT(0)		/* OctaBus (not HyperBus) */

/* Octa memory controller */
#define OCTA_BASE		0x1F401000
#define OCTA_DCR		(OCTA_BASE + 0x00)
#define OCTA_DAR		(OCTA_BASE + 0x04)
#define OCTA_DCSR		(OCTA_BASE + 0x08)
#define  DCSR_DALEN(n)		(n)		/* data length */
#define  DCSR_DMLEN(n)		((n) << 8)	/* dummy cycles */
#define  DCSR_ACDV		BIT(19)		/* device 1 */
#define  DCSR_CMDLEN(n)		((n) << 20)
#define  DCSR_DAOR		BIT(23)		/* byte order 1, 0, 3, 2 */
#define  DCSR_ADLEN(n)		((n) << 24)
#define  DCSR_ACDA		BIT(28)		/* data access (RAM address conv.) */
#define OCTA_DSR0		(OCTA_BASE + 0x0C)
#define OCTA_DSR1		(OCTA_BASE + 0x10)
#define  DSR_TYP_RAM		(1 << 30)
#define OCTA_MDTR		(OCTA_BASE + 0x14)
#define  MDTR_DV0DEL		GENMASK(7, 0)
#define  MDTR_DQSERAM		GENMASK(11, 8)
#define  MDTR_DV1DEL		GENMASK(23, 16)
#define  MDTR_DQSEDOPI		GENMASK(27, 24)
#define OCTA_DRCSTR		(OCTA_BASE + 0x34)
#define OCTA_DWCSTR		(OCTA_BASE + 0x38)
#define OCTA_DCSTR		(OCTA_BASE + 0x3C)
#define OCTA_CDSR		(OCTA_BASE + 0x40)
#define  CDSR_DV0TTYP		GENMASK(1, 0)
#define  CDSR_DV0TTYP_SPI	(0 << 0)
#define  CDSR_DV0TTYP_DOPI	(2 << 0)
#define  CDSR_DV1TTYP		GENMASK(3, 2)
#define  CDSR_DV1TTYP_DOPI	(2 << 2)
#define  CDSR_DLFT		BIT(31)		/* 1: deadlock timer disabled */
#define OCTA_MDLR		(OCTA_BASE + 0x44)
#define  MDLR_DV0RDL		GENMASK(7, 0)
#define  MDLR_DV1RDL		GENMASK(23, 16)
#define  MDLR_DV1WDL		GENMASK(31, 24)
#define OCTA_MRWCR0		(OCTA_BASE + 0x48)
#define OCTA_MRWCR1		(OCTA_BASE + 0x4C)
#define OCTA_MRWCSR		(OCTA_BASE + 0x50)
#define OCTA_CWNDR		(OCTA_BASE + 0x58)
#define OCTA_CWDR		(OCTA_BASE + 0x5C)
#define OCTA_CRR		(OCTA_BASE + 0x60)

#define OCTAFLASH_SIZE		0x01000000	/* Macronix MX25UW12845G */
#define OCTARAM_BASE		0x60000000	/* AP Memory APS12808L */
#define OCTARAM_SIZE		CONFIG_GRMANGO_OCTARAM_SIZE
#define OCTARAM_LATENCY		5		/* MR latency code 2: LC = 5 */

static bool octaflash_dopi;

#define OCTA_CMD_DELAY_US	10

static void octa_cmd(u32 dcr, u32 dar, u32 dcsr, u32 data)
{
	writel(dcr, OCTA_DCR);
	writel(dar, OCTA_DAR);
	writel(dcsr, OCTA_DCSR);
	if (dcsr & 0xff)
		writel(data, OCTA_CWDR);
	else
		writel(0, OCTA_CWNDR);
	udelay(OCTA_CMD_DELAY_US);
}

static void octaram_init(void)
{
	writel(DSR_TYP_RAM | OCTARAM_SIZE, OCTA_DSR1);
	clrsetbits_le32(OCTA_CDSR, CDSR_DV1TTYP, CDSR_DV1TTYP_DOPI);
	clrsetbits_le32(OCTA_MDLR, MDLR_DV1WDL | MDLR_DV1RDL,
			(OCTARAM_LATENCY << 24) | (OCTARAM_LATENCY << 16));
	clrsetbits_le32(OCTA_MDTR, MDTR_DV1DEL | MDTR_DQSERAM,
			(0x28 << 16) | (6 << 8));
	/* CS# timing: DVRDLO1 = 0, DVRDHI1 = 5, DVRDCMD1 = 0 */
	clrsetbits_le32(OCTA_DRCSTR, GENMASK(31, 24), 5 << 27);
	/* DVWLO1 = 0, DVWHI1 = 1, DVWCMD1 = 0 */
	clrsetbits_le32(OCTA_DWCSTR, GENMASK(31, 24), 1 << 27);

	clrsetbits_le32(OCTA_MRWCR1, GENMASK(31, 24) | GENMASK(15, 8),
			(0x20 << 24) | (0xA0 << 8));
	/* 2-byte commands, 4-byte addresses, byte order 1, 0, 3, 2 */
	clrsetbits_le32(OCTA_MRWCSR, GENMASK(31, 16), 0x5454 << 16);
	setbits_le32(GPIO_PHMOM0, PHMOM0_HOSEL);

	/*
	 * Mode register: normal operation, 100 ohm drive, LC = 5 (up to
	 * 133 MHz), variable latency, wrapped bursts of 64 bytes.
	 */
	octa_cmd(0x4000, 0x00040000, DCSR_ADLEN(4) | DCSR_DAOR |
		 DCSR_CMDLEN(2) | DCSR_ACDV | DCSR_DALEN(2), 0x8021);
}

#if GR_MANGO_OCTARAM_CALIBRATION

#define OCTARAM_PATTERN_SEED	0x2545f491

static u32 octaram_xorshift(u32 x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

static void octaram_fill(volatile u32 *mem, int words)
{
	u32 x = OCTARAM_PATTERN_SEED;
	int i;

	for (i = 0; i < words; i++) {
		x = octaram_xorshift(x);
		mem[i] = x;
	}
}

static ulong octaram_check(const volatile u32 *mem, int words, int passes)
{
	ulong errors = 0;
	u32 x;
	int i;

	while (passes-- > 0) {
		x = OCTARAM_PATTERN_SEED;
		for (i = 0; i < words; i++) {
			x = octaram_xorshift(x);
			if (mem[i] != x)
				errors++;
		}
	}
	return errors;
}

#define OCTARAM_BOOTCAL_BASE	(OCTARAM_BASE + OCTARAM_SIZE - SZ_4K)
#define OCTARAM_BOOTCAL_WORDS	(SZ_4K / 4)
#define OCTARAM_BOOTCAL_PASSES	16
#define OCTARAM_BOOTCAL_STEP	4

static int octaram_delay = -1, octaram_win_lo, octaram_win_hi;

static void octaram_calibrate(void)
{
	volatile u32 *mem = (volatile u32 *)OCTARAM_BOOTCAL_BASE;
	const u32 mdtr = readl(OCTA_MDTR);
	int del, lo = -1, best_lo = -1, best_len = 0;

	octaram_fill(mem, OCTARAM_BOOTCAL_WORDS);
	for (del = 0; del <= 0xff; del += OCTARAM_BOOTCAL_STEP) {
		writel((mdtr & ~MDTR_DV1DEL) | (del << 16), OCTA_MDTR);
		if (octaram_check(mem, OCTARAM_BOOTCAL_WORDS,
				  OCTARAM_BOOTCAL_PASSES) > 0) {
			lo = -1;
			continue;
		}
		if (lo < 0)
			lo = del;
		if (del - lo + OCTARAM_BOOTCAL_STEP > best_len) {
			best_len = del - lo + OCTARAM_BOOTCAL_STEP;
			best_lo = lo;
		}
	}

	if (best_len == 0) {
		/* Nothing worked: keep the boot loader's value */
		writel(mdtr, OCTA_MDTR);
		return;
	}
	octaram_win_lo = best_lo;
	octaram_win_hi = best_lo + best_len - OCTARAM_BOOTCAL_STEP;
	octaram_delay = (octaram_win_lo + octaram_win_hi) / 2;
	writel((mdtr & ~MDTR_DV1DEL) | (octaram_delay << 16), OCTA_MDTR);
}
#endif /* GR_MANGO_OCTARAM_CALIBRATION */

static void octaflash_spi_wren(void)
{
	octa_cmd(0x06, 0, DCSR_CMDLEN(1), 0);
}

static void octaflash_spi_wrcr2(u32 addr, u32 data)
{
	octa_cmd(0x72, addr, DCSR_ADLEN(4) | DCSR_CMDLEN(1) | DCSR_DALEN(1),
		 data);
}

static void octaflash_set_dopi(void)
{
	udelay(OCTA_CMD_DELAY_US);
	clrsetbits_le16(CPG_SCLKSEL, SCLKSEL_OCTCR, SCLKSEL_OCTCR_G);
	readw(CPG_SCLKSEL);
	udelay(OCTA_CMD_DELAY_US);
	setbits_le32(GPIO_PHMOM0, PHMOM0_HOSEL);
	writel(OCTAFLASH_SIZE, OCTA_DSR0);

	/* Configuration register 2: 14 dummy cycles, then DTR OPI */
	octaflash_spi_wren();
	octaflash_spi_wrcr2(0x300, 0x03);
	octaflash_spi_wren();
	octaflash_spi_wrcr2(0x000, 0x02);
	octaflash_dopi = true;

	clrsetbits_le32(OCTA_CDSR, CDSR_DLFT | CDSR_DV0TTYP,
			CDSR_DLFT | CDSR_DV0TTYP_DOPI);
	/* Memory map: read EEh 11h (8DTRD), write 12h EDh */
	writel(0x12EDEE11, OCTA_MRWCR0);
	clrsetbits_le32(OCTA_MRWCSR, GENMASK(15, 0), 0x5454);
	clrsetbits_le32(OCTA_MDTR, MDTR_DQSEDOPI | MDTR_DV0DEL,
			(6 << 24) | 0x48);
	clrsetbits_le32(OCTA_MDLR, MDLR_DV0RDL, 14);
	/* CS# timing: DVSELLO = 0, DVSELHI = 5, DVSELCMD = 0 */
	clrsetbits_le32(OCTA_DCSTR, GENMASK(15, 8), 5 << 11);
	/* DVWLO0 = 0, DVWHI0 = 2, DVWCMD0 = 1 */
	clrsetbits_le32(OCTA_DWCSTR, GENMASK(15, 8), (2 << 11) | (1 << 8));
	/* DVRDLO0 = 0, DVRDHI0 = 5, DVRDCMD0 = 1, CTR0 = 0, CTRW0 = 0x20 */
	clrsetbits_le32(OCTA_DRCSTR, GENMASK(15, 0),
			(5 << 11) | (1 << 8) | 0x20);
}

/*
 * Put the flash back into SPI mode and device 0 back to the boot ROM settings.
 * The flash is unreadable through the memory map afterwards.
 */
static void octaflash_set_spi(void)
{
	if (!octaflash_dopi)
		return;

	octa_cmd(0x06F9, 0, DCSR_CMDLEN(2), 0);
	octa_cmd(0x728D, 0, DCSR_ADLEN(4) | DCSR_CMDLEN(2) | DCSR_DALEN(2),
		 0x0000);
	octaflash_dopi = false;

	clrsetbits_le32(OCTA_CDSR, CDSR_DV0TTYP, CDSR_DV0TTYP_SPI);
	writel(0x00000003, OCTA_MRWCR0);
	clrsetbits_le32(OCTA_MRWCSR, GENMASK(15, 0), 0x000B);
	clrsetbits_le32(OCTA_MDLR, MDLR_DV0RDL, 0);
}

void rza2_reset_prepare(void)
{
	octaflash_set_spi();
}

void board_quiesce_devices(void)
{
	octaflash_set_spi();
}

/*
 * Do not print from board_init(): until serial_initialize() puts() still
 * calls the serial device bound before relocation, whose driver runs from
 * the flash, and U-Boot maps everything but the DRAM banks execute-never
 * once the MMU is on. The resulting prefetch abort recurses through the
 * abort message and U-Boot hangs silently. The debug UART is safe.
 */
#define early_printf(...)					\
	do {							\
		if (IS_ENABLED(CONFIG_DEBUG_UART))		\
			printf(__VA_ARGS__);			\
	} while (0)

int board_init(void)
{
	gd->bd->bi_boot_params = CFG_SYS_SDRAM_BASE + 0x100;

	early_printf("OCTA:  OctaRAM setup\n");
	octaram_init();
	early_printf("OCTA:  OctaFlash to DOPI\n");
	octaflash_set_dopi();
#if GR_MANGO_OCTARAM_CALIBRATION
	early_printf("OCTA:  OctaRAM read calibration\n");
	octaram_calibrate();
#endif /* GR_MANGO_OCTARAM_CALIBRATION */
	early_printf("OCTA:  done\n");

	gd->dram[1].start = OCTARAM_BASE;
	gd->dram[1].size = OCTARAM_SIZE;

	return 0;
}

int board_late_init(void)
{
	printf("OCTA:  OctaFlash DOPI 132 MHz, OctaRAM %lu MiB at 0x%08x\n",
	       (ulong)OCTARAM_SIZE >> 20, OCTARAM_BASE);
#if GR_MANGO_OCTARAM_CALIBRATION
	if (octaram_delay >= 0)
		printf("OCTA:  OctaRAM read delay %d (no errors from %d to %d)\n",
		       octaram_delay, octaram_win_lo, octaram_win_hi);
	else
		printf("OCTA:  OctaRAM read calibration FAILED, delay %lu\n",
		       (ulong)((readl(OCTA_MDTR) & MDTR_DV1DEL) >> 16));
#endif /* GR_MANGO_OCTARAM_CALIBRATION */
	return 0;
}

#if IS_ENABLED(CONFIG_DEBUG_UART_BOARD_INIT)
#define PFC_BASE		0xFCFFE000
#define PFC_PDR(p)		(PFC_BASE + 0x000 + (p) * 2)	/* 16 bit */
#define PFC_PMR(p)		(PFC_BASE + 0x080 + (p))	/* 8 bit */
#define PFC_PFS(p, n)		(PFC_BASE + 0x200 + (p) * 8 + (n))
#define PFC_PWPR		(PFC_BASE + 0x2FF)
#define  PWPR_B0WI		BIT(7)
#define  PWPR_PFSWE		BIT(6)
#define SCIF4_BASE		0xE8009000
#define  SCIF_SCBRR		0x02

static void grmango_set_pin_function(int port, int pin, int func)
{
	clrbits_le16(PFC_PDR(port), 3 << (pin * 2));
	clrbits_8(PFC_PMR(port), BIT(pin));
	writeb(0, PFC_PWPR);
	writeb(PWPR_PFSWE, PFC_PWPR);
	writeb(func, PFC_PFS(port, pin));
	writeb(0, PFC_PWPR);
	writeb(PWPR_B0WI, PFC_PWPR);
	setbits_8(PFC_PMR(port), BIT(pin));
}

void board_debug_uart_init(void)
{
	grmango_set_pin_function(9, 0, 4);	/* TxD4 */
	grmango_set_pin_function(9, 1, 4);	/* RxD4 */
	/* 115200 bps from P1 = 66 MHz: 66000000 / (32 * 115200) - 1 */
	writeb(17, SCIF4_BASE + SCIF_SCBRR);
}
#endif

#if IS_ENABLED(CONFIG_DEBUG_UART)
int reserve_arch(void)
{
	printf("\nram_top %08lx relocaddr %08lx sp %08lx mon_len %08lx\n",
	       gd->ram_top, gd->relocaddr, gd->start_addr_sp, (ulong)gd->mon_len);
	return 0;
}
#endif

int dram_init(void)
{
	gd->ram_base = CFG_SYS_SDRAM_BASE;
	gd->ram_size = CFG_SYS_SDRAM_SIZE;
	return 0;
}

int dram_init_banksize(void)
{
	/* on-chip RAM only */
	gd->dram[0].start = gd->ram_base;
	gd->dram[0].size = gd->ram_size;
	return 0;
}
