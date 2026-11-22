// SPDX-License-Identifier: GPL-2.0+
/*
 * RZ/A2 pin controller support
 *
 * Based on R7S72100 pinctrl, Copyright (C) 2019 Marek Vasut
 * Based on RZA2MEVB board code, Copyright (C) 2018 Chris Brandt
 *
 * Copyright (C) 2018 Renesas Electronics Corporation
 */

#include <dm.h>
#include <asm/global_data.h>
#include <dm/lists.h>
#include <dm/pinctrl.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/err.h>

#define PDR_BASE        0x0000     /* 16-bit, 2 bytes apart */
#define PODR_BASE       0x0040     /* 8-bit, 1 byte apart */
#define PIDR_BASE       0x0060     /* 8-bit, 1 byte apart */
#define PMR_BASE        0x0080     /* 8-bit, 1 byte apart */
#define DSCR_BASE       0x0140     /* 16-bit, 2 bytes apart */
#define PFS_BASE        0x0200     /* 8-bit, 8 bytes apart */
#define PWPR            0x02FF     /* 8-bit */
#define PFENET          0x0820     /* 8-bit */
#define PPOC            0x0900     /* 32-bit */
#define PHMOMO          0x0980     /* 32-bit */
#define PMODEPFS        0x09C0     /* 32-bit */
#define PCKIO           0x09D0     /* 8-bit */

#define RZA2_PINS_PER_PORT 8

DECLARE_GLOBAL_DATA_PTR;

struct r7s9210_pfc_plat {
	void __iomem	*base;
};

static void r7s9210_pfc_set_function(struct udevice *dev, u8 port, u8 pin, u8 func)
{
	struct r7s9210_pfc_plat *plat = dev_get_plat(dev);
	u16 reg16, mask16;

	/* Set pin to 'Non-use (Hi-z input protection)' */
	reg16 = readw(plat->base + PDR_BASE + (port * 2));
	mask16 = 0x03 << (pin * 2);
	reg16 &= ~mask16;
	writew(reg16, plat->base + PDR_BASE + (port * 2));

	/* Temporary switch to GPIO */
	clrbits_8(plat->base + PMR_BASE + port, BIT(pin));

	/* PFS Register Write Protect : OFF */
	writeb(0x00, plat->base + PWPR); /* B0WI=0, PFSWE=0 */
	writeb(0x40, plat->base + PWPR); /* B0WI=0, PFSWE=1 */

	/* Set Pin function (interrupt disabled, ISEL=0) */
	writeb(func, plat->base + PFS_BASE + (port * 8) + pin);

	/* PFS Register Write Protect : ON */
	writeb(0x00, plat->base + PWPR); /* B0WI=0, PFSWE=0 */
	writeb(0x80, plat->base + PWPR); /* B0WI=1, PFSWE=0 */

	/* Port Mode : Peripheral module pin functions */
	setbits_8(plat->base + PMR_BASE + port, BIT(pin));
}

static int r7s9210_pfc_set_state(struct udevice *dev, struct udevice *config)
{
	const void *blob = gd->fdt_blob;
	int node = dev_of_offset(config);
	u32 cells[32];
	u16 bank, line, func;
	int i, count;

	count = fdtdec_get_int_array_count(blob, node, "pinmux",
					   cells, ARRAY_SIZE(cells));
	if (count < 0) {
		printf("%s: bad pinmux array %d\n", __func__, count);
		return -EINVAL;
	}

	if (count > ARRAY_SIZE(cells)) {
		printf("%s: unsupported pinmux array count %d\n",
		       __func__, count);
		return -EINVAL;
	}

	for (i = 0 ; i < count; i++) {
		func = (cells[i] >> 16) & 0xf;
		if (func == 0 || func > 8) {
			printf("Invalid cell %d in node %s!\n",
			       i, ofnode_get_name(dev_ofnode(config)));
			continue;
		}

		bank = (cells[i] / RZA2_PINS_PER_PORT) & 0xff;
		line = cells[i] % RZA2_PINS_PER_PORT;

		r7s9210_pfc_set_function(dev, bank, line, func);
	}

	return 0;
}

const struct pinctrl_ops r7s9210_pfc_ops  = {
	.set_state = r7s9210_pfc_set_state,
};

static int r7s9210_pfc_probe(struct udevice *dev)
{
	struct r7s9210_pfc_plat *plat = dev_get_plat(dev);
	fdt_addr_t addr_base;

	addr_base = dev_read_addr(dev);
	if (addr_base == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->base = (void __iomem *)addr_base;
	return 0;
}

static const struct udevice_id r7s9210_pfc_match[] = {
	{ .compatible = "renesas,r7s9210-pinctrl" },
	{}
};

U_BOOT_DRIVER(r7s9210_pfc) = {
	.name		= "r7s9210_pfc",
	.id		= UCLASS_PINCTRL,
	.of_match	= r7s9210_pfc_match,
	.probe		= r7s9210_pfc_probe,
	.plat_auto	= sizeof(struct r7s9210_pfc_plat),
	.ops		= &r7s9210_pfc_ops,
};
