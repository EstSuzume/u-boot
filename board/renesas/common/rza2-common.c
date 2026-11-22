// SPDX-License-Identifier: GPL-2.0+
/*
 * RZ/A2 common code: reset through the watchdog timer
 *
 * Copyright (C) 2017 Renesas Electronics
 * Copyright (C) Chris Brandt
 */

#include <cpu_func.h>
#include <init.h>
#include <asm/io.h>

#define RZA2_WDT_BASE	0xFCFE7000
#define WTCSR		(RZA2_WDT_BASE + 0x00) /* Watchdog Timer Control Register */
#define WTCNT		(RZA2_WDT_BASE + 0x02) /* Watchdog Timer Counter Register */
#define WRCSR		(RZA2_WDT_BASE + 0x04) /* Watchdog Reset Control Register */

void __weak reset_cpu(void)
{
	/* Dummy read (must read WRCSR:WOVF at least once before clearing) */
	writeb(readb(WRCSR), WRCSR);

	writew(0xa500, WRCSR); /* Clear WOVF */
	writew(0x5a5f, WRCSR); /* Reset Enable */
	writew(0x5a00, WTCNT); /* Counter to 00 */
	writew(0xa578, WTCSR); /* Start timer */

	/* Wait for WDT overflow */
	for (;;)
		asm volatile("wfi");
}
