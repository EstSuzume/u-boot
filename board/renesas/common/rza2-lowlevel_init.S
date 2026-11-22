/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 Renesas Electronics Corporation
 * Copyright (C) 2018 Chris Brandt
 */
#include <config.h>
#include <asm/macro.h>

.macro  dummy_read32, addr
	ldr     r4, =\addr
	ldr     r5, [r4]
.endm

.macro  dummy_read8, addr
	ldr     r4, =\addr
	ldrb    r6, [r4]
.endm

/* Busy wait of at least 3 us at up to 528 MHz (2 instructions a turn) */
.macro  wait_3us
	ldr     r4, =2000
1:	subs    r4, r4, #1
	bne     1b
.endm

#define RZA2_WDT_BASE	0xFCFE7000
#define RZA2_STBCR_BASE	0xFCFE0020

/* Watchdog Registers */
#define WTCSR		(RZA2_WDT_BASE + 0x00) /* Watchdog Timer Control Register */
#define WTCNT		(RZA2_WDT_BASE + 0x02) /* Watchdog Timer Counter Register */

/* Disable WDT */
#define WTCSR_D		0xA518
#define WTCNT_D		0x5A00

/* PL310 L2 Cache */
#define PL310_BASE	0x1F003000
#define REG15_PWR_CTRL	0xf80

/* Clock Registers */
#define FRQCR		0xFCFE0010
#define CKIOSEL		0xFCFE0100

#define FRQCR_D		0x1012
#define CKIOSEL_D	0x0000 /* CKIO Output = B Clock  */

/* Standby controller registers */
#define STBCR3		(RZA2_STBCR_BASE + 0x400)
#define STBCR4		(RZA2_STBCR_BASE + 0x404)
#define STBCR5		(RZA2_STBCR_BASE + 0x408)
#define STBCR6		(RZA2_STBCR_BASE + 0x40c)
#define STBCR7		(RZA2_STBCR_BASE + 0x410)
#define STBCR8		(RZA2_STBCR_BASE + 0x414)
#define STBCR9		(RZA2_STBCR_BASE + 0x418)
#define STBCR10		(RZA2_STBCR_BASE + 0x41c)
#define STBCR11		(RZA2_STBCR_BASE + 0x420)

/* Enable all peripheral clocks */
#define STBCR3_D        0x80 /* OSTM0, OSTM1, OSTM3, MTU3, CAN-FD, ADC, GPT */
#define STBCR4_D        0x00 /* SCIF0, SCIF1, SCIF2, SCIF3, SCIF4, SCI0, SCI1,IrDA */
#define STBCR5_D        0x31 /* A/D, CEU, RTC0, RTC1, JCU */
#define STBCR6_D        0x88
#define STBCR7_D        0x10 /* IMR-LS2, DAVE-2D, MIPI, SSIF0, SSIF1, SSIF2, SSIF3 */
#define STBCR8_D        0x05 /* IIC0, IIC1, IIC2, IIC3, SPIBSC, VDC6 */
#define STBCR9_D        0x10 /* RSPI0, RSPI1, RSPI2, HYPER, OCTA, SPDIF, DRP */
#define STBCR10_D       0x00 /* TSIP, SDHI00, SDHI01, SDHI10, SDHI11 */
#define STBCR11_D       0x3F /* POE3, POEG */

#define EPTPC_BYPASS	0xE8204508
#define EPTPC_BYPASS_D	0x00010001 /* BYPASS1, BYPASS0 */

#define SYSCR1		0xFCFE0400 /* large-capacity on-chip RAM access enable */
#define SYSCR2		0xFCFE0404 /* large-capacity on-chip RAM write enable */
#define SYSCR3		0xFCFE0408 /* data-retention RAM write enable */

	.global	lowlevel_init

	.text
	.align	2

lowlevel_init:
	/* PL310 init: Power Control Register */
	write32 (PL310_BASE + REG15_PWR_CTRL), 1   /* Standby mode enable */
	dummy_read32 (PL310_BASE + REG15_PWR_CTRL) /* dummy read */

	/* Set clocks */
	write16 FRQCR, FRQCR_D
	write16 CKIOSEL, CKIOSEL_D

	/* Enable the peripherals (Standby Control) */
	write8 STBCR3, STBCR3_D
	dummy_read8 STBCR3
	write8 STBCR4, STBCR4_D
	dummy_read8 STBCR4
	write8 STBCR5, STBCR5_D
	dummy_read8 STBCR5
	write8 STBCR6, STBCR6_D
	dummy_read8 STBCR6

	/* Bypass the stopped EPTPC (wait 3 us around it) */
	wait_3us
	write32 EPTPC_BYPASS, EPTPC_BYPASS_D
	dummy_read32 EPTPC_BYPASS
	wait_3us

	write8 STBCR7, STBCR7_D
	dummy_read8 STBCR7
	write8 STBCR8, STBCR8_D
	dummy_read8 STBCR8
	write8 STBCR9, STBCR9_D
	dummy_read8 STBCR9
	write8 STBCR10, STBCR10_D
	dummy_read8 STBCR10
	write8 STBCR11, STBCR11_D
	dummy_read8 STBCR11

	b part2

	.pool
	.align 4
part2:
	/* Disable WDT */
	write16 WTCSR, WTCSR_D
	write16 WTCNT, WTCNT_D

	/*
	 * Enable all internal RAM
	 */
	ldr	r4, =SYSCR1
	mov	r5, #0xFF
	strb	r5, [r4]
	ldrb	r6, [r4]
	ldr	r4, =SYSCR2
	strb	r5, [r4]
	ldrb	r6, [r4]
	ldr	r4, =SYSCR3
	mov	r5, #0x0F
	strb	r5, [r4]
	ldrb	r6, [r4]

	nop
	/* back to arch calling code */
	mov	pc, lr

	.align 4
