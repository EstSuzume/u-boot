/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration settings for the GR-MANGO board (Renesas RZ/A2M)
 */

#ifndef __GRMANGO_H
#define __GRMANGO_H

/* On-chip RAM; the OctaRAM is added as a second bank by board_init() */
#define CFG_SYS_SDRAM_BASE		0x80000000
#define CFG_SYS_SDRAM_SIZE		(4 * 1024 * 1024)

#endif	/* __GRMANGO_H */
