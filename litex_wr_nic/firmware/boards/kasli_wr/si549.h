/*
 * This file is part of LiteX-WR-NIC.
 *
 * Copyright (c) 2026 TechnoSystem
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Kasli v2.0 Si549 DCXO initial-frequency setup, run from WRPC firmware.
 */

#ifndef __KASLI_SI549_H
#define __KASLI_SI549_H

/*
 * Program both Si549 DCXOs (Main + Helper) to their target frequencies via
 * the frequency-update sequence, bit-banged over the WR-core aux
 * Wishbone GPIO (BASE_AUXWB). Must run from wrc_board_early_init(), before the
 * SoftPLL / PHY come up. Returns 0 on success, -1 if a chip did not respond.
 */
int kasli_si549_init(void);

#endif /* __KASLI_SI549_H */
