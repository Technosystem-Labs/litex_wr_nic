/*
 * This file is part of LiteX-WR-NIC.
 *
 * Copyright (c) 2025 Warsaw University of Technology
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Kasli v2.0 board support for WRPC firmware.
 *
 * This file is an overlay for wrpc-sw boards/generic/board.c.
 * It extends the generic board init to configure the PCA9548 I2C muxes
 * that gate the SFP EEPROM I2C bus on Kasli v2.0.
 *
 * The Kasli v2.0 board I2C bus (sfp_i2c, pins J16/M17) has two cascaded
 * PCA9548 8-channel I2C switches:
 *
 *   0x70 — top-level switch; all channels must be DISABLED (write 0x00)
 *           before accessing SFP0 to avoid contention with other peripherals
 *           (EEM modules, etc.) that share the same bus.
 *
 *   0x71 — SFP switch; bit N selects SFP channel N.
 *           Channel 0 (write 0x01) routes to SFP0 EEPROM (address 0x50).
 *
 * The muxes are configured once at early board init and left permanently
 * with 0x71 channel 0 selected.  This is sufficient for a single-SFP WR
 * node that always uses SFP0.
 */

#include "board.h"
#include "wrc-debug.h"
#include "dev/w1.h"
#include "dev/syscon.h"
#include "dev/endpoint.h"
#include "dev/bb_i2c.h"
#include "storage.h"
#include "board-decl.h"

/* I2C addresses of the PCA9548 switches on the Kasli v2.0 board I2C bus */
#define KASLI_I2C_MUX_TOP_ADDR 0x70  /* Top-level mux: disable all channels */
#define KASLI_I2C_MUX_SFP_ADDR 0x71  /* SFP mux: bit0 = SFP0 channel        */

/*
 * kasli_i2c_mux_write - write one byte to a PCA9548 I2C switch
 *
 * The PCA9548 control register is accessed with a bare write (no register
 * address byte): START | (addr<<1) | data | STOP.
 * Returns 0 on success, -1 if the device does not ACK.
 */
static int kasli_i2c_mux_write(uint8_t addr, uint8_t value)
{
	int ack;

	bb_i2c_init(&dev_i2c_sfp1);
	bb_i2c_start(&dev_i2c_sfp1);
	ack  = bb_i2c_put_byte(&dev_i2c_sfp1, addr << 1); /* addr + write */
	ack |= bb_i2c_put_byte(&dev_i2c_sfp1, value);
	bb_i2c_stop(&dev_i2c_sfp1);

	return ack ? 0 : -1;
}

int wrc_board_early_init(void)
{
	generic_board_storage_init();

	/*
	 * Configure PCA9548 I2C muxes for SFP0 access:
	 *   1. Disable all channels on 0x70 (prevents bus contention).
	 *   2. Enable channel 0 on 0x71 (routes to SFP0 EEPROM).
	 * Leave both muxes in this state permanently.
	 */
	if (kasli_i2c_mux_write(KASLI_I2C_MUX_TOP_ADDR, 0x00) < 0)
		board_dbg("Kasli: failed to configure I2C mux 0x%02x\n",
			  KASLI_I2C_MUX_TOP_ADDR);

	if (kasli_i2c_mux_write(KASLI_I2C_MUX_SFP_ADDR, 0x01) < 0)
		board_dbg("Kasli: failed to configure I2C mux 0x%02x\n",
			  KASLI_I2C_MUX_SFP_ADDR);

	return 0;
}

static int board_get_persistent_mac(uint8_t *mac)
{
	int i;
	struct w1_dev *d;

	/* Try from SDB */
	if (storage_get_persistent_mac(0, mac) == 0)
		return 0;

	/* Get from one-wire (derived from unique id) */
	if (HAS_W1) {
		for (i = 0; i < W1_MAX_DEVICES; i++) {
			d = wrpc_w1_bus.devs + i;
			if (d->rom) {
				mac[0] = 0x22;
				mac[1] = 0x33;
				mac[2] = 0xff & (d->rom >> 32);
				mac[3] = 0xff & (d->rom >> 24);
				mac[4] = 0xff & (d->rom >> 16);
				mac[5] = 0xff & (d->rom >> 8);
				return 0;
			}
		}
	}

	/* Not found */
	return -1;
}

int wrc_board_init(void)
{
	uint8_t mac_addr[6];

	if (board_get_persistent_mac(mac_addr) < 0) {
		board_dbg("Failed to get MAC address from the flash. "
			  "Using fallback address.\n");
		mac_addr[0] = 0x22;
		mac_addr[1] = 0x33;
		mac_addr[2] = 0x44;
		mac_addr[3] = 0x55;
		mac_addr[4] = 0x66;
		mac_addr[5] = 0x77;
	}
	ep_set_mac_addr(&wrc_endpoint_dev, mac_addr);
	ep_pfilter_init_default(&wrc_endpoint_dev);

	return 0;
}
