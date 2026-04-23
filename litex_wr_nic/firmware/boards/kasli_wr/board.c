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

	return ack == 0 ? 0 : -1;
}

static int kasli_i2c_write(uint8_t addr, uint8_t reg, uint8_t val)
{
	int ack;

	bb_i2c_init(&dev_i2c_sfp1);
	bb_i2c_start(&dev_i2c_sfp1);
	ack  = bb_i2c_put_byte(&dev_i2c_sfp1, addr << 1); /* dev addr + write */
	ack |= bb_i2c_put_byte(&dev_i2c_sfp1, reg);
	ack |= bb_i2c_put_byte(&dev_i2c_sfp1, val);
	bb_i2c_stop(&dev_i2c_sfp1);

	return ack == 0 ? 0 : -1;
}

static int enable_i2c_switch_port(uint8_t switch_adr, uint8_t port_no)
{
	if (port_no > 7) {
		board_dbg("Kasli: port_no must be in smaller than 8, not: %d\n", port_no);
		return -1;
	}

	return kasli_i2c_mux_write(switch_adr, 1 << port_no);
}

static int release_i2c_switch(uint8_t switch_adr)
{
	return kasli_i2c_mux_write(switch_adr,  0x00);
}

static int setup_i2c_switches(void)
{
	/*
	 * Configure PCA9548 I2C muxes for SFP0 access:
	 *   1. Disable all channels on 0x70 (prevents bus contention).
	 *   2. Enable channel 0 on 0x71 (routes to SFP0 EEPROM).
	 * Leave both muxes in this state permanently.
	 */
	int ret = kasli_i2c_mux_write(KASLI_I2C_MUX_TOP_ADDR, 0x00);
	if ( ret < 0 ) 
		board_dbg("Kasli: failed to configure I2C mux 0x%02x; return code %d\n",
			  KASLI_I2C_MUX_TOP_ADDR, ret);

	ret = kasli_i2c_mux_write(KASLI_I2C_MUX_SFP_ADDR, 0x01);
	if ( ret < 0 )
		board_dbg("Kasli: failed to configure I2C mux 0x%02x; return code %d\n",
			  KASLI_I2C_MUX_SFP_ADDR, ret);

	return 0;
}


static int detect_expander(void)
{
	// PCA9539 (IC28/IC29)  -> 0x75/0x74
	// MCP23017 (IC24/IC25) -> 0x21/0x20

	/* 
	* NOTE: silent assumption that if one GPIO expander is reachable, the second
	*	one should be reachable, as well. It's pretty sensible assumption.
	*	At the moment, we are interested only in SFP0, so we actually care only
	* 	about the 0x20/0x74 ICs.
	*/
	
	release_i2c_switch(KASLI_I2C_MUX_SFP_ADDR);
	if(enable_i2c_switch_port(KASLI_I2C_MUX_SFP_ADDR, 3) < 0)
		return -1;

	int variant = -1;
	
	if (bb_i2c_devprobe(&dev_i2c_sfp1, 0x20)){
		variant = 0;	/* MCP23017 variant */
		board_dbg("Kasli: found MCP23017 GPIO expander at 0x20\n");
	} else if (bb_i2c_devprobe(&dev_i2c_sfp1, 0x74)) {
		variant = 1;	/* PCA9539 variant */
		board_dbg("Kasli: found PCA9539 GPIO expander at 0x20\n");
	} else {
		board_dbg("Kasli: no GPIO expander detected at 0x20 or 0x74\n");
	}

	release_i2c_switch(KASLI_I2C_MUX_SFP_ADDR);
	return variant;
}

static int configure_gpio_expander(int variant){
	// Silent assumption that if one GPIO expander was detected, the second one
	// is present and reachable as well. It's pretty sensible assumption.
	
	// We should not need to drive Helper and Main DCXO OE, because they are actively
	// pulled up by resistors, but we need to actively drive CLK_SEL LOW, to use
	// clock signal from main DCXO
	release_i2c_switch(KASLI_I2C_MUX_SFP_ADDR);
	if(enable_i2c_switch_port(KASLI_I2C_MUX_SFP_ADDR, 3) < 0)
		return -1;

	// GPA
	// MSB to LSB:
	// 	- 7: VUSB_PRESENT(1)
	//  - 6: SFP0_LED(0)
	//  - 5: SFP0_LOS(1)
	//  - 4: SFP0_MOD_PRESENT(1)
	// 	- 3: SFP0_RATE_SELECT(0)
	// 	- 2: SFP0_RATE_SELECT1(0)
	//  - 1: SFP0_TXDISABLE(0)	- needs to be actively driven LOW
	// 	- 0: SFP0_TX_FAULT(1)

	// GPB
	// MSB to LSB:
	// 	- 7: CLK_SEL(0)
	//  - 6: SFP1_LED(0)	- might as well drive it
	//  - 5: SFP1_LOS(1)
	//  - 4: SFP1_MOD_PRESENT(1)
	// 	- 3: SFP1_RATE_SELECT(0)
	// 	- 2: SFP1_RATE_SELECT1(0)
	//  - 1: SFP1_TXDISABLE(0)	- migh as well...
	// 	- 0: SFP1_TX_FAULT(1)

	uint8_t port_config_a = 0b10110001;
	uint8_t io_value_a = 0b01000000; // SPF0 LED ON, TXDISABLE LOW

	uint8_t port_config_b = 0b00110001;
	uint8_t io_value_b = 0b11000000;	// CLK_SEL HIGH, SFP1 LED ON, TXDISABLE LOW

	uint8_t port_cfgs[2] = {port_config_a, port_config_b};
	uint8_t io_vals[2] = {io_value_a, io_value_b};

	uint8_t iodir[2] = {0x00, 0x06};	// [base_addr_MCP, base_addr_PCA]
	uint8_t olat[2] = {0x14, 0x02};		// [base_addr_MCP, base_addr_PCA]

	uint8_t ic_addr[2] = {0x20, 0x74};	// [addr_MCP, addr_PCA]

	int ret = -1;
	if (variant < 0 || variant > 1)
	{
		board_dbg("Kasli: supported GPIO assembly variants are only 0 and 1, not: %d\n", variant);
		return ret;
	}

	// 
	for (uint8_t i=0; i<2; i++){
		ret = kasli_i2c_write(ic_addr[variant], iodir[variant] + i, port_cfgs[i]);
		if (ret < 0)
			board_dbg("Kasli: could not write to I/O expander config register\n");
		ret |= kasli_i2c_write(ic_addr[variant], olat[variant] + i, io_vals[i]);
	}
	release_i2c_switch(KASLI_I2C_MUX_SFP_ADDR);
	return ret;
}


int wrc_board_early_init(void)
{
	generic_board_storage_init();
	int gpio_variant = detect_expander();
	if (gpio_variant < 0)
		return 0;	/* no expander */
	configure_gpio_expander(gpio_variant);
	setup_i2c_switches();

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
