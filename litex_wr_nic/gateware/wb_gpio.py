#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 TechnoSystem
# SPDX-License-Identifier: BSD-2-Clause
#
# Minimal wbgen-compatible GPIO over Wishbone.
#
# wrpc-sw ships a generic GPIO driver (`dev/gpio.c`, wb_gpio_create) and an
# I2C bit-bang driver (`dev/bb_i2c.c`) that together drive an I2C bus over a
# GPIO peripheral whose register layout is the CERN wbgen "wb_gpio" block:
#
#     0x0  COR  (W)  write-1 bits CLEAR the matching output bit
#     0x4  SOR  (W)  write-1 bits SET   the matching output bit;  (R) -> output reg
#     0x8  DDR  (R/W) data-direction register (stored; unused by bb_i2c)
#     0xC  PSR  (R)  pin-state register (live input pins)
#
# LiteX has no Wishbone GPIO with this exact layout (its GPIO cores are CSR
# based), so this small module re-implements it so the *firmware* can reuse
# the stock wb_gpio_create + bb_i2c drivers verbatim -- zero custom C transport.
# See docs / the Si549 firmware-init plan for why this hangs off the WR core
# `aux_master` (periph3) slot rather than LiteX CSR space.

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone

# Wishbone GPIO (wbgen layout) ---------------------------------------------------------------------

class WbGpio(LiteXModule):
    """wbgen-compatible GPIO slave on a classic Wishbone bus.

    Attributes:
        bus  : Wishbone slave interface (data_width=32, word-addressed).
        oreg : output register (Signal(nbits)) -- wire to pad drivers.
        ireg : live pin inputs (Signal(nbits)) -- wire from pad inputs.
        ddr  : data-direction register (stored only; bb_i2c never uses it).
    """
    def __init__(self, nbits=32):
        self.bus  = wishbone.Interface(data_width=32, address_width=32, addressing="byte")
        self.oreg = Signal(nbits)
        self.ireg = Signal(nbits)
        self.ddr  = Signal(nbits)

        # # #

        # Byte-addressed bus (matches the WR-core aux_master adr). Register =
        # byte offset / 4 -> adr[3:2]: 0=COR(0x0), 1=SOR(0x4), 2=DDR(0x8), 3=PSR(0xC).
        sel = self.bus.adr[2:4]

        # Classic Wishbone (ack-only handshake; no stall signal on this bus).
        self.sync += [
            self.bus.ack.eq(0),
            If(self.bus.cyc & self.bus.stb & ~self.bus.ack,
                self.bus.ack.eq(1),
                If(self.bus.we,
                    Case(sel, {
                        0b00 : self.oreg.eq(self.oreg & ~self.bus.dat_w),  # COR: clear bits.
                        0b01 : self.oreg.eq(self.oreg |  self.bus.dat_w),  # SOR: set bits.
                        0b10 : self.ddr.eq(self.bus.dat_w),                # DDR: store.
                    }),
                ).Else(
                    Case(sel, {
                        0b01    : self.bus.dat_r.eq(self.oreg),  # SOR read-back.
                        0b10    : self.bus.dat_r.eq(self.ddr),   # DDR.
                        0b11    : self.bus.dat_r.eq(self.ireg),  # PSR: live pins.
                        "default": self.bus.dat_r.eq(0),
                    }),
                ),
            ),
        ]
