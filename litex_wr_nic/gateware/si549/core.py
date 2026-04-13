#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2025 Warsaw University of Technology
# SPDX-License-Identifier: BSD-2-Clause
#
# Adapted from ARTIQ (artiq/gateware/wrpll/si549.py)
# Copyright (C) 2019-2024 M-Labs Limited
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# The Si549DAC module bridges the WRPC SoftPLL DAC interface (16-bit word +
# load strobe) to I2C ADPLL writes on the Si549 DCXO. The 16-bit DAC word is
# linearly mapped to the 24-bit Si549 ADPLL register (reg 231):
#   ADPLL = (dac_data - 0x8000) * adpll_scale >> 8
# Mid-scale DAC value (0x8000) maps to ADPLL=0 (nominal frequency).

from migen import *
from migen.genlib.fsm import *

from litex.gen import *
from litex.soc.interconnect.csr import *

# I2C Clock Generator ------------------------------------------------------------------------------

class I2CClockGen(Module):
    def __init__(self, width):
        self.load  = Signal(width)
        self.clk2x = Signal()

        cnt = Signal.like(self.load)
        self.comb += self.clk2x.eq(cnt == 0)
        self.sync += [
            If(self.clk2x,
                cnt.eq(self.load),
            ).Else(
                cnt.eq(cnt - 1),
            )
        ]

# I2C Master Machine -------------------------------------------------------------------------------

class I2CMasterMachine(Module):
    def __init__(self, clock_width):
        self.scl   = Signal(reset=1)
        self.sda_o = Signal(reset=1)
        self.sda_i = Signal()

        self.submodules.cg = CEInserter()(I2CClockGen(clock_width))
        self.start = Signal()
        self.stop  = Signal()
        self.write = Signal()
        self.ack   = Signal()
        self.data  = Signal(8)
        self.ready = Signal()

        # # #

        bits = Signal(4)
        data = Signal(8)

        fsm = CEInserter()(FSM("IDLE"))
        self.submodules += fsm

        fsm.act("IDLE",
            self.ready.eq(1),
            If(self.start,
                NextState("START0"),
            ).Elif(self.stop,
                NextState("STOP0"),
            ).Elif(self.write,
                NextValue(bits, 8),
                NextValue(data, self.data),
                NextState("WRITE0")
            )
        )

        fsm.act("START0",
            NextValue(self.scl, 1),
            NextState("START1")
        )
        fsm.act("START1",
            NextValue(self.sda_o, 0),
            NextState("IDLE")
        )

        fsm.act("STOP0",
            NextValue(self.scl, 0),
            NextState("STOP1")
        )
        fsm.act("STOP1",
            NextValue(self.sda_o, 0),
            NextState("STOP2")
        )
        fsm.act("STOP2",
            NextValue(self.scl, 1),
            NextState("STOP3")
        )
        fsm.act("STOP3",
            NextValue(self.sda_o, 1),
            NextState("IDLE")
        )

        fsm.act("WRITE0",
            NextValue(self.scl, 0),
            NextState("WRITE1")
        )
        fsm.act("WRITE1",
            If(bits == 0,
                NextValue(self.sda_o, 1),
                NextState("READACK0"),
            ).Else(
                NextValue(self.sda_o, data[7]),
                NextState("WRITE2"),
            )
        )
        fsm.act("WRITE2",
            NextValue(self.scl, 1),
            NextValue(data[1:], data[:-1]),
            NextValue(bits, bits - 1),
            NextState("WRITE0"),
        )
        fsm.act("READACK0",
            NextValue(self.scl, 1),
            NextState("READACK1"),
        )
        fsm.act("READACK1",
            NextValue(self.ack, ~self.sda_i),
            NextState("IDLE")
        )

        run  = Signal()
        idle = Signal()
        self.comb += [
            run.eq((self.start | self.stop | self.write) & self.ready),
            idle.eq(~run & fsm.ongoing("IDLE")),
            self.cg.ce.eq(~idle),
            fsm.ce.eq(run | self.cg.clk2x),
        ]

# ADPLL Programmer ---------------------------------------------------------------------------------
#
# Writes a 24-bit ADPLL value to Si549 register 231 via I2C.
# Transaction: START | ADDR<<1 | W | REG(231) | DATA[7:0] | DATA[15:8] | DATA[23:16] | STOP

class ADPLLProgrammer(Module):
    def __init__(self):
        self.i2c_divider = Signal(16)
        self.i2c_address = Signal(7)

        self.adpll = Signal(24)
        self.stb   = Signal()
        self.busy  = Signal()
        self.nack  = Signal()

        self.scl   = Signal()
        self.sda_i = Signal()
        self.sda_o = Signal()

        # # #

        master = I2CMasterMachine(16)
        self.submodules += master

        self.comb += [
            master.cg.load.eq(self.i2c_divider),
            self.scl.eq(master.scl),
            master.sda_i.eq(self.sda_i),
            self.sda_o.eq(master.sda_o),
        ]

        fsm = FSM()
        self.submodules += fsm

        fsm.act("IDLE",
            If(self.stb,
                NextValue(self.nack, 0),
                NextState("START")
            )
        )
        fsm.act("START",
            master.start.eq(1),
            If(master.ready, NextState("DEVADDRESS"))
        )
        fsm.act("DEVADDRESS",
            master.data.eq(self.i2c_address << 1),
            master.write.eq(1),
            If(master.ready, NextState("REGADDRESS"))
        )
        fsm.act("REGADDRESS",
            master.data.eq(231),  # Si549 ADPLL register
            master.write.eq(1),
            If(master.ready,
                If(master.ack,
                    NextState("DATA0")
                ).Else(
                    NextValue(self.nack, 1),
                    NextState("STOP")
                )
            )
        )
        fsm.act("DATA0",
            master.data.eq(self.adpll[0:8]),
            master.write.eq(1),
            If(master.ready,
                If(master.ack,
                    NextState("DATA1")
                ).Else(
                    NextValue(self.nack, 1),
                    NextState("STOP")
                )
            )
        )
        fsm.act("DATA1",
            master.data.eq(self.adpll[8:16]),
            master.write.eq(1),
            If(master.ready,
                If(master.ack,
                    NextState("DATA2")
                ).Else(
                    NextValue(self.nack, 1),
                    NextState("STOP")
                )
            )
        )
        fsm.act("DATA2",
            master.data.eq(self.adpll[16:24]),
            master.write.eq(1),
            If(master.ready,
                If(~master.ack, NextValue(self.nack, 1)),
                NextState("STOP")
            )
        )
        fsm.act("STOP",
            master.stop.eq(1),
            If(master.ready,
                NextState("IDLE")
            )
        )

        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

# Si549DAC -----------------------------------------------------------------------------------------
#
# Bridges the WRPC SoftPLL DAC interface (16-bit word + load strobe) to I2C
# ADPLL writes on the Si549 DCXO.
#
# DAC-to-ADPLL mapping (signed, centered at mid-scale):
#   offset = dac_data - 0x8000          (signed 16-bit, range [-32768, 32767])
#   adpll  = (offset * adpll_scale) >> 8 (truncated to 24-bit signed)
#
# Default adpll_scale=256 gives a 1:1 mapping (1 DAC LSB = 1 ADPLL LSB).
# Increase to widen tuning range at the cost of resolution.
#
# When a new load strobe arrives while I2C is busy, the new value is latched
# and sent as soon as the current transaction completes (no queue, last wins).

class Si549DAC(LiteXModule):
    def __init__(self, pads, load, value, sys_clk_freq=62.5e6):
        # CSR: I2C configuration.
        self._i2c_divider = CSRStorage(16, reset=int(sys_clk_freq / (4 * 400e3)),
            description="I2C clock divider. I2C freq = sys_clk / (4 * (divider+1)).")
        self._i2c_address = CSRStorage(7, reset=0x55,
            description="Si549 I2C address (7-bit, without R/W bit).")

        # CSR: ADPLL scale factor.
        self._adpll_scale = CSRStorage(16, reset=256,
            description="ADPLL scale: adpll = (dac - 0x8000) * scale >> 8. "
                         "Default 256 = 1:1 mapping.")

        # CSR: Force/override mode (bypass WRPC DAC interface).
        self._force       = CSRStorage(
            description="Set to 1 to override DAC input with _force_adpll value.")
        self._force_adpll = CSRStorage(24,
            description="ADPLL value sent when _force=1.")
        self._force_stb   = CSR(
            description="Write 1 to trigger a forced ADPLL write.")

        # CSR: Status.
        self._busy  = CSRStatus(description="1 while an I2C transaction is in progress.")
        self._nack  = CSRStatus(description="1 if the last transaction received a NACK.")

        # # #

        self.submodules.programmer = programmer = ADPLLProgrammer()

        self.comb += [
            programmer.i2c_divider.eq(self._i2c_divider.storage),
            programmer.i2c_address.eq(self._i2c_address.storage),
            self._busy.status.eq(programmer.busy),
            self._nack.status.eq(programmer.nack),
        ]

        # DAC word to ADPLL conversion.
        # Use 32-bit arithmetic: offset = value - 0x8000 (signed)
        #                        adpll  = (offset * scale) >> 8
        dac_offset = Signal((17, True))  # signed 17-bit to hold full range
        adpll_raw  = Signal((33, True))  # signed 33-bit intermediate
        adpll_out  = Signal(24)

        self.comb += [
            dac_offset.eq(Cat(value, 0) - 0x8000),
            adpll_raw.eq(dac_offset * self._adpll_scale.storage),
            adpll_out.eq(adpll_raw[8:32]),
        ]

        # Latch the latest DAC value; send when programmer becomes free.
        adpll_pending = Signal()
        adpll_latched = Signal(24)

        self.sync += [
            # Latch on WRPC load strobe (normal mode) or force strobe.
            If(~self._force.storage & load,
                adpll_latched.eq(adpll_out),
                adpll_pending.eq(1),
            ).Elif(self._force.storage & self._force_stb.re,
                adpll_latched.eq(self._force_adpll.storage),
                adpll_pending.eq(1),
            ),
            # Deassert pending when programmer accepts the strobe.
            If(programmer.stb,
                adpll_pending.eq(0),
            ),
        ]

        self.comb += [
            programmer.adpll.eq(adpll_latched),
            programmer.stb.eq(adpll_pending & ~programmer.busy),
        ]

        # I2C tristate I/O.
        scl_t = TSTriple(1)
        sda_t = TSTriple(1)
        self.specials += [
            scl_t.get_tristate(pads.scl),
            sda_t.get_tristate(pads.sda),
        ]

        self.comb += [
            scl_t.oe.eq(~programmer.scl),
            scl_t.o.eq(0),
            sda_t.oe.eq(~programmer.sda_o),
            sda_t.o.eq(0),
            programmer.sda_i.eq(sda_t.i),
        ]
