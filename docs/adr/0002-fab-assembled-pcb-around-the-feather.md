# Fab-assembled PCBs around a plugged-in FeatherS3D

## Context

The stripboards work but their joints and jumpers flex and corrode in a hive,
each unit takes hours to build, and the modules make the boards large. Most
modules (INA226, NAU7802, SHT40, the charger, the microphone) are one chip
and a few passives. The FeatherS3D is not: replacing it means designing the
ESP32-S3 module and antenna keep-out, USB, both LDOs, the GPIO39-gated sensor
rail, the MAX17048 fuel gauge and the battery path.

## Decision

- JLCPCB fabricates and assembles everything, through-hole parts included. It
  was the cheapest for runs of 1-10 boards built from LCSC stock in October
  2026.
- The FeatherS3D stays permanently and plugs into fab-fitted female headers.
  Every other module is replaced by its chip on the board.
- Two boards, as now: a main board under the hive and a hive board inside the
  brood box, joined by a cable (ADR 0004). The hive board holds only the
  SHT40, the microphone, its connector and decoupling.
- Both boards are as small as sensible; the case is redesigned around them.
- Two layers, with the bottom kept as an almost unbroken ground plane under
  the NAU7802. Four layers were wanted for a forgiving first layout if they cost
  under about US$20 more, but five 72 x 56 mm boards quoted at US$74 against
  US$4.
- The first revision is the JLCPCB minimum order, with test pads on every
  power rail and on the I2C and I2S lines.
- Firmware is not kept compatible with the stripboard units. Pins, addresses
  and rails can change where the layout benefits.

## Consequences

- Every part chosen so far is an LCSC Extended part, so each order pays about
  US$3 per unique part in feeder fees.
- A dead Feather is swapped without desoldering, at the cost of 8-11 mm of
  board stack height. The Feather needs a standoff or strap point to stop it
  working loose.
- The Feather's own charger, LDO2 rail and MAX17048 remain part of the power
  design (ADR 0003).
