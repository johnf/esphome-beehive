---
name: circuit
description: Load the circuit from the Fritzing sketch into context. Use before answering anything about the circuit, wiring, nets, pin assignments, part connections, or the stripboard layout, and before reviewing the schematic or checking the build against it.
---

# Circuit

The sketch is `fritzing/BeeHive.fzz`. Everything about the circuit is derived
from it; Fritzing's XML netlist export is not needed.

1. Run the extractor and read all of its output:

   ```bash
   python3 .claude/skills/circuit/fzz_nets.py fritzing/BeeHive.fzz
   ```

2. Answer from the **SCHEMATIC NETS**. They are the design of record. Use
   **PHYSICAL NETS** and **SCHEMATIC vs PHYSICAL** only for questions about
   the stripboard build. A review is done when every schematic net has been
   considered, not just the ones that look suspicious.

## Reading the output

- **PARTS** lists each label (FeatherS3D, INA226, C1...) with its pin index
  to name map. The FeatherS3D part has duplicate-looking pins: 18/19 are the
  STEMMA connector, 20/21 the header SDA/SCL on IO8/IO9, 36/37 the JST battery
  pads. Ceramic capacitor pins are named 0 and 1.
- Nets are printed as `label.pin`. `[GND symbol]` marks the net tied to the
  schematic ground symbols.
- **MERGED in build** means a stripboard strip or breadboard wire shorts two
  schematic nets. **NOT BUILT** means the schematic connection has no physical
  route yet, which is normal while placement is in progress.
- **STRIPBOARDS** gives board size, strip direction, the cut list as `x.yv`
  (a cut between hole (x,y) and (x,y+1)), and where each part pin sits.

## Facts the sketch cannot tell you

- FeatherS3D LDO2 (the gated sensor rail, GPIO39) is fed from the VBAT/USB
  node, not the 3.3 V rail. The board schematic is
  `series_d/schematics/schematic-feathers3d-p1.pdf` in the
  `UnexpectedMaker/esp32s3` GitHub repo; the pinout card is beside it.
- Firmware pin and I2C address assignments live in the `substitutions` block
  of `beehive-monitor.yaml`. Cross-check them against the nets when reviewing.
- The INA226 module pins IN+ and IN- are the two ends of its 0.1 Ω shunt; VBS
  must be wired to the bus being measured.
- The CN3065 part file buses Batt- with the other grounds; the extractor
  overrides that (`BUS_OVERRIDES`) because the module's protection FETs sit
  between Batt- and Out-. The cell must only touch Batt+ and Batt-.
- C1 (470 µF) is bulk decoupling on VBAT at the FeatherS3D. C2 and C3
  (100 nF) decouple the LDO2 rail and belong physically at the INA226 and
  INMP441 VCC pins, even though the schematic hangs them off the rail.
