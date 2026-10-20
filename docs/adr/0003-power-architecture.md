# Power architecture

## Context

The device sleeps for five minutes between wakes of about 15 s at ~150 mA,
averaging ~2.3 mA (measured as a 2.7% per day fall on the fuel gauge with the
panel shaded). It runs from a 2000 mAh LiPo charged by a 6 V 1 W panel.

On the stripboard build, a CN3065 linear charger held the panel at 4.77 V in
direct sun, well below its 6 V maximum power point, and delivered 113-121 mA
(0.54-0.58 W, 55-58% of nameplate; Sydney spring, panel not aimed). A linear
charger passes current roughly one-for-one, so charge current, not panel
watts, fills the cell: half an hour of direct sun covers a day's use. Shaded
and badly angled, the same panel gave 5-50 mW. Siting matters far more than
panel size, and panels added for margin go in parallel; series gains nothing
because the charger pulls its input down regardless.

## Decision

- **Charger:** CN3165, the CN3065's successor from the same maker, which also
  throttles its current to hold up a weak panel. An MPPT charger (CN3791) was
  rejected: perhaps 10-20% more current for three times the board area and an
  inductor, MOSFET, diode and sense resistor.
- **Solar monitor:** INA226 high-side between the panel and the charger input,
  behind a reverse-polarity MOSFET.
- **Protection:** DW01A and FS8205A on the main board between the cell and the
  load, in addition to the cell's own protection board, so a replacement cell
  without one is still covered.
- **Feather supply:** the charger output feeds the FeatherS3D's VBAT pin, not
  3V3. LDO2's input is the VBAT/USB node, so feeding 3V3 would leave it
  unpowered. The Feather's own charger only runs on USB; both chargers are
  4.2 V CC/CV and share the cell harmlessly.
- **Fuel gauge:** the Feather's MAX17048 (I2C 0x36) sees the cell through VBAT
  and sits on the always-on LDO1, so its ModelGauge state survives deep sleep.
- **Sensor rail:** every sensor and the I2C pull-ups run from LDO2 (GPIO39),
  which turns off in deep sleep. Anything powered elsewhere draws standby
  current all night.

## Consequences

- The charger burns the panel-to-cell voltage difference as heat inside the
  case, which is negligible at ~120 mA.
- Solar current is only sampled while awake, so panel output during deep sleep
  is invisible.
- The two protection circuits in series trip at slightly different voltages;
  whichever trips first wins, which is harmless.
