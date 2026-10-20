# Hive board sensors

## Context

The hive board sits in the brood box at around 35 °C and 50-80% RH, among wax
and propolis; bees seal any gap they can reach. The INMP441 microphone the
stripboard build used is out of production, and in October 2026 no 3.3 V I2S
MEMS microphone in current production was stocked by LCSC.

## Decision

- **Microphone:** ICS-43434, a bottom-port I2S part with the INMP441's data
  format (Philips I2S, 24 bits in a 32-bit slot, left channel with LR low), so
  `bee_audio` needs no driver change. It is end of life, so lifetime stock is
  bought with the first order and held at JLCPCB. The fallback is the
  TDK T5837, which needs a 1.8 V supply, level shifting and a PDM capture in
  `bee_audio`; the T5848 was rejected for its 1.8 V supply and thin stock.
- **Temperature and humidity:** SHT40-AD1F (address 0x44), whose PTFE membrane
  keeps wax, propolis and liquid water off the sensing element at the cost of a
  slower response that a five-minute interval doesn't notice.
- **Protection:** the microphone hears through a hole in the PCB aligned with a
  hole in the case, sealed by an adhesive ePTFE acoustic vent (1-2 dB loss over
  100 Hz-10 kHz). The SHT40 reads hive air through open 2 mm vent holes, too
  small for bees. The board is conformally coated except over the microphone
  port and the SHT40.

## Consequences

- The ICS-43434 only runs at 6.25-18.75 kHz or 23-51.6 kHz sample rates, so
  `bee_audio` must reject the 4-6.25 kHz and 18.75-23 kHz ranges it currently
  accepts. 8 kHz gives a 512 kHz bit clock, above its 400 kHz minimum.
- If bees propolise the open vents, the SHT40 reads the air in the case rather
  than the hive. They are checked at each inspection; a replaceable
  breathable patch over them is the fallback, without a PCB change.
