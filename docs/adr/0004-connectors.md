# Connectors

## Context

Every cable is made up in the field, often by the beekeeper, and the stripboard
build's JST-XH sockets meant crimping dozens of tiny contacts. Cables enter the
main case through glands, so connectors mate on the board inside the sealed
case.

## Decision

- **Hive cable:** an unshielded RJ45 jack without magnetics on each board,
  silkscreened "NOT ETHERNET", carrying 3V3, SDA, SCL, WS, SCK, SD and two
  grounds over Cat5e or Cat6. Any length of patch lead works, the latch holds
  it, and the jack sits just behind the gland. The jack pinout puts SCK with
  GND, SCL with GND, SDA with 3V3, and WS with SD on the T568 pairs (pins 1-2,
  3-6, 4-5, 7-8), so any straight patch lead twists each clock with a ground
  and the cable can run a metre or two. WS and SD can share a pair because
  both change on the falling edge of SCK and SD is sampled on the rising edge.
- **Load cells:** four 3-way pluggable screw terminals, CELL 1 to CELL 4, each
  taking one cell's red, white and black leads. The board joins the outer
  wires like to like round the platform (white 1-2, black 2-3, white 3-4,
  black 4-1) into a full Wheatstone bridge. Like must join like: within a cell
  the white and black halves change in opposite directions under load, so a
  white-to-black ring cancels in every arm.
- **Solar panel:** a 2-way pluggable screw terminal, so a bare-lead panel needs
  no crimping and can't be confused with the battery plug.
- **Battery:** JST-PH 2-way, matching the LiPo's own lead so it is never cut
  and recrimped.

## Consequences

- An RJ45 plug won't pass through a PG9 gland, so the cable is threaded
  through first and crimped after.
- Plugging an Ethernet device into the hive jack would put 3V3 on its pins;
  the silkscreen is the only guard.
- Screw terminals are not keyed, so the silkscreen carries the colour order and
  the solar input relies on its reverse-polarity MOSFET.
