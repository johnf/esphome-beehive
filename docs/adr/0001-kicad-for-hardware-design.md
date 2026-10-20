# KiCad for hardware design

## Context

The circuit was prototyped in Fritzing, whose breadboard view suited the
stripboard build. The next hardware is a fab-assembled PCB (ADR 0002).
Fritzing's PCB tooling is weak, it has no headless checks, and its files are
hard for an agent to read or edit; the repo needed a Python extractor just to
list the nets.

## Decision

The hardware is designed in KiCad 10, under `kicad/`, and that project is the
design of record.

- Symbols, footprints and 3D models used by the project live in project-local
  libraries referenced through `${KIPRJMOD}`, so the project opens identically
  on any machine and in CI.
- Every part carries its LCSC part number for JLCPCB assembly.
- CI runs `kicad-cli` ERC and DRC and exports the schematic PDF, BOM and
  Gerbers.
- Agent edits go through Konnect (an MCP server and KiCad plugin). Schematic
  tools edit the `.kicad_sch` files directly; PCB tools need KiCad running
  with the board open.
- The case model imports the board outline exported by `kicad-cli`, so the
  enclosure follows the PCB.

Fritzing is retired. The stripboard design, its custom parts and the
`fzz_nets.py` extractor survive on the `stripboard` branch until the PCBs are
deployed, and permanently under the `stripboard-v1` tag.

## Consequences

- Konnect is beta and least tested on Linux.
- Stripboard units run firmware from the `stripboard` branch, so their device
  configurations must set the package `ref:` to it before PCB firmware lands
  on `main`.
- The schematic is redrawn, not converted, and checked by hand against the
  Fritzing nets and each chip's datasheet reference circuit before the first
  order.
