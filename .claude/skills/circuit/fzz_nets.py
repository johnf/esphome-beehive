#!/usr/bin/env python3
"""Print parts, schematic nets, physical (breadboard/stripboard) nets and their
differences from a Fritzing .fzz sketch.

Usage: fzz_nets.py <sketch.fzz> [--parts-dir DIR]...

Fritzing encoding notes (the reason this script exists):
- Instances connect via <connect modelIndex= connectorId=> per view. Wires are
  instances too; a wire's two connectors are one conductor. wireFlags 64 =
  breadboard wire, 128 = schematic wire, 4 = PCB trace.
- Generic parts name pins "Pin N"; the real name is in the connector's
  <description>. Part-internal <bus> elements short pins (e.g. all GNDs).
- Stripboard connector N encodes (x, y) as y*1000+x. The "buses" property is
  the list of REMOVED segments: "x.yh" cuts (x,y)-(x+1,y), "x.yv" cuts
  (x,y)-(x,y+1). With "vertical strips" every h segment is listed.
- All ground symbols are one net.
- BUS_OVERRIDES replaces a part's declared buses where the part file is wrong
  for the board actually used.
"""
import glob
import os
import sys
import zipfile
try:
    import defusedxml.ElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET
from collections import defaultdict

PARTS_DIR_CANDIDATES = [
    'fritzing/.fritzing-parts/core',
    os.path.expanduser('~/Fritzing/parts/user'),
    os.path.expanduser('~/.local/share/Fritzing/Fritzing/partfactory/*/core'),
    '/tmp/.mount_fritzi*/fritzing-parts/core',  # running AppImage
    '/usr/share/fritzing/parts/core',
    '/opt/fritzing/fritzing-parts/core',
]
GND_MODULES = {'GroundModuleID'}
# The third-party CN3065 part buses Solar-, Batt- and Out- together. The real
# module has DW01/FS8205 protection FETs between Batt- and the other grounds,
# so Batt- (pins 4 and 7) is its own node.
BUS_OVERRIDES = {
    'CN3065-Mini-Solar-Charger-Module_1': [
        ['connector0', 'connector2'],                 # Solar+
        ['connector1', 'connector3', 'connector9'],   # Solar-, Out-
        ['connector4', 'connector7'],                 # Batt-
        ['connector5', 'connector6', 'connector8'],   # Batt+, Out+
    ],
}
BB_WIRE, SCH_WIRE = 64, 128


class UF:
    def __init__(self):
        self.p = {}

    def find(self, x):
        self.p.setdefault(x, x)
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


class Sketch:
    def __init__(self, fzz, extra_dirs):
        self.pins = {}    # moduleId -> {connectorId: name}
        self.buses = {}   # moduleId -> [[connectorId, ...]]
        with zipfile.ZipFile(fzz) as z:
            fz = [n for n in z.namelist() if n.endswith('.fz')][0]
            self.root = ET.fromstring(z.read(fz))
            for n in z.namelist():
                if n.endswith('.fzp'):
                    self._load_fzp(z.read(n))
        self.inst = {i.get('modelIndex'): i for i in self.root.iter('instance')}
        self.title = {mi: i.findtext('title') for mi, i in self.inst.items()}
        self.mid = {mi: i.get('moduleIdRef') for mi, i in self.inst.items()}
        self._load_core_parts(extra_dirs)

    def _load_fzp(self, data):
        try:
            t = ET.fromstring(data)
        except ET.ParseError:
            return
        names = {}
        for c in t.iter('connector'):
            n = c.get('name') or ''
            d = (c.findtext('description') or '').strip()
            names[c.get('id')] = d if (n.startswith('Pin ') and d) else (n or d)
        self.pins[t.get('moduleId')] = names
        self.buses[t.get('moduleId')] = BUS_OVERRIDES.get(t.get('moduleId')) or [
            [m.get('connectorId') for m in b.iter('nodeMember')] for b in t.iter('bus')]

    def _load_core_parts(self, extra_dirs):
        missing = {m for m in self.mid.values()
                   if m not in self.pins and m != 'WireModuleID' and 'Stripboard' not in m
                   and m not in GND_MODULES and 'PCB' not in m}
        dirs = [d for pat in extra_dirs + PARTS_DIR_CANDIDATES for d in glob.glob(pat)]
        for d in dirs:
            if not missing:
                break
            for f in glob.glob(os.path.join(d, '*.fzp')):
                if not missing:
                    break
                with open(f, 'rb') as fh:
                    data = fh.read()
                for m in list(missing):
                    if f'moduleId="{m}"'.encode() in data:
                        self._load_fzp(data)
                        missing.discard(m)
        self.missing_parts = missing

    # ---- classification
    def is_wire(self, mi):
        return self.mid[mi] == 'WireModuleID'

    def is_strip(self, mi):
        return 'Stripboard' in self.mid[mi]

    def is_gnd(self, mi):
        return self.mid[mi] in GND_MODULES

    def wire_flags(self, mi):
        views = self.inst[mi].find('views')
        if views is None or len(views) == 0:
            return -1
        g = views[0].find('geometry')
        return int(g.get('wireFlags') or 0) if g is not None else -1

    def pin_name(self, mi, cid):
        return self.pins.get(self.mid[mi], {}).get(cid, cid)

    def label(self, mi, cid):
        return f'{self.title[mi]}.{self.pin_name(mi, cid)}'

    @staticmethod
    def strip_xy(cid):
        n = int(cid[len('connector'):])
        return n % 1000, n // 1000

    def strip_edges(self, mi):
        i = self.inst[mi]
        cols, rows = map(int, i.find("property[@name='size']").get('value').split('.'))
        cut = set(i.find("property[@name='buses']").get('value').split())
        for x in range(cols):
            for y in range(rows):
                a = (mi, f'connector{y*1000+x}')
                if x < cols - 1 and f'{x}.{y}h' not in cut:
                    yield a, (mi, f'connector{y*1000+x+1}')
                if y < rows - 1 and f'{x}.{y}v' not in cut:
                    yield a, (mi, f'connector{(y+1)*1000+x}')

    def strip_cuts(self, mi):
        i = self.inst[mi]
        layout = i.find("property[@name='layout']")
        layout = layout.get('value') if layout is not None else 'horizontal strips'
        default = 'h' if 'vertical' in layout else 'v'  # segment type removed wholesale
        cuts = [c for c in i.find("property[@name='buses']").get('value').split()
                if not c.endswith(default)]
        return layout, sorted(cuts, key=lambda s: tuple(int(v) for v in s[:-1].split('.')))

    # ---- net building
    def nets(self, view):
        """view: 'sch' (schematic wires only, no stripboards) or 'bb' (breadboard
        wires + stripboard strips + direct part contacts)."""
        wire_ok = (lambda f: f == SCH_WIRE) if view == 'sch' else (lambda f: f == BB_WIRE)
        vtag = 'schematicView' if view == 'sch' else 'breadboardView'
        uf = UF()
        for mi, i in self.inst.items():
            if self.is_wire(mi):
                if wire_ok(self.wire_flags(mi)):
                    uf.union((mi, 'connector0'), (mi, 'connector1'))
                else:
                    continue
            if view == 'sch' and self.is_strip(mi):
                continue
            v = i.find('views')
            v = v.find(vtag) if v is not None else None
            if v is not None:
                for c in v.iter('connector'):
                    a = (mi, c.get('connectorId'))
                    for k in c.iter('connect'):
                        b = (k.get('modelIndex'), k.get('connectorId'))
                        if b[0] not in self.inst:
                            continue
                        if self.is_wire(b[0]) and not wire_ok(self.wire_flags(b[0])):
                            continue
                        if view == 'sch' and self.is_strip(b[0]):
                            continue
                        uf.union(a, b)
            for bus in self.buses.get(self.mid[mi], []):
                for c in bus[1:]:
                    uf.union((mi, bus[0]), (mi, c))
            if view == 'bb' and self.is_strip(mi):
                for a, b in self.strip_edges(mi):
                    uf.union(a, b)
        gnds = [mi for mi in self.inst if self.is_gnd(mi)]
        for mi in gnds[1:]:
            uf.union((gnds[0], 'connector0'), (mi, 'connector0'))
        groups = defaultdict(set)
        for node in list(uf.p):
            if self.is_wire(node[0]) or self.is_strip(node[0]) or self.is_gnd(node[0]):
                continue
            groups[uf.find(node)].add(node)
        out = []
        for k, nodes in groups.items():
            pins = sorted({self.label(mi, cid) for mi, cid in nodes})
            has_gnd = any(self.is_gnd(m) for m, _ in
                          [n for n in uf.p if uf.find(n) == k])
            if len(pins) > 1 or has_gnd:
                out.append((frozenset(pins), has_gnd))
        return sorted(out, key=lambda t: (-len(t[0]), sorted(t[0])))

    def placements(self):
        """Part pins sitting on stripboard holes, breadboard view."""
        rows = []
        for mi, i in self.inst.items():
            if self.is_wire(mi) or self.is_strip(mi):
                continue
            v = i.find('views')
            v = v.find('breadboardView') if v is not None else None
            if v is None:
                continue
            for c in v.iter('connector'):
                for k in c.iter('connect'):
                    m = k.get('modelIndex')
                    if m in self.inst and self.is_strip(m):
                        x, y = self.strip_xy(k.get('connectorId'))
                        rows.append((self.title[mi], self.pin_name(mi, c.get('connectorId')),
                                     self.title[m], x, y))
        return sorted(rows)


def fmt_net(pins, has_gnd):
    s = ', '.join(sorted(pins))
    return f'{s}  [GND symbol]' if has_gnd else s


def main():
    args = sys.argv[1:]
    extra = []
    while '--parts-dir' in args:
        i = args.index('--parts-dir')
        extra.append(args[i + 1])
        del args[i:i + 2]
    if len(args) != 1:
        sys.exit(__doc__)
    sk = Sketch(args[0], extra)

    print('# PARTS')
    for mi, i in sk.inst.items():
        if sk.is_wire(mi) or sk.is_gnd(mi):
            continue
        props = {p.get('name'): p.get('value') for p in i.findall('property')
                 if p.get('name') not in ('buses',)}
        pins = sk.pins.get(sk.mid[mi])
        print(f'- {sk.title[mi]}: {sk.mid[mi]}'
              + (f' {props}' if props else '')
              + ('' if pins is not None or sk.is_strip(mi) else '  (part definition not found)'))
        if pins and not sk.is_strip(mi):
            print('    pins: ' + ', '.join(f'{cid[len("connector"):]}={n}' for cid, n in pins.items()))

    print('\n# SCHEMATIC NETS (schematic wires + part buses; the design intent)')
    sch = sk.nets('sch')
    for pins, g in sch:
        print('- ' + fmt_net(pins, g))

    placements = sk.placements()
    print('\n# PHYSICAL NETS (breadboard wires + stripboard strips + part buses)')
    bb = [(p, g) for p, g in sk.nets('bb') if len({l.split('.')[0] for l in p}) > 1]
    if not bb:
        print('no part-to-part connections in the breadboard view yet')
    for pins, g in bb:
        print('- ' + fmt_net(pins, g))

    if bb:
        print('\n# SCHEMATIC vs PHYSICAL')
        sch_list = [p for p, _ in sch]
        bb_list = [p for p, _ in bb]
        sch_pins = set().union(*sch_list)
        findings = 0
        for q in bb_list:
            touched = [p for p in sch_list if p & q]
            if len(touched) > 1:
                findings += 1
                print('- MERGED: the build shorts these schematic nets together:')
                for p in touched:
                    print(f'    {fmt_net(p & q, False)}')
            stray = q - sch_pins
            if stray and touched:
                findings += 1
                print(f'- STRAY: {fmt_net(stray, False)} joined to net of {fmt_net(touched[0] & q, False)}')
        for p in sch_list:
            cover = set().union(*[q for q in bb_list if p & q]) & p if any(p & q for q in bb_list) else set()
            if len(cover) < 2:
                findings += 1
                print(f'- NOT BUILT: {fmt_net(p, False)}')
            elif cover < p:
                findings += 1
                print(f'- PARTIAL: {fmt_net(cover, False)} built; missing {fmt_net(p - cover, False)}')
        if not findings:
            print('identical')

    strips = [mi for mi in sk.inst if sk.is_strip(mi)]
    if strips:
        print('\n# STRIPBOARDS')
        for mi in strips:
            layout, cuts = sk.strip_cuts(mi)
            size = sk.inst[mi].find("property[@name='size']").get('value')
            print(f'- {sk.title[mi]}: {size} (cols.rows), {layout}, cuts: {" ".join(cuts) or "none"}')
        print('  pin placements (part.pin -> board(x,y)):' + ('' if placements else ' none'))
        for part, pin, board, x, y in placements:
            print(f'  - {part}.{pin} -> {board}({x},{y})')

    if sk.missing_parts:
        print('\n# WARNING: no part definition for: ' + ', '.join(sorted(sk.missing_parts))
              + '\n  pins print as raw connector ids; pass --parts-dir <fritzing core parts dir>')


if __name__ == '__main__':
    main()
