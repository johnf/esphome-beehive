// Beehive monitor enclosures. Export one part at a time, e.g.
//   openscad -D 'part="box"' -o stl/box.stl beehive-case.scad
// Parts: box, lid, clamp, foot, fitcheck, hive_base, hive_lid
// Views: assembly, layout, seal_section

part = "assembly";

$fn = 48;
eps = 0.01;

/* ---------- Main box ---------- */

wall = 2.4;
floor_t = 2.4;
corner_r = 8;               // keeps the O-ring bend radius above 3x its diameter

board = [65, 109];
board_t = 1.6;
board_gap = 3;              // left and top edges
connector_clearance = 30;   // right and bottom (terminal) edges
board_z = 15;               // board underside above the floor
component_h = 20;
headroom = 5.4;

inner = [
  board_gap + board[0] + connector_clearance,
  board_gap + board[1] + connector_clearance,
  board_z + board_t + component_h + headroom
];
outer = [inner[0] + 2 * wall, inner[1] + 2 * wall, inner[2] + floor_t];

// Board in inner coordinates; y runs from the bottom (terminal) edge.
board_pos = [board_gap, connector_clearance];
board_right = board_pos[0] + board[0];

// Left edge M3 holes, measured from the board's left and bottom edges
left_hole_inset = 2.5;
left_holes_y = [15, board[1] - 35];
post_d = 7;
pilot_d = 2.6;              // M3 self-tapper

// Right edge clamps, measured from the board's bottom edge
clamps_y = [25, board[1] - 20];
clamp_w = 8;
clamp_ledge = 4;            // support under the board
clamp_gap = 1;              // board edge to clamp column
clamp_col = 8;
clamp_t = 2.5;
clamp_finger = [3, 3];      // overlap onto the board, width
clamp_preload = 0.3;
tower_top = board_z + board_t - clamp_preload;

battery = [54, 60, 7];
battery_pos = [9.5, 68];
charger = [42, 20, 8];      // CN3065
charger_pos = [12, 36];
guide_t = 1.2;
guide_len = 10;
guide_clr = 0.5;

silica_pos = [40, 2];
silica = [30, 14, 6];

// PG9 glands on the +x wall: load cells, Cat6, solar
gland_d = 15.5;
gland_y = [29, 53, 105];
gland_z = floor_t + 13;

// M12 ePTFE vent on the -y wall
vent_d = 12.2;
vent_x = 25;
vent_z = floor_t + 13;

// Lid lugs with captive M3 nuts
lug_depth = 14;
lug_w = 12;
lug_h = 10;
lug_y = [10, outer[1] / 2, outer[1] - 10];  // long sides
lug_end_x = outer[0] / 2;                     // one on each short side
screw_offset = 9;
screw_clear_d = 3.4;
screw_hole_depth = 13;
nut_af = 5.9;
nut_t = 2.8;
nut_depth = 6;

// Face-seal O-ring cord in a groove in the rim, 23% squeeze with the lid flush
oring_d = 2.4;
groove_depth = 1.85;
groove_w = 3.1;
groove_land = 1.6;          // inner wall face to groove
groove_outer_land = 2;
rim_ext = groove_land + groove_w + groove_outer_land - wall;
rim_t = groove_depth + 1.2;

lid_t = 4;
lid_r = 6;
lid_chamfer = 1;
skirt_t = 1.6;
skirt_h = 3;
skirt_clr = 0.4;

groove_offset = groove_land + groove_w / 2;
groove_r = corner_r - wall + groove_offset;
groove_len = 2 * (inner[0] + inner[1] + 4 * groove_offset) - 8 * groove_r + 2 * PI * groove_r;
echo(str("O-ring cord length: ", round(groove_len), " mm"));

// Press-fit feet
foot_d = 14;
foot_h = 6;
foot_peg_d = 5;
foot_peg_h = 4;
foot_hole_d = 5.3;
foot_inset = 10;
foot_boss_d = 10;
foot_boss_h = 3;
foot_pos = [
  [foot_inset, foot_inset],
  [outer[0] - foot_inset, foot_inset],
  [foot_inset, outer[1] - foot_inset],
  [outer[0] - foot_inset, outer[1] - foot_inset]
];

fitcheck_wall_h = 3;

/* ---------- Hive board housing ---------- */

hive_board = [40, 37];
hive_component_h = 17;
hive_solder = 3;
hwall = 2;
hfloor = 2;
htop = 2;
hclr = 0.5;
hboss_zone = 7;
hinner = [
  hive_board[0] + 2 * hclr,
  hive_board[1] + 2 * hclr + 2 * hboss_zone,
  hive_solder + board_t + hive_component_h + 1
];
houter = [hinner[0] + 2 * hwall, hinner[1] + 2 * hwall, hinner[2] + hfloor + htop];
hboard_pos = [hclr, hboss_zone + hclr];
hboard_top = hive_solder + board_t;
hsplit = hboard_top + 3;
hledge_w = 2;
hpost_d = 6;
hpost_y = [hboss_zone / 2, hinner[1] - hboss_zone / 2];
htube_d = 8;
hcbore_d = 6;
hcbore_floor = 3;
hpeg = 2.5;
hpeg_inset = 2;
hcable_d = 8;
hcable_x = 8;
hhole_d = 2;
hhole_pitch = 4;

/* ---------- Helpers ---------- */

module rrect(p0, p1, r) {
  translate(p0) translate([r, r]) offset(r = r) square([p1[0] - p0[0] - 2 * r, p1[1] - p0[1] - 2 * r]);
}

module to_inner() {
  translate([wall, wall, floor_t]) children();
}

module corner_guides(pos, size, h) {
  x0 = pos[0] - guide_clr;
  y0 = pos[1] - guide_clr;
  x1 = pos[0] + size[0] + guide_clr;
  y1 = pos[1] + size[1] + guide_clr;
  for (c = [[x0, y0, 1, 1], [x1, y0, -1, 1], [x0, y1, 1, -1], [x1, y1, -1, -1]]) {
    translate([c[0], c[1], 0]) {
      translate([c[2] < 0 ? 0 : -guide_t, c[3] < 0 ? -guide_len + guide_t : -guide_t, 0])
        cube([guide_t, guide_len, h]);
      translate([c[2] < 0 ? -guide_len + guide_t : -guide_t, c[3] < 0 ? 0 : -guide_t, 0])
        cube([guide_len, guide_t, h]);
    }
  }
}

/* ---------- Main box parts ---------- */

module shell() {
  difference() {
    linear_extrude(outer[2]) rrect([0, 0], [outer[0], outer[1]], corner_r);
    translate([0, 0, floor_t])
      linear_extrude(outer[2]) rrect([wall, wall], [outer[0] - wall, outer[1] - wall], corner_r - wall);
  }
}

module lug() {
  difference() {
    hull() {
      translate([-eps, 0, lug_depth]) cube([lug_depth + eps, lug_w, lug_h]);
      translate([-eps, 0, 0]) cube([eps, lug_w, eps]);
    }
    translate([screw_offset, lug_w / 2, lug_depth + lug_h - screw_hole_depth])
      cylinder(d = screw_clear_d, h = screw_hole_depth + eps);
    translate([screw_offset, lug_w / 2, lug_depth + lug_h - nut_depth - nut_t / 2]) {
      cylinder(d = nut_af / cos(30), h = nut_t, $fn = 6);
      translate([0, -nut_af / 2, 0]) cube([lug_depth, nut_af, nut_t]);
    }
  }
}

module lugs() {
  z = outer[2] - lug_h - lug_depth;
  for (y = lug_y) {
    translate([outer[0], y - lug_w / 2, z]) lug();
    translate([0, y + lug_w / 2, z]) rotate([0, 0, 180]) lug();
  }
  translate([lug_end_x - lug_w / 2, 0, z]) rotate([0, 0, -90]) lug();
  translate([lug_end_x + lug_w / 2, outer[1], z]) rotate([0, 0, 90]) lug();
}

lid_screws = concat(
  [for (y = lug_y) for (x = [-screw_offset, outer[0] + screw_offset]) [x, y]],
  [[lug_end_x, -screw_offset], [lug_end_x, outer[1] + screw_offset]]
);

module inner_outline() {
  rrect([wall, wall], [outer[0] - wall, outer[1] - wall], corner_r - wall);
}

module rim() {
  z0 = outer[2] - rim_t;
  difference() {
    hull() {
      translate([0, 0, z0 - rim_ext]) linear_extrude(eps) rrect([0, 0], [outer[0], outer[1]], corner_r);
      translate([0, 0, z0]) linear_extrude(rim_t) offset(r = rim_ext) rrect([0, 0], [outer[0], outer[1]], corner_r);
    }
    translate([0, 0, z0 - rim_ext - 1]) linear_extrude(rim_t + rim_ext + 2) inner_outline();
  }
}

module oring_groove() {
  translate([0, 0, outer[2] - groove_depth]) linear_extrude(groove_depth + 1) difference() {
    offset(r = groove_land + groove_w) inner_outline();
    offset(r = groove_land) inner_outline();
  }
}

module clamp_tower() {
  translate([board_right - clamp_ledge, -clamp_w / 2, 0])
    cube([clamp_ledge + clamp_gap + clamp_col, clamp_w, board_z]);
  translate([board_right + clamp_gap, -clamp_w / 2, 0])
    cube([clamp_col, clamp_w, tower_top]);
}

module inner_features() {
  for (y = left_holes_y)
    translate([board_pos[0] + left_hole_inset, board_pos[1] + y, 0]) cylinder(d = post_d, h = board_z);
  for (y = clamps_y)
    translate([0, board_pos[1] + y, 0]) clamp_tower();
  corner_guides(battery_pos, battery, 4);
  corner_guides(charger_pos, charger, 2);
  translate(silica_pos) difference() {
    cube(silica);
    translate([guide_t, guide_t, -eps]) cube([silica[0] - 2 * guide_t, silica[1] - 2 * guide_t, silica[2] + 1]);
  }
}

module inner_holes() {
  for (y = left_holes_y)
    translate([board_pos[0] + left_hole_inset, board_pos[1] + y, 1]) cylinder(d = pilot_d, h = board_z);
  for (y = clamps_y)
    translate([board_right + clamp_gap + clamp_col / 2, board_pos[1] + y, tower_top - 10])
      cylinder(d = pilot_d, h = 10 + eps);
}

module box() {
  difference() {
    union() {
      shell();
      rim();
      lugs();
      to_inner() translate([0, 0, -eps]) inner_features();
      for (p = foot_pos) translate([p[0], p[1], floor_t - eps]) cylinder(d = foot_boss_d, h = foot_boss_h);
    }
    to_inner() inner_holes();
    oring_groove();
    for (y = gland_y)
      translate([outer[0] - wall - 1, y, gland_z]) rotate([0, 90, 0]) cylinder(d = gland_d, h = wall + 2);
    translate([vent_x, -1, vent_z]) rotate([-90, 0, 0]) cylinder(d = vent_d, h = wall + 2);
    for (p = foot_pos)
      translate([p[0], p[1], -eps]) cylinder(d = foot_hole_d, h = floor_t + foot_boss_h - 1);
  }
}

module fitcheck() {
  intersection() {
    box();
    union() {
      translate([-1, -1, -1]) cube([outer[0] + 2, outer[1] + 2, floor_t + fitcheck_wall_h + 1]);
      translate([wall, wall, 0]) cube([inner[0], inner[1], outer[2]]);
    }
  }
}

// Modelled as fitted, underside at z = 0; print it flipped (see export).
module lid() {
  outline = [[-lug_depth, -lug_depth], [outer[0] + lug_depth, outer[1] + lug_depth]];
  difference() {
    union() {
      hull() {
        linear_extrude(lid_t - lid_chamfer) rrect(outline[0], outline[1], lid_r);
        linear_extrude(lid_t) offset(delta = -lid_chamfer) rrect(outline[0], outline[1], lid_r);
      }
      translate([0, 0, -skirt_h]) linear_extrude(skirt_h + eps) difference() {
        offset(r = -skirt_clr) inner_outline();
        offset(r = -skirt_clr - skirt_t) inner_outline();
      }
    }
    for (s = lid_screws) translate([s[0], s[1], -1]) cylinder(d = screw_clear_d, h = lid_t + 2);
  }
}

module clamp() {
  difference() {
    union() {
      cube([clamp_col, clamp_w, clamp_t]);
      translate([-clamp_gap - clamp_finger[0], (clamp_w - clamp_finger[1]) / 2, 0])
        cube([clamp_gap + clamp_finger[0] + eps, clamp_finger[1], clamp_t]);
    }
    translate([clamp_col / 2, clamp_w / 2, -1]) cylinder(d = screw_clear_d, h = clamp_t + 2);
  }
}

module foot() {
  cylinder(d = foot_d, h = foot_h);
  cylinder(d = foot_peg_d, h = foot_h + foot_peg_h);
}

/* ---------- Hive housing parts ---------- */

module hive_shell(z0, z1) {
  translate([0, 0, z0]) linear_extrude(z1 - z0)
    rrect([0, 0], [houter[0], houter[1]], 3);
}

module hive_cavity() {
  translate([hwall, hwall, hfloor]) cube(hinner);
}

module hive_cable_cut() {
  translate([hwall + hcable_x, -1, hfloor + hsplit]) rotate([-90, 0, 0]) cylinder(d = hcable_d, h = hwall + 2);
}

module hive_base() {
  difference() {
    union() {
      difference() {
        hive_shell(0, hfloor + hsplit);
        hive_cavity();
      }
      translate([hwall, hwall, hfloor]) {
        for (x = [0, hinner[0] - hclr - hledge_w])
          translate([x, hboard_pos[1], 0]) cube([hclr + hledge_w, hive_board[1], hive_solder]);
        for (y = hpost_y) translate([hinner[0] / 2, y, 0]) cylinder(d = hpost_d, h = hsplit);
        // cable-tie anchor
        translate([hcable_x - 5, 1, 0]) {
          cube([2, 3, 5.5]);
          translate([8, 0, 0]) cube([2, 3, 5.5]);
          translate([0, 0, 4]) cube([10, 3, 1.5]);
        }
      }
    }
    for (y = hpost_y)
      translate([hwall + hinner[0] / 2, hwall + y, hfloor + 0.5]) cylinder(d = pilot_d, h = hsplit);
    hive_cable_cut();
  }
}

// avoid: [[x, y, r], ...] in grid coordinates; holes within r are skipped
module hive_grid(size, avoid = []) {
  nx = max(0, floor((size[0] - hhole_pitch) / hhole_pitch));
  ny = max(0, floor((size[1] - hhole_pitch) / hhole_pitch));
  for (i = [0:nx], j = [0:ny]) {
    p = [(size[0] - nx * hhole_pitch) / 2 + i * hhole_pitch, (size[1] - ny * hhole_pitch) / 2 + j * hhole_pitch];
    if (len([for (a = avoid) if (norm(p - [a[0], a[1]]) < a[2]) 1]) == 0)
      translate(p) children();
  }
}

// Modelled as fitted; print it upside down.
module hive_lid() {
  zb = hfloor + hsplit;
  zt = houter[2];
  tubes = [for (y = hpost_y) [hwall + hinner[0] / 2, hwall + y]];
  wall_rows = [for (z = [zb + 3 : hhole_pitch : zt - htop - 2]) z];
  // peg centres in top-grid coordinates
  pegs = [for (x = [hpeg_inset, hive_board[0] - hpeg_inset - hpeg], y = [hpeg_inset, hive_board[1] - hpeg_inset - hpeg])
    [hboard_pos[0] + x + hpeg / 2, hclr + y + hpeg / 2, hpeg / 2 + hhole_d / 2 + 1]];
  difference() {
    union() {
      difference() {
        hive_shell(zb, zt);
        translate([0, 0, -htop]) hive_cavity();
      }
      for (t = tubes) translate([t[0], t[1], zb]) cylinder(d = htube_d, h = zt - zb);
      translate([hwall + hboard_pos[0], hwall + hboard_pos[1], hfloor + hboard_top])
        for (x = [hpeg_inset, hive_board[0] - hpeg_inset - hpeg], y = [hpeg_inset, hive_board[1] - hpeg_inset - hpeg])
          translate([x, y, 0]) cube([hpeg, hpeg, zt - hfloor - hboard_top - eps]);
    }
    for (t = tubes) {
      translate([t[0], t[1], zb - 1]) cylinder(d = screw_clear_d, h = zt - zb + 2);
      translate([t[0], t[1], zb + hcbore_floor]) cylinder(d = hcbore_d, h = zt);
    }
    hive_cable_cut();
    translate([hwall, hwall + hboss_zone, zt - htop - 1])
      hive_grid([hinner[0], hinner[1] - 2 * hboss_zone], pegs) cylinder(d = hhole_d, h = htop + 2, $fn = 12);
    for (z = wall_rows, x = [-1, houter[0] - hwall - 1])
      translate([x, hwall + hboss_zone, z])
        hive_grid([0, hinner[1] - 2 * hboss_zone]) rotate([0, 90, 0]) cylinder(d = hhole_d, h = hwall + 2, $fn = 12);
    end_xs = [for (x = [hwall + 3 : hhole_pitch : houter[0] - hwall - 3]) if (abs(x - houter[0] / 2) > htube_d / 2 + 2) x];
    for (z = wall_rows, x = end_xs, y = [-1, houter[1] - hwall - 1])
      if (y > 0 || abs(x - hwall - hcable_x) > hcable_d / 2 + 2)
        translate([x, y, z]) rotate([-90, 0, 0]) cylinder(d = hhole_d, h = hwall + 2, $fn = 12);
  }
}

/* ---------- Output ---------- */

module board_dummies() {
  to_inner() {
    %translate([board_pos[0], board_pos[1], board_z]) cube([board[0], board[1], board_t]);
    %translate([board_pos[0], board_pos[1], board_z + board_t]) cube([board[0], board[1], component_h]);
    %translate(battery_pos) cube(battery);
    %translate(charger_pos) cube(charger);
  }
}

module label(t, size = 5) {
  color("black") linear_extrude(0.2) text(t, size = size, halign = "center", valign = "center");
}

// Colour only shows in preview (F5 / PNG export without --render).
module layout() {
  box();
  to_inner() {
    color("royalblue") translate(battery_pos) cube(battery);
    translate([battery_pos[0] + battery[0] / 2, battery_pos[1] + battery[1] / 2, battery[2]]) label("BATTERY");
    color("firebrick") translate(charger_pos) cube(charger);
    translate([charger_pos[0] + charger[0] / 2, charger_pos[1] + charger[1] / 2, charger[2]]) label("CN3065", 4);
    color("seagreen", 0.45) translate([board_pos[0], board_pos[1], board_z]) cube([board[0], board[1], board_t]);
    color("white", 0.12) translate([board_pos[0], board_pos[1], board_z + board_t]) cube([board[0], board[1], component_h]);
    translate([board_pos[0] + board[0] / 2, board_pos[1] + board[1] - 12, board_z + board_t + component_h]) label("FEATHER END");
    translate([board_pos[0] + board[0] / 2, board_pos[1] + 3, board_z + board_t + component_h]) label("TERMINALS", 4);
    for (y = clamps_y)
      color("orange") translate([board_right + clamp_gap, board_pos[1] + y - clamp_w / 2, tower_top]) clamp();
    translate([board_right + connector_clearance / 2 + clamp_gap, inner[1] / 2, 0.5]) rotate(90) label("CABLE ZONE");
    translate([inner[0] / 2, connector_clearance / 2 + 4, 0.5]) label("CABLE ZONE");
  }
  translate([outer[0] + 30, 0, 0]) {
    hive_base();
    color("seagreen", 0.45)
      translate([hwall + hboard_pos[0], hwall + hboard_pos[1], hfloor + hive_solder]) cube([hive_board[0], hive_board[1], board_t]);
    color("white", 0.12)
      translate([hwall + hboard_pos[0], hwall + hboard_pos[1], hfloor + hboard_top]) cube([hive_board[0], hive_board[1], hive_component_h]);
    translate([houter[0] / 2, houter[1] / 2, hfloor + hboard_top + hive_component_h]) label("HIVE BOARD", 4);
  }
}

// Cross-section through a long wall: uncompressed O-ring overlapping the flush lid shows the squeeze.
module seal_section() {
  y = outer[1] / 2 + 20;
  module cut() projection(cut = true) translate([0, 0, -y]) rotate([-90, 0, 0]) mirror([0, 1, 0]) children();
  color("steelblue") cut() box();
  color("orange") cut() translate([0, 0, outer[2]]) lid();
  color("black", 0.7) translate([wall - groove_offset, outer[2] - groove_depth + oring_d / 2]) circle(d = oring_d);
}

if (part == "box") box();
else if (part == "lid") translate([0, 0, lid_t]) mirror([0, 0, 1]) lid();
else if (part == "clamp") clamp();
else if (part == "foot") foot();
else if (part == "fitcheck") fitcheck();
else if (part == "hive_base") hive_base();
else if (part == "hive_lid") translate([0, 0, houter[2]]) mirror([0, 0, 1]) hive_lid();
else if (part == "layout") layout();
else if (part == "seal_section") seal_section();
else if (part == "assembly") {
  box();
  board_dummies();
  translate([0, 0, outer[2] + 30]) lid();
  to_inner() for (y = clamps_y)
    translate([board_right + clamp_gap, board_pos[1] + y - clamp_w / 2, board_z + board_t + 5]) clamp();
  for (p = foot_pos) translate([p[0], p[1], -foot_h - 10]) foot();
  translate([outer[0] + 40, 0, 0]) {
    hive_base();
    translate([0, 0, 20]) hive_lid();
  }
}
