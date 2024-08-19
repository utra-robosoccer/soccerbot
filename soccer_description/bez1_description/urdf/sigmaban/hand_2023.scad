% scale(1000) import("hand_2023.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=40.000000,h=thickness);
  }
  translate([-134.400000, -20.600000, 0]) {
    rotate([0, 0, 0.0]) {
      cube([99.551184, 41.200000, thickness]);
    }
  }
}
}
