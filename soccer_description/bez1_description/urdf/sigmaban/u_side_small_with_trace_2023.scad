% scale(1000) import("u_side_small_with_trace_2023.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([37.500000, 0.000000, 0]) {
    rotate([0, 0, 180.0]) {
      cube([37.500000, 47.500000, thickness]);
    }
  }
}
}
