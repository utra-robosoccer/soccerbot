% scale(1000) import("x_plate_23_front.stl");

// Sketch PureShapes 6.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, -6.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 6.000000;
translate([0, 0, -thickness]) {
  translate([-72.500000, -57.878267, 0]) {
    rotate([0, 0, 0.0]) {
      cube([145.000000, 115.591551, thickness]);
    }
  }
}
}
