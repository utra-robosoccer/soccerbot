% scale(1000) import("humerus_plate.stl");

// Sketch PureShapes 3.0
multmatrix([[0.0, 0.0, -1.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([-105.000000, -0.000000, 0]) {
    rotate([0, 0, 0.0]) {
      cube([105.000000, 30.000000, thickness]);
    }
  }
}
}
