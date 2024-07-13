import os
from os.path import expanduser

import numpy as np
import open3d as o3d

home = expanduser("~")
print(o3d.__version__)

print("Simplifying meshes in Open3D...")

# iterate over files in
# that directory
directory = home + "/catkin_ws/src/soccerbot/bez1_description/meshes"
new_dir = home + "/catkin_ws/src/soccerbot/bez1_description/Bez1_meshes"
if not os.path.exists(new_dir):
    os.mkdir(new_dir)
    print(f"----------------------- creating {new_dir}")
for filename in os.listdir(directory):
    f = os.path.join(directory, filename)
    # checking if it is a file
    if os.path.isfile(f):

        print(f"----------------------- parsing {os.path.basename(f)}")

        if os.path.basename(f)[-3:] == "stl":
            mesh = o3d.io.read_triangle_mesh(f)
            print("Vertices:")
            print(len(mesh.vertices))
            print("Triangles:")
            print(len(mesh.triangles))
            if (
                os.path.basename(f) == "head.stl"
                or os.path.basename(f) == "left_bicep.stl"
                or os.path.basename(f) == "right_bicep.stl"
                or os.path.basename(f) == "torso.stl"
            ):
                target_triangles = 21000
            else:
                target_triangles = int(len(mesh.triangles) * 0.5)

            mesh.compute_vertex_normals()
            mesh_smp = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles, maximum_error=1.0, boundary_weight=0.5)
            print(f"Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles")
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(os.path.join(new_dir, os.path.basename(f)), mesh_smp)
