import open3d as o3d
import numpy as np
from ompl import base as ob
from ompl import geometric as og

VOXEL_SIZE = .01

def main():
    mesh = o3d.io.read_triangle_mesh("dock.stl")
    mesh.translate([431.8, 0, 0])
    mesh.scale(0.001, center=[0, 0, 0])  # Convert mm to meters
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=VOXEL_SIZE)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])

    o3d.io.write_voxel_grid("dock_voxel.ply", voxel_grid)
    
    # Visualize results
    geometries = [voxel_grid, axis]
    o3d.visualization.draw_geometries(geometries)


if __name__ == '__main__':
    main()