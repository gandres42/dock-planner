import open3d as o3d
import numpy as np

VOXEL_SIZE = .01
ROBOT_DIMENSIONS = [.4, .55, .4]

# def main():
#     mesh = o3d.io.read_triangle_mesh("simple_dock.stl")
#     # mesh.translate([431.8, 0, 0])
#     # mesh.scale(0.001, center=[0, 0, 0])  # Convert mm to meters
#     voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=VOXEL_SIZE)
#     axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])
#
#     o3d.io.write_voxel_grid("dock_voxel.ply", voxel_grid)
#
#     # Visualize results
#     geometries = [voxel_grid, axis]
#     o3d.visualization.draw_geometries(geometries)

def main():
    mesh = o3d.io.read_triangle_mesh("simple_dock.stl")
    # mesh.translate([431.8, 0, 0])
    # mesh.scale(0.001, center=[0, 0, 0])  # Convert mm to meters
    point_cloud = mesh.sample_points_uniformly(number_of_points=1000)

    clouds = [point_cloud]
    for dx in np.linspace(0, ROBOT_DIMENSIONS[0]):
        for dy in np.linspace(-ROBOT_DIMENSIONS[1] / 2, ROBOT_DIMENSIONS[1] / 2):
            for dz in np.linspace(-ROBOT_DIMENSIONS[2] / 2, ROBOT_DIMENSIONS[2] / 2):
                tmp_cloud = point_cloud.__copy__()
                tmp_cloud.translate([dx, dy, dz])
                clouds.append(tmp_cloud)
    merged_cloud = clouds[0]
    for cloud in clouds[1:]:
        merged_cloud += cloud
    clouds = [merged_cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])]
    o3d.visualization.draw_geometries(clouds) # type: ignore


    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(merged_cloud, voxel_size=VOXEL_SIZE)
    o3d.io.write_voxel_grid("dock_voxel_gapped.ply", voxel_grid)
    o3d.visualization.draw_geometries([voxel_grid])


if __name__ == '__main__':
    main()