import open3d as o3d

path_obj = '/home/gnij/UnevenModel/SinglePeak/singlepeak.obj'
mesh = o3d.io.read_triangle_mesh(path_obj)  # 确保这一步可以成功加载模型
mesh.compute_vertex_normals()

print("Visualizing the OBJ mesh...")
o3d.visualization.draw_geometries([mesh])

print("Sampling points using Poisson disk...")
pcd = mesh.sample_points_poisson_disk(number_of_points=100000, init_factor=10)

print("Visualizing the Poisson disk sampled points...")
o3d.visualization.draw_geometries([pcd])

output_path = "/home/gnij/UnevenModel/SinglePeak/singlepeak.pcd"
print(f"Writing the point cloud data to {output_path}")
o3d.io.write_point_cloud(output_path, pcd, write_ascii=True)