from hierarchy_custom_function import *
import os
import re

#supported format:
example_pc_title = "pointcloud_1721122286380527267.ply"

coupled_saving_folder = "your_folder_with_raw_pcs"

pointcloud_files = [f for f in os.listdir(coupled_saving_folder) if f.endswith('.ply')]

pointcloud_timestamps = []
for file in pointcloud_files:
    match = re.search(r'(\d+)', file)
    if match:
        timestamp = int(match.group(1))
        pointcloud_timestamps.append(timestamp)

if pointcloud_timestamps:
    # Sort files and timestamps together
    pointcloud_files, pointcloud_timestamps = zip(*sorted(zip(pointcloud_files, pointcloud_timestamps)))
    pointcloud_files_sorted = list(pointcloud_files)
else:
    print("ERROR NO TIMESTAMP")
    pointcloud_files_sorted = []

print("total PCs", len(pointcloud_files))
starts = 200
ends = 2000

print("analizing: S:", starts, " E:", ends, " TOT:", ends-starts)
timestamp_sorted = []
pointclouds = []
for idx, file_name in enumerate(pointcloud_files_sorted):


    timestamp_str = file_name.split('_')[1].split('.ply')[0]
    timestamp = float(timestamp_str)

    if idx > starts and idx < ends:
        pcd_raw = o3d.io.read_point_cloud(
            os.path.join(coupled_saving_folder, file_name))  # Usa file_name invece di pointcloud_files[idx]
        pcd_raw = convert_to_meters(pcd_raw)

        pointclouds.append(pcd_raw)
        timestamp_sorted.append(timestamp)



fused = hierarchy_slam_icp(pointclouds,timestamp_sorted)
filename = "output_double/fused.ply"
o3d.io.write_point_cloud(filename, fused)
print(f"PointCloud salvata come {filename}")


visualize_pc(fused,"end")