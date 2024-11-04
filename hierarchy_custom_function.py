import numpy as np
from decorator import *
from fusion_utilities import *
import open3d as o3d



import numpy as np
from decorator import *
from fusion_utilities import *
import open3d as o3d

@timeit
def hierarchy_slam_icp(input_all_pointcloud, timestamp_sorted):
    # Initialize variables for the starting point clouds and transformation matrices.
    # The first point cloud (index 0) acts only as a target for transformations and is not modified.
    epoch_start_pointclouds = input_all_pointcloud
    epoch = 0

    # Initialize transformations for the previous epoch, starting with identity matrices.
    last_epoch_transformation = [np.eye(4) for _ in range(len(input_all_pointcloud))]
    x_y = []
    timestamp_dict = {}

    # Continue merging point clouds until only one remains.
    while len(epoch_start_pointclouds) > 1:
        start_time = time.time()
        epoch += 1
        i = 1

        new_halfed_pointcloud = []
        transformation_current = []

        print(f"START Epoch {epoch}, Number of PCs: {len(epoch_start_pointclouds)}")

        # Loop through pairs of point clouds in the current epoch.
        while i < len(epoch_start_pointclouds):

            if len(epoch_start_pointclouds) == 2:
                print("LAST EPOCH")
                print(f"Previous epoch transformations: {len(last_epoch_transformation)}")
                print(f"Point clouds in this epoch: {len(epoch_start_pointclouds)}")

            if i > 0:
                # Retrieve prior transformations for the current pair of point clouds.
                transform_previous_1 = last_epoch_transformation[i-1]
                transform_previous_2 = last_epoch_transformation[i]

                # Combine transformations to align the current source point cloud with the target.
                prior_transformation_composed = np.dot(transform_previous_1, transform_previous_2)

                # Define the source and target point clouds for the current transformation.
                source_raw = epoch_start_pointclouds[i]
                current_source = o3d.geometry.PointCloud(source_raw)
                current_source.transform(transform_previous_1)
                target = epoch_start_pointclouds[i-1]

                # Downsample the point clouds to improve processing efficiency.
                VOXEL_VOLUME = 0.03
                current_source = downsample_point_cloud(current_source, VOXEL_VOLUME)
                target = downsample_point_cloud(target, VOXEL_VOLUME)

                # Optional visualization for ICP process.
                SHOW_ICP_PROCESS = 0
                if SHOW_ICP_PROCESS:
                    if len(current_source.points) < 600 or len(target.points) < 600:
                        target = remove_isolated_points(target, nb_neighbors=12, radius=0.7)
                        current_source = remove_isolated_points(current_source, nb_neighbors=12, radius=0.8)

                    yellow, red, green, blue = np.array([1, 1, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
                    current_source.colors = o3d.utility.Vector3dVector(np.tile(yellow, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(red, (len(target.points), 1)))
                    pre_icp = target + current_source
                    visualize_pc(pre_icp, "Pre-ICP visualization")

                # Apply ICP transformation to align the source and target point clouds.
                updated_transform_icp_result = icp_open3d(current_source, target, 0.05, 200)
                x_y.append(updated_transform_icp_result[0, -3:])
                total_transformation = np.dot(prior_transformation_composed, updated_transform_icp_result)

                # Transform the source and merge it with the target.
                transformed_icp_source = o3d.geometry.PointCloud(current_source)
                transformed_icp_source.transform(updated_transform_icp_result)
                merged_pcd = target + transformed_icp_source

                if SHOW_ICP_PROCESS:
                    visualize_pc(merged_pcd, "Merged")

                # Clean the merged point cloud by removing isolated points.
                merged_pcd = remove_isolated_points(merged_pcd, nb_neighbors=12, radius=0.4)
                transformation_current.append(total_transformation)
                new_halfed_pointcloud.append(merged_pcd)

                # Handle cases where an odd number of point clouds remains.
                if i == len(epoch_start_pointclouds) - 2 and len(epoch_start_pointclouds) % 2 != 0:
                    print(f"Epoch {epoch}, Odd number of PCs, adding last PC")
                    last_pc = epoch_start_pointclouds[i + 1]
                    last_transformation = last_epoch_transformation[i + 1]
                    transformation_current.append(last_transformation)
                    new_halfed_pointcloud.append(last_pc)

                # Update timestamp dictionary with transformations for tracking the trajectory.
                timestamp_index = i // 2 if epoch == 1 else len(timestamp_sorted) // (2 ** (epoch - 1)) + i // 2
                timestamp_dict[timestamp_sorted[timestamp_index]] = updated_transform_icp_result

            i += 2

        # Prepare for the next epoch by updating the starting point clouds and transformations.
        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_transformation = transformation_current
        elapsed_time = round(time.time() - start_time, 3)
        print(f"Epoch {epoch} completed in {elapsed_time} seconds, PCs created: {len(new_halfed_pointcloud)}")

    # Extract x, y, z coordinates for visualization.
    x_values = [item[0] for item in x_y]
    y_values = [item[1] for item in x_y]
    z_values = [item[2] for item in x_y]

    # Plot the trajectory based on timestamped transformations.
    plot_trajectory_3d(timestamp_dict)

    return new_halfed_pointcloud[0]
