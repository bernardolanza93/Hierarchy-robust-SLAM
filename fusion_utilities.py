import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def convert_to_meters(pcd):
    points = np.asarray(pcd.points) / 1000.0  # Convert mm to meters
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def downsample_point_cloud(point_cloud, voxel_size):
    """
    Downsample a point cloud using a voxel grid filter and remove duplicated points.

    Parameters:
        point_cloud (o3d.geometry.PointCloud): The input point cloud.
        voxel_size (float): The voxel size for downsampling.

    Returns:
        o3d.geometry.PointCloud: The downsampled point cloud.
    """
    # Downsample the point cloud using voxel grid filter
    downsampled_pc = point_cloud.voxel_down_sample(voxel_size)

    # Remove duplicate points
    downsampled_pc = downsampled_pc.remove_duplicated_points()

    return downsampled_pc


def remove_isolated_points(pcd, nb_neighbors=10, radius=500.0):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    return pcd_cleaned


def icp_open3d(source, target, threshold = 0.1 , max_iterations=250):
    initial_transformation = np.eye(4)

    # Soglia di distanza per ICP
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )
    return icp_result.transformation


def visualize_pc(pc, title):
    point_size = 2
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)

    # Aggiungi le geometrie una alla volta
    vis.add_geometry(pc)


    opt = vis.get_render_option()
    # Imposta il colore di sfondo (colore RGB normalizzato)
    opt.background_color = np.array([0.1, 0.1, 0.1])  # Colore nero
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()



def plot_trajectory_3d(timestamp_dict):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Ordina i timestamp
    sorted_timestamps = sorted(timestamp_dict.keys())

    # Inizializza la posizione iniziale (matrice identità)
    current_position = np.eye(4)

    # Liste per memorizzare le coordinate x, y, z
    x = [current_position[0, 3]]
    y = [current_position[1, 3]]
    z = [current_position[2, 3]]

    # Applica le trasformazioni in ordine di timestamp
    for ts in sorted_timestamps:
        transform = timestamp_dict[ts]
        current_position = np.dot(current_position, transform)
        x.append(current_position[0, 3])
        y.append(current_position[1, 3])
        z.append(current_position[2, 3])

    # Plotta la traiettoria 3D
    ax.plot(x, y, z, marker='o', linewidth=0.2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory 3D Plot')

    # Assicura che gli assi abbiano la stessa scala
    ax.set_aspect('auto')
    max_range = np.array([max(x)-min(x), max(y)-min(y), max(z)-min(z)]).max()
    mid_x = (max(x) + min(x)) * 0.5
    mid_y = (max(y) + min(y)) * 0.5
    mid_z = (max(z) + min(z)) * 0.5
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

    plt.show()
