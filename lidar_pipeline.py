"""
LiDAR Point Cloud Preprocessing Pipeline
=========================================
Raw Point Cloud -> Statistical Outlier Removal -> Voxel Downsampling
-> Ground Plane Segmentation -> ICP Registration -> Registered Point Cloud

Dependencies: open3d, numpy
    pip install open3d numpy

Usage:
    Replace load_scan() with o3d.io.read_point_cloud("your_scan.pcd")
    to run this on a real LiDAR capture instead of the synthetic demo scene.
"""

import numpy as np
import open3d as o3d


# ----------------------------------------------------------------------
# Stage 0: Load raw point cloud
# ----------------------------------------------------------------------
def load_scan(path: str) -> o3d.geometry.PointCloud:
    """Load a .pcd / .ply / .xyz point cloud from disk."""
    pcd = o3d.io.read_point_cloud(path)
    print(f"[Stage 0] Loaded raw scan: {len(pcd.points)} points")
    return pcd


# ----------------------------------------------------------------------
# Stage 1: Statistical Outlier Removal (SOR)
# ----------------------------------------------------------------------
def remove_outliers(pcd: o3d.geometry.PointCloud,
                     nb_neighbors: int = 20,
                     std_ratio: float = 2.0):
    """
    Removes sparse noise points (e.g. dust, rain, multipath returns).
    For each point, the mean distance to its `nb_neighbors` nearest
    neighbors is computed; points whose mean distance exceeds
    (global_mean + std_ratio * global_std) are discarded.
    """
    clean, inlier_idx = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    removed = len(pcd.points) - len(clean.points)
    print(f"[Stage 1] Statistical Outlier Removal: "
          f"{len(clean.points)} kept, {removed} removed "
          f"({100 * removed / len(pcd.points):.2f}%)")
    return clean


# ----------------------------------------------------------------------
# Stage 2: Voxel Downsampling
# ----------------------------------------------------------------------
def voxel_downsample(pcd: o3d.geometry.PointCloud, voxel_size: float = 0.15):
    """
    Reduces point density by averaging all points inside each voxel_size
    cube into a single representative point. Cuts computation cost for
    downstream registration/matching with minimal loss of structure.
    """
    ds = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"[Stage 2] Voxel Downsampling ({voxel_size} m): "
          f"{len(pcd.points)} -> {len(ds.points)} points")
    return ds


# ----------------------------------------------------------------------
# Stage 3: Ground Plane Segmentation (RANSAC)
# ----------------------------------------------------------------------
def segment_ground(pcd: o3d.geometry.PointCloud,
                    distance_threshold: float = 0.08,
                    ransac_n: int = 3,
                    num_iterations: int = 1000):
    """
    Fits a plane model ax+by+cz+d=0 using RANSAC and splits the cloud
    into ground inliers and non-ground (obstacle) outliers.
    """
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations)
    ground = pcd.select_by_index(inliers)
    obstacles = pcd.select_by_index(inliers, invert=True)
    a, b, c, d = plane_model
    print(f"[Stage 3] Ground Segmentation: plane "
          f"{a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0 | "
          f"{len(ground.points)} ground / {len(obstacles.points)} obstacle points")
    return ground, obstacles, plane_model


# ----------------------------------------------------------------------
# Stage 4: ICP Registration
# ----------------------------------------------------------------------
def register_icp(source: o3d.geometry.PointCloud,
                  target: o3d.geometry.PointCloud,
                  threshold: float = 0.5,
                  trans_init: np.ndarray = None):
    """
    Aligns `source` onto `target` using point-to-plane Iterative Closest
    Point. Requires per-point normals on both clouds.
    """
    if trans_init is None:
        trans_init = np.eye(4)

    for cloud in (source, target):
        cloud.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

    print(f"[Stage 4] ICP Registration: fitness={result.fitness:.4f}, "
          f"inlier_rmse={result.inlier_rmse:.5f}")
    registered = source.transform(result.transformation)
    return registered, result


# ----------------------------------------------------------------------
# Pipeline driver
# ----------------------------------------------------------------------
def run_pipeline(raw_scan_t: o3d.geometry.PointCloud,
                  raw_scan_t1: o3d.geometry.PointCloud,
                  voxel_size: float = 0.15):
    """
    Runs the full preprocessing + registration pipeline on a pair of
    consecutive LiDAR scans (scan at time t = target/local map,
    scan at time t+1 = source to be aligned).
    """
    # Preprocess reference scan (t)
    target = remove_outliers(raw_scan_t)
    target = voxel_downsample(target, voxel_size)
    ground, obstacles, plane = segment_ground(target)

    # Preprocess incoming scan (t+1)
    source = remove_outliers(raw_scan_t1)
    source = voxel_downsample(source, voxel_size)

    # Register t+1 onto t
    registered, icp_result = register_icp(source, target)

    return {
        "target_preprocessed": target,
        "ground": ground,
        "obstacles": obstacles,
        "plane_model": plane,
        "registered_source": registered,
        "icp_result": icp_result,
    }


def load_kitti_bin(path: str) -> o3d.geometry.PointCloud:
    """
    Load a KITTI Odometry/raw .bin velodyne scan (float32 x,y,z,intensity)
    directly into an Open3D point cloud (intensity channel is dropped).
    Point to a real sequences/00/velodyne/000000.bin here once downloaded
    from https://www.cvlibs.net/datasets/kitti/eval_odometry.php
    """
    pts = np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    print(f"[Stage 0] Loaded KITTI scan {path}: {len(pcd.points)} points")
    return pcd


if __name__ == "__main__":
    # Example A: synthetic / generic .pcd or .ply scans
    #   scan_t  = load_scan("scan_001.pcd")
    #   scan_t1 = load_scan("scan_002.pcd")

    # Example B: real KITTI Odometry data (recommended once you have Sequence 00)
    #   scan_t  = load_kitti_bin("dataset/sequences/00/velodyne/000000.bin")
    #   scan_t1 = load_kitti_bin("dataset/sequences/00/velodyne/000001.bin")

    scan_t = load_scan("scan_t.pcd")
    scan_t1 = load_scan("scan_t1.pcd")
    results = run_pipeline(scan_t, scan_t1)
