# open3d visualizer for point clouds
import open3d as o3d
import numpy as np
# argument
import argparse

def visualize(folder):
    print("Visualizing point cloud from folder: ", folder)
    pcd = o3d.io.read_point_cloud(folder+"/total_cloud.ply")
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize point cloud')
    parser.add_argument('--folder', type=str, help='Path to point cloud file')
    visualize(parser.parse_args().folder)