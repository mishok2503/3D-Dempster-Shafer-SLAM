import open3d as o3d
import sys

points = []
with open(sys.argv[1], "r") as pc:
    points = list(map(lambda l: list(map(float, l.split())), pc.readlines()))


pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
o3d.visualization.draw_geometries([pcd])