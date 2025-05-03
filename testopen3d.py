import open3d as o3d
print(o3d.version)
print(o3d.core.cuda.is_available())
print(o3d.core.cuda.device_count())