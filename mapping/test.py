import torch

print("PyTorch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())

if torch.cuda.is_available():
    print("GPU Name:", torch.cuda.get_device_name(0))
else:
    print("CUDA not available.")

import testopen3d as o3d

if o3d.core.cuda.is_available():
    print("CUDA is enabled for Open3D!")
else:
    print("CUDA is not enabled for Open3D.")

