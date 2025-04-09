import onnxruntime as ort
import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt

# --- Load RGB and depth images ---
target_size = (480, 640)  # H, W

rgb = cv2.imread("colors.png")  # BGR
rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
rgb = cv2.resize(rgb, (target_size[1], target_size[0]))  # (W, H)
# rgb = rgb.astype(np.float32)

depth = cv2.imread("depths.png", cv2.IMREAD_UNCHANGED)
if depth.ndim == 3:
    depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
depth = cv2.resize(depth, (target_size[1], target_size[0]))
depth = depth.astype(np.float32)

# --- Normalize and format to NCHW (but ONNX expects input before wrapper processing) ---
rgb_tensor = rgb  # Shape: [H, W, 3]
depth_tensor = depth  # Shape: [H, W]

# --- Run ONNX inference ---
ort_session = ort.InferenceSession("esanet_with_preproc.onnx", providers=["CPUExecutionProvider"])
ort_inputs = {
    "rgb": rgb_tensor,
    "depth": depth_tensor
}
# ort_inputs = {k: v.astype(np.uint8) for k, v in ort_inputs.items()}

ort_outputs = ort_session.run(None, ort_inputs)
pred = ort_outputs[0].astype(np.uint8)  # [H, W]

# --- Colorize the output ---
np.random.seed(28)
colors = np.random.randint(0, 255, size=(21, 3), dtype=np.uint8)
colorize = lambda x: colors[x]

seg_vis = colorize(pred)

# --- Show results ---
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.imshow(rgb.astype(np.uint8))
plt.title("RGB Input")
plt.axis("off")

plt.subplot(1, 2, 2)
plt.imshow(seg_vis)
plt.title("ONNX Runtime Prediction")
plt.axis("off")

plt.tight_layout()
plt.show()
