import matplotlib.pyplot as plt
import numpy as np
import torch
import cv2
from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation
import torch
import onnx
import onnxruntime as ort
import numpy as np
import matplotlib.pyplot as plt
import cv2
import json
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from transformers import SegformerFeatureExtractor, SegformerForSemanticSegmentation, MaskFormerFeatureExtractor, MaskFormerForInstanceSegmentation
# import nvidia_smi
import os

img = cv2.imread("test/scene.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

img_tensor = torch.tensor(img.transpose(2, 0, 1)).unsqueeze(0).float()  


ort_session = ort.InferenceSession("segformer_preprocessed.onnx", providers=["CPUExecutionProvider"])
ort_inputs = {"rgb": img_tensor.numpy()}
ort_outputs = ort_session.run(None, ort_inputs)
pred3 = ort_outputs[0].squeeze()  # [H, W] after argmax
print(pred3.shape)
np.random.seed(0)
colors = np.random.randint(0, 255, size=(150, 3), dtype=np.uint8)
colorize = lambda seg: colors[seg]

seg3 = colorize(pred3)

plt.figure(figsize=(18, 5))
plt.subplot(1, 2, 1)
plt.imshow(img)
plt.title("Original")
plt.axis("off")


plt.subplot(1, 2, 2)
plt.imshow(seg3)
plt.title("ONNX Runtime")
plt.axis("off")

plt.tight_layout()
plt.show()
