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




class SegformerSegmenter():
    def __init__(self,temperature = 1,model_ckpt = "nvidia/segformer-b4-finetuned-ade-512-512"):
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.feature_extractor = SegformerFeatureExtractor.from_pretrained("nvidia/segformer-b4-finetuned-ade-512-512")
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_ckpt).to(self.device)
        self.model.eval()
        self.temperature = temperature

        self.softmax = nn.Softmax(dim = 1)

        # for idx,new_class in enumerate(self.class_mapping):
        #     class_matrix[idx,new_class] = 1

        # self.cm = torch.from_numpy(class_matrix.astype(np.float32)).to(self.device)
    def set_temperature(self,temperature):
        self.temperature = temperature

    def classify(self,rgb,depth = None,x=None,y = None,temperature = None):
        with torch.no_grad():
            image = Image.fromarray(np.uint8(rgb))
            inputs = self.feature_extractor(images=image, return_tensors="pt")
            print(inputs['pixel_values'].shape)
            outputs = self.model(pixel_values=inputs['pixel_values'].to(self.device))
            logits = outputs.logits
            # print(logits.shape)
            # print(logits)


            # pred = torch.tensordot(logits,self.cm,dims = ([0],[0])).permute((2,0,1)).unsqueeze(0)
            if((x == None) or( y == None)):
                print(image.height, image.width)
                pred = F.interpolate(logits, (image.height,image.width),mode='bilinear')
            else:
                pred = F.interpolate(logits, (x,y),mode='bilinear')

            if(temperature):
                # print('applying temperature scaling')
                pred = self.softmax(pred/temperature)
            else:
                pred = self.softmax(pred/self.temperature)

            pred = torch.argmax(pred,axis = 1)

        return pred.squeeze().detach().cpu().numpy()

    def get_pred_probs(self,rgb,depth = None,x = None,y = None,temperature = None):
        with torch.no_grad():
            image = Image.fromarray(np.uint8(rgb))
            inputs = self.feature_extractor(images=image, return_tensors="pt")
            print(inputs['pixel_values'].shape)
            outputs = self.model(pixel_values=inputs['pixel_values'].to(self.device))
            logits = outputs.logits
            # print(logits.shape)
            # pred = torch.tensordot(logits,self.cm,dims = ([0],[0])).permute((2,0,1)).unsqueeze(0)
            # pred = self.aggregate_logits(logits)
            pred = logits
            # print(pred.shape)
            # pred = logits.unsqueeze(0)

            if(temperature):
                # print('applying temperature scaling')
                pred = self.softmax(pred/temperature)
            else:
                pred = self.softmax(pred/self.temperature)
            if((x == None) or( y == None)):
                pred = F.interpolate(pred, (image.height,image.width),mode='nearest')
            else:
                pred = F.interpolate(pred, (x,y),mode='nearest')
            # print(pred.shape)
        return pred.squeeze().detach().permute((1,2,0)).contiguous().cpu().numpy()


    def get_raw_logits(self,rgb,depth = None,x=None,y = None,temperature = 1):
        with torch.no_grad():
            image = Image.fromarray(np.uint8(rgb))
            inputs = self.feature_extractor(images=image, return_tensors="pt")
            # print(inputs['pixel_values'].shape)
            outputs = self.model(pixel_values=inputs['pixel_values'].to(self.device))
            logits = outputs.logits
            # print(logits.shape)

            if((x == None) or( y == None)):
                pred = F.interpolate(logits, (image.height,image.width),mode='nearest')
            else:
                pred = F.interpolate(logits, (x,y),mode='nearest')
            # print(pred.shape)
        return pred.squeeze().detach().permute((1,2,0)).contiguous().cpu().numpy()





import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers import SegformerForSemanticSegmentation

import torch
import torch.nn as nn
import torch.nn.functional as F


class Segformer_torchnn(nn.Module):
    def __init__(self, model_ckpt="nvidia/segformer-b4-finetuned-ade-512-512", temperature=1.0):
        super().__init__()
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_ckpt)
        self.temperature = temperature
        self.softmax = nn.Softmax(dim=1)

        # imagenet mean and std 
        self.register_buffer("mean", torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))
        self.target_size = (512, 512)

    def forward(self, x):
        orig_h, orig_w = x.shape[2], x.shape[3]

        x = x / 255.0
        x = (x - self.mean.to(x.device)) / self.std.to(x.device)
        x = F.interpolate(x, size=(512, 512), mode='bilinear', align_corners=False)


        logits = self.model(pixel_values=x).logits  # [B, C, 512, 512]
        probs = self.softmax(logits / self.temperature)

        probs = F.interpolate(probs, size=(orig_h, orig_w), mode='bilinear', align_corners=False)
        pred = torch.argmax(probs, dim=1)  # [B, H, W]

        return pred.squeeze().detach().cpu()

segmenter = SegformerSegmenter()
segmenter2 = Segformer_torchnn().eval()
model = Segformer_torchnn().eval()

dummy_input = torch.randint(0, 256, (1, 3, 720, 1280), dtype=torch.float32)

torch.onnx.export(
    model,
    (dummy_input,),
    "segformer_preprocessed.onnx",
    input_names=["rgb"],
    output_names=["probs"],
    dynamic_axes={
        "rgb": {0: "batch", 2: "height", 3: "width"},
        "probs": {0: "height", 1: "width", 2: "classes"},
    },
    opset_version=12
)
print("Exported as segformer_preprocessed.onnx")

import matplotlib.pyplot as plt
import numpy as np
import torch
import cv2

img = cv2.imread("test/scene.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

pred1 = segmenter.classify(img)
print(img.shape)
img_tensor = torch.tensor(img.transpose(2, 0, 1)).unsqueeze(0).float()  
with torch.no_grad():
    pred2 = segmenter2(img_tensor)


ort_session = ort.InferenceSession("segformer_preprocessed.onnx", providers=["CPUExecutionProvider"])
ort_inputs = {"rgb": img_tensor.numpy()}
ort_outputs = ort_session.run(None, ort_inputs)
pred3 = ort_outputs[0].squeeze()  # [H, W] after argmax
print(pred3.shape)
np.random.seed(0)
colors = np.random.randint(0, 255, size=(150, 3), dtype=np.uint8)
colorize = lambda seg: colors[seg]

seg1 = colorize(pred1)
seg2 = colorize(pred2)
seg3 = colorize(pred3)

plt.figure(figsize=(18, 5))
plt.subplot(1, 4, 1)
plt.imshow(img)
plt.title("Original")
plt.axis("off")

plt.subplot(1, 4, 2)
plt.imshow(seg1)
plt.title("segmenter (HF)")
plt.axis("off")

plt.subplot(1, 4, 3)
plt.imshow(seg2)
plt.title("segmenter2 (Wrapper)")
plt.axis("off")

plt.subplot(1, 4, 4)
plt.imshow(seg3)
plt.title("ONNX Runtime")
plt.axis("off")

plt.tight_layout()
plt.show()
