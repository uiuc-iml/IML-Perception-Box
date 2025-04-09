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


class FineTunedTSegmenter():
    def __init__(self,temperature = 1,model_ckpt = "../semanticmapping/finetunedsegmenter/Semantic-3D-Mapping-Uncertainty-Calibration/segmentation_model_checkpoints/Segformer"):
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
            outputs = self.model(pixel_values=inputs['pixel_values'].to(self.device))
            logits = outputs.logits


            # pred = torch.tensordot(logits,self.cm,dims = ([0],[0])).permute((2,0,1)).unsqueeze(0)
            if((x == None) or( y == None)):
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
            # print(inputs['pixel_values'].shape)
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
        
        return pred.squeeze().detach().permute((1,2,0)).contiguous().cpu().numpy()


    def get_raw_logits(self,rgb,depth = None,x=None,y = None,temperature = 1):
        with torch.no_grad():
            image = Image.fromarray(np.uint8(rgb))
            inputs = self.feature_extractor(images=image, return_tensors="pt")
            # print(inputs['pixel_values'].shape)
            outputs = self.model(pixel_values=inputs['pixel_values'].to(self.device))
            logits = outputs.logits
            if((x == None) or( y == None)):
                pred = F.interpolate(logits, (image.height,image.width),mode='nearest')
            else:
                pred = F.interpolate(logits, (x,y),mode='nearest')
        return pred.squeeze().detach().permute((1,2,0)).contiguous().cpu().numpy()


class FineTunedSegformer_torchnn(nn.Module):
    def __init__(self, model_ckpt="../semanticmapping/finetunedsegmenter/Semantic-3D-Mapping-Uncertainty-Calibration/segmentation_model_checkpoints/Segformer", temperature=1.0):
        super().__init__()
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_ckpt)
        self.temperature = temperature
        self.softmax = nn.Softmax(dim=1)

        self.register_buffer("mean", torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))
        self.target_size = (512, 512)

    def forward(self, x):
        orig_h, orig_w = x.shape[2], x.shape[3]

        x = x / 255.0
        x = (x - self.mean.to(x.device)) / self.std.to(x.device)
        x = F.interpolate(x, size=self.target_size, mode='bilinear', align_corners=False)

        logits = self.model(pixel_values=x).logits  
        probs = self.softmax(logits / self.temperature)  

        probs = F.interpolate(probs, size=(orig_h, orig_w), mode='bilinear', align_corners=False)

        return torch.argmax(probs, dim=1).squeeze(0).contiguous()  # [H, W]

segmenter = FineTunedTSegmenter()
model = FineTunedSegformer_torchnn().eval()

dummy_input = torch.randint(0, 255, (1, 3, 720, 1280), dtype=torch.float32)

torch.onnx.export(
    model,
    (dummy_input,),
    "fine_tuned_segformer.onnx",
    input_names=["rgb"],
    output_names=["segmentation"],
    dynamic_axes={
        "rgb": {0: "batch", 2: "height", 3: "width"},
        "segmentation": {0: "height", 1: "width", 2:"probs"}
    },
    opset_version=12
)

img = cv2.imread("test/scene3.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_tensor = torch.tensor(img.transpose(2, 0, 1)).unsqueeze(0).float()

session = ort.InferenceSession("fine_tuned_segformer.onnx", providers=["CPUExecutionProvider"])
onnx_output = session.run(None, {"rgb": img_tensor.numpy()})[0]

with torch.no_grad():
    torch_output = model(img_tensor).cpu().numpy()

reference_output = segmenter.classify(img)  

np.random.seed(0)
colors = np.random.randint(0, 255, size=(21, 3), dtype=np.uint8)
colorize = lambda seg: colors[seg]

seg1 = colorize(reference_output)
seg2 = colorize(torch_output)
seg3 = colorize(onnx_output.astype(np.uint8))

plt.figure(figsize=(18, 6))
plt.subplot(1, 4, 1)
plt.imshow(img)
plt.title("Original")
plt.axis("off")

plt.subplot(1, 4, 2)
plt.imshow(seg1)
plt.title("original segmenter")
plt.axis("off")

plt.subplot(1, 4, 3)
plt.imshow(seg2)
plt.title("PyTorch wrapper Model")
plt.axis("off")

plt.subplot(1, 4, 4)
plt.imshow(seg3)
plt.title("ONNX Model")
plt.axis("off")

plt.tight_layout()
plt.show()




