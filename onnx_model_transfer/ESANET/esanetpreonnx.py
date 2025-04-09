import torch
import sys
import pickle
from collections import OrderedDict
import cv2
import numpy as np
import torch
import matplotlib.pyplot as plt

from segmentation_model_loader import FineTunedESANet

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
# Add ESANet to path
sys.path.append('../external_dependencies/ESANet')
import torch
import torch.nn as nn
import torch.nn.functional as F
import pickle
from src.build_model import build_model
from src.models.model import Upsample
from src.prepare_data import prepare_data
import cv2

class ESANetWrapperWithPreprocessing(nn.Module):
    def __init__(self, model_ckpt='../segmentation_model_checkpoints/ESANet/model.ckpt', args_path='../external_dependencies/ESANet/args.p', temperature=1.0):
        super().__init__()

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        args = pickle.load(open(args_path, 'rb'))
        self.model, _ = build_model(args, n_classes=40)
        self.model.eval()
        for param in self.model.parameters():
            param.requires_grad = False
        # decoder_head = model.decode_head
        self.model.decoder.conv_out = nn.Conv2d(
            128, 21, kernel_size=(3, 3), stride=(1, 1), padding=(1, 1))
        self.model.decoder.conv_out.requires_grad = False
        self.model.decoder.upsample1 = Upsample(mode='nearest', channels=21)
        self.model.decoder.upsample2 = Upsample(mode='nearest', channels=21)
        self.checkpoint = '../segmentation_model_checkpoints/ESANet'
        self.temperature = temperature
        new_state_dict = self.get_clean_state_dict()
        self.model.load_state_dict(new_state_dict)
        for param in self.model.parameters():
            param.requires_grad = False
        self.model.to(self.device)
        self.dataset, self.preprocessor = prepare_data(args, with_input_orig=True)

        # Precomputed statistics
        self.depth_mean_val = 2841.9494
        self.depth_std_val = 1417.2594

        # ImageNet mean/std
        self.register_buffer("rgb_mean", torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer("rgb_std", torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))
        self.register_buffer("depth_mean", torch.tensor([self.depth_mean_val]).view(1, 1, 1, 1))
        self.register_buffer("depth_std", torch.tensor([self.depth_std_val]).view(1, 1, 1, 1))

    def get_clean_state_dict(self):
        params_dict = torch.load(self.checkpoint+'/model.ckpt', map_location="cpu")
        state = params_dict['state_dict']
        new_state_dict = OrderedDict()
        for param in state.keys():
            prefix,new_param = param.split('.',1)
            if(prefix != 'criterion'):
                new_state_dict.update({new_param:state[param]})
        return new_state_dict

    def forward(self, rgb, depth):
        # Inputs: rgb [1, 3, H, W], depth [1, 1, H, W], dtype=float32
        # rgb = torch.from_numpy(rgb)
        # depth = torch.from_numpy(depth)
        # rgb = rgb / 255
        # rgb = (rgb - self.rgb_mean.to(rgb.device)) / self.rgb_std.to(rgb.device)
        # depth_mask = (depth == 0)
        # depth = (depth - self.depth_mean.to(depth.device)) / self.depth_std.to(depth.device)
        # depth[depth_mask] = 0
        # sample = self.preprocessor({'image': rgb, 'depth': depth})

        # # add batch axis and copy to device
        # image = sample['image'][None].to(self.device)
        # depth = sample['depth'][None].to(self.device)
        rgb = rgb.float() / 255.0  # [H, W, 3]

        # Normalize RGB with ImageNet stats
        mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 1, 3)
        std = torch.tensor([0.229, 0.224, 0.225]).view(1, 1, 3)
        rgb = (rgb - mean) / std  # [H, W, 3]
        
        depth_mean = 2841.94941272766
        depth_std = 1417.2594281672277
        

        depth_mask = (depth == 0)
        depth = (depth - depth_mean) / depth_std
        depth[depth_mask] = 0.0  # Reset invalid depth

        # Permute and reshape
        rgb = rgb.permute(2, 0, 1).unsqueeze(0)      # [1, 3, H, W]
        depth = depth.unsqueeze(0).unsqueeze(0)      # [1, 1, H, W]


        pred = self.model(rgb, depth)
        
        pred = F.interpolate(pred, (pred.shape[2],pred.shape[3]),mode='nearest')
        pred = (torch.argmax(pred, dim=1)).squeeze()
        return pred


# Example usage
if __name__ == "__main__":
    model = ESANetWrapperWithPreprocessing().eval()
    target_size = (480, 640)  

    rgb = cv2.imread("colors.png")
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (target_size[1], target_size[0]))  # cv2 uses (W, H)
    depth = cv2.imread("depths.png", cv2.IMREAD_UNCHANGED)
    
    depth = cv2.resize(depth, (640, 480))  # (W, H)
    rgb_tensor = torch.tensor(rgb)
    if len(depth.shape) == 3:
        depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY) 
    depth = depth.astype(np.float32)  
    depth_tensor = torch.tensor(depth)
    segmenter = FineTunedESANet()
    seg1 = segmenter.classify(rgb, depth)
    print(seg1.shape)
    print(seg1[120:348])
    with torch.no_grad():
        out = model(rgb_tensor, depth_tensor)
        print("Output shape:", out.shape)

    # Export
    torch.onnx.export(
        model,
        (rgb_tensor, depth_tensor),
        "esanet_with_preproc.onnx",
        input_names=["rgb", "depth"],
        output_names=["segmentation"],
        opset_version=12
    )
    

    # # # Wrapper inference
    wrapper = ESANetWrapperWithPreprocessing().eval()
    with torch.no_grad():
        probs = (wrapper(rgb_tensor, depth_tensor))
        print(probs.shape)
    seg2 = (probs).cpu().numpy().squeeze().astype(np.uint8)
    print(seg2[0,8:90])
    print(seg2[0,0])
    # # Colorize
    np.random.seed(28)
    colors = np.random.randint(0, 255, size=(21, 3), dtype=np.uint8)
    colorize = lambda seg: colors[seg]

    seg1_vis = colorize(seg1)
    seg2_vis = colorize(seg2)

    # Plot
    plt.figure(figsize=(15, 5))
    plt.subplot(1, 3, 1)
    plt.imshow(rgb)
    plt.title("Original")
    plt.axis("off")

    plt.subplot(1, 3, 2)
    plt.imshow(seg1_vis)
    plt.title("FineTunedESANet")
    plt.axis("off")

    plt.subplot(1, 3, 3)
    plt.imshow(seg2_vis)
    plt.title("ESANetWrapper")
    plt.axis("off")

    plt.tight_layout()
    plt.show()

