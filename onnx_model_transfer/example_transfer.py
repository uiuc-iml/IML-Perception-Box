import xmlrpc.client
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
# connect to the server
server = xmlrpc.client.ServerProxy("http://localhost:5001") # you may need to change this

with open("ESANET/esanet_with_preproc.onnx", "rb") as f:
    model_data = f.read()
time1 = time.time()
# send model as a binary using load_segmentation_model()
response = server.load_segmentation_model(xmlrpc.client.Binary(model_data))
time2 = time.time()
print(time2-time1)
print("Model load response:", response)
