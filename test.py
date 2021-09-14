import numpy as np
import os
import cv2
import sys
sys.path.append("..")

# Keras / TensorFlow
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'
from keras.models import load_model
from CNN_python_Module.layers import BilinearUpSampling2D
from CNN_python_Module.utils import predict

max_depth = 5 #4.5
focal_scale = 1.0
nyu_focal = 518.8579
our_focal = 533.6423766 #353
tum_focal1 = 517.3
tum_focal2 = 520.9
tum_focal3 = 535.4

our_scale = 1000
TUM_scale = 13107

# Argument Parser
model_name = 'NoDAugmentation'
model = './nyu.h5' 
MINConv = 32



# Custom object needed for inference and training
custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': None}

print('Loading model...')

# Load model into GPU / CPU
model = load_model(model, custom_objects=custom_objects, compile=False)

print('\nModel loaded ({0}).'.format(model))


def execute(rgb_image):
    global result
    x = np.clip(rgb_image.astype(float) / 255, 0, 1)
    h, w, ch = x.shape
    inputs = np.stack(x, axis=0)

    outputs_tensor = predict(model, inputs, minDepth=500, maxDepth=8000, batch_size=1) 
    outputs = outputs_tensor[0, :, :, 0]
    scaledDepth = outputs * 8.0 * (tum_focal2 / nyu_focal)
    resizedDepth = cv2.resize(scaledDepth, (w, h), interpolation=cv2.INTER_NEAREST)
    result = resizedDepth
"""  
  return result
if __name__ == '__main__':
    im = execute(cv2.imread('./test.png'))
    im = 255 * im / np.amax(im)
    cv2.imshow('test', cv2.applyColorMap(np.uint8(im), cv2.COLORMAP_JET))
    cv2.waitKey(0) 

"""
