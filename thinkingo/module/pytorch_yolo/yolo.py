from __future__ import division
import torch 
from torch.autograd import Variable
import cv2
import numpy as np
from module.pytorch_yolo.util import *
from module.pytorch_yolo.darknet import Darknet
from module.pytorch_yolo.preprocess import prep_image
import random 


def prep_image(img, inp_dim):
    """
    Prepare image for inputting to the neural network. 
    
    Returns a Variable 
    """

    orig_im = img
    dim = orig_im.shape[1], orig_im.shape[0]
    img = cv2.resize(orig_im, (inp_dim, inp_dim))
    img_ = img[:,:,::-1].transpose((2,0,1)).copy()
    img_ = torch.from_numpy(img_).float().div(255.0).unsqueeze(0)
    return img_, orig_im, dim

def write(x, img):
    c1 = tuple(x[1:3].int())
    c2 = tuple(x[3:5].int())
    cls = int(x[-1])
    label = "{0}".format(classes[cls])
    color = random.choice(colors)
    cv2.rectangle(img, c1, c2,color, 1)
    t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1 , 1)[0]
    c2 = c1[0] + t_size[0] + 3, c1[1] + t_size[1] + 4
    cv2.rectangle(img, c1, c2,color, -1)
    cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225,255,255], 1);
    return img

#path to models
cfgfile = "module/data/yolo-sign.cfg"
weightsfile = "module/data/yolo-sign_12900.weights"
classes = load_classes('module/data/sign.names')

#parameters
confidence = float(0.25)
nms_thesh = float(0.4)
CUDA = torch.cuda.is_available()
num_classes = 8


def init_yolo_sign():
    
    #Build YOLO Model from cfg
    model = Darknet(cfgfile)

    #Load Weight on Memory.
    model.load_weights(weightsfile)
    
    inp_dim = int(model.net_info["height"])

    assert inp_dim % 32 == 0 
    assert inp_dim > 32

    #Load Weight on GPU if CUDA is True
    if CUDA:
        model.cuda()

    #Get Ready for running Model.     
    model.eval()
    return model

def run_yolo_sign(model, frame):

    Data = []
    #Get input size
    inp_dim = int(model.net_info["height"])

    #Convert frame to Image Tensor.
    img, orig_im, dim = prep_image(frame, inp_dim)
    im_dim = torch.FloatTensor(dim).repeat(1,2)

    #Convert Image Tensor to Image CUDA Tesnor if CUDA is True.
    if CUDA:
        im_dim = im_dim.cuda()
        img = img.cuda()

    #Get Result from Model.
    output = model(Variable(img), CUDA)
    output = write_results(output, confidence, num_classes, nms = True, nms_conf = nms_thesh)

    #If Result is None.
    if type(output) == int:
        Datas = [[None, None]]
        return
    #Else Result is Names and percentages.
    else:
        output2 = output.cpu().numpy()
        Datas = []
        
        for i in range(len(output2[:,7])):
            Data = [ classes[ int(output2[:,7][i]) ], output2[:,5][i] ]
            Datas.append( Data )
        
    return Datas
