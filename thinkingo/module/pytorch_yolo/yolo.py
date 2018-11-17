from __future__ import division
import torch
from torch.autograd import Variable
import cv2
import numpy as np
from module.pytorch_yolo.util import *
from module.pytorch_yolo.darknet import Darknet
from module.pytorch_yolo.preprocess import prep_image
import random
import pickle as pkl
import numpy as np

colors = pkl.load(open("module/pytorch_yolo/pallete", "rb"))
color = random.choice(colors)


def write(x, img):
    c1 = tuple(x[1:3].int())
    c2 = tuple(x[3:5].int())
    cls = int(x[-1])
    label = "{0}".format(classes[cls])
    cv2.rectangle(img, c1, c2, color, 1)
    t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
    c2 = c1[0] + t_size[0] + 3, c1[1] + t_size[1] + 4
    cv2.rectangle(img, c1, c2, color, -1)
    cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225, 255, 255], 1);
    return img


def prep_image(img, inp_dim):
    """
    Prepare image for inputting to the neural network. 
    
    Returns a Variable 
    """

    orig_im = img
    dim = orig_im.shape[1], orig_im.shape[0]
    img = cv2.resize(orig_im, (inp_dim, inp_dim))
    img_ = img[:, :, ::-1].transpose((2, 0, 1)).copy()
    img_ = torch.from_numpy(img_).float().div(255.0).unsqueeze(0)
    return img_, orig_im, dim


# path to models
cfgfile = "module/data/yolo-sign.cfg"
weightsfile = "module/data/yolo-sign_16300.weights"
#weightsfile = "module/data/yolo-sign_17200.weights"
classes = load_classes('module/data/sign.names')

# parameters
confidence = float(0.1)

nms_thesh = float(0.4)
CUDA = torch.cuda.is_available()
num_classes = 9


def init_yolo_sign():
    # Build YOLO Model from cfg
    model = Darknet(cfgfile)

    # Load Weight on Memory.
    model.load_weights(weightsfile)

    inp_dim = int(model.net_info["height"])

    assert inp_dim % 32 == 0
    assert inp_dim > 32

    # Load Weight on GPU if CUDA is True
    if CUDA:
        model.cuda()

    # Get Ready for running Model.
    model.eval()
    return model


def run_yolo_sign(model, frame, SHOW):
    Data = []
    # Get input size
    inp_dim = int(model.net_info["height"])

    # Convert frame to Image Tensor.
    img, orig_im, dim = prep_image(frame, inp_dim)
    im_dim = torch.FloatTensor(dim).repeat(1, 2)

    # Convert Image Tensor to Image CUDA Tesnor if CUDA is True.
    if CUDA:
        im_dim = im_dim.cuda()
        img = img.cuda()

    # Get Result from Model.
    output = model(Variable(img), CUDA)
    output = write_results(output, confidence, num_classes, nms=True, nms_conf=nms_thesh)

    # If Result is None.
    if type(output) == int:
        Sign = [0, 1]
        Traffic = [10, 1]
        Parking = [11, 1]
        return Parking[0], Traffic[0], Sign[0], orig_im
    # Else Result is Names and percentages.
    else:
        output2 = output.cpu().numpy()
        Datas = []
        Signs = []
        Traffics = []
        Parkings = []

        for i in range(len(output2[:, 7])):
            Data = [int(output2[:, 7][i]), output2[:, 5][i]]
            Datas.append(Data)

        for Data in Datas:
            if (Data[0] == 8 or Data[0] == 9) and (Data[1] > 0.75):
                Traffics.append(Data)
            if (Data[0] == 6 or Data[0] == 7) and (Data[1] > 0.75):
                Parkings.append(Data)
            elif (Data[0] == 1 or Data[0] == 2 or Data[0] == 3 or Data[0] == 4 or Data[0] == 5) and (Data[1] > 0.85):
                Signs.append(Data)

        if len(Signs) == 0:
            Sign = [0, 1]
        else:
            Signs = np.array(Signs)
            min_index_s = np.argmin(Signs[:, 0])
            Sign = Signs[min_index_s]

        if len(Traffics) == 0:
            Traffic = [10, 1]
        else:
            Traffics = np.array(Traffics)
            min_index_t = np.argmin(Traffics[:, 0])
            Traffic = Traffics[min_index_t]

        if len(Parkings) == 0:
            Parking = [11, 1]
        else:
            Parkings = np.array(Parkings)
            min_index_p = np.argmin(Parkings[:, 0])
            Parking = Parkings[min_index_p]

        if SHOW:
            output[:, 1:5] = torch.clamp(output[:, 1:5], 0.0, float(inp_dim)) / inp_dim
            output[:, [1, 3]] *= frame.shape[1]
            output[:, [2, 4]] *= frame.shape[0]
            list(map(lambda x: write(x, orig_im), output))

    return int(Parking[0]), int(Traffic[0]), int(Sign[0]), orig_im
