# -*- coding: utf-8 -*-
import os
import imgaug as ia
from imgaug import augmenters as iaa
import scipy
import shutil

path = 'C:\\Users\\HEVEN\\Desktop\\background_ilseok\\background'

augmentation_path = 'C:\\Users\\HEVEN\\Desktop\\background_ilseok\\augmentation'

for root, dirs, files in os.walk(path):
    for file in files:
        if file.endswith(".jpg"):
            for light in range(10):
                jpg_path = path + "\\" + file
                txt_path = path + "\\" + file[:-4] + ".txt"

                image = scipy.misc.imread(jpg_path, flatten=False, mode="RGB")

                if light < 5:
                    seq = iaa.Sequential([
                            iaa.Multiply( 1 - 0.15 * (light) )
                    ])
                else:
                    seq = iaa.Sequential([
                            iaa.Multiply( 1 + 0.3 * (light-4) )
                    ])
                    

                image_aug = seq.augment_images([image])[0]

                new_jpg_path = augmentation_path + "\\new_" + file[:-4] + "_%d"%light + ".jpg"
                new_txt_path = augmentation_path + "\\new_" + file[:-4] + "_%d"%light + ".txt"

                scipy.misc.imsave(new_jpg_path, image_aug)
                shutil.copy2(txt_path, new_txt_path)            
            


