import cv2
import numpy as np
import random
from imgaug import augmenters as iaa # pip install imgaug

vertex = []
padding = 50
picture_num = 100
label = ['0', '1', '2', '3', '4']

index = int(input("몇 번째 표지판으로 하실? (0~%d) : "%(len(label)-1)))

sign = label[index]
target = cv2.imread("target/" + sign + ".jpg", 1)
background = cv2.imread("background/background.jpg", 1)

def save_vertex(event, x, y, a, b):
    if event == cv2.EVENT_LBUTTONUP:
        cv2.circle(target, (x, y), 2, (255,0,0), -1)
        vertex.append( [y,x] )

cv2.namedWindow("target")
cv2.setMouseCallback("target", save_vertex)

def line_and_point(line, point):
    x0, y0 = line[0][0], line[0][1]
    x1, y1 = line[1][0], line[1][1]
    x2, y2 = point[0], point[1]

    if x0 == x1:
        if x2 >= x0:
            return True
        else:
            return False
    elif y0 == y1:
        if y2 >= y0:
            return True
        else:
            return False
    else:
        m = (y1 - y0) / (x1 - x0)
        y = m * (x2 - x0) + y0

        if y > y2:
            return True
        else:
            return False

def make_area_box_and_img_box(points):
    points = np.array(points)
    min_x = min(points[:,0])
    min_y = min(points[:,1])

    max_x = max(points[:,0])
    max_y = max(points[:,1])

    del_x = max_x - min_x
    del_y = max_y - min_y

    area_box = np.zeros( (del_x + padding * 2, del_y + padding * 2, 3) , dtype= np.uint8)
    img_box = np.zeros( (del_x + padding * 2, del_y + padding * 2, 3) , dtype= np.uint8)
    return img_box, area_box

while True:
    cv2.imshow("target", target)
    if cv2.waitKey(33) == ord('d'):
        break
cv2.destroyAllWindows()
for l in range(picture_num):
    target = cv2.imread("target/" + sign + ".jpg", 1)
    background = cv2.imread("background/background.jpg", 1)
    img, area = make_area_box_and_img_box(vertex)
    vertex = np.array(vertex)
    vertex_num = len(vertex)

    left_top = [min(vertex[:, 0]), min(vertex[:, 1])]
    center = [np.mean(vertex[:, 0]), np.mean(vertex[:, 1])]
    bool_table = []

    for i in range(vertex_num):
        vertex1 = vertex[i - 1]
        vertex2 = vertex[i]
        line = [vertex1, vertex2]
        bool_table.append(line_and_point(line, center))

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            bool_table1 = []
            for k in range(vertex_num):
                vertex1 = vertex[k - 1]
                vertex2 = vertex[k]
                line = [vertex1, vertex2]
                bool_table1.append(line_and_point(line, [left_top[0]+ i, left_top[1] + j]))
            if bool_table == bool_table1:
                 area[padding + i][padding + j] = 1

    for i in range(img.shape[0] - 2 * padding):
        for j in range(img.shape[1] - 2 * padding):
            img[padding + i][padding + j][0] = int(target[left_top[0] + i][left_top[1] + j][:][0].copy())
            img[padding + i][padding + j][1] = int(target[left_top[0] + i][left_top[1] + j][:][1].copy())
            img[padding + i][padding + j][2] = int(target[left_top[0] + i][left_top[1] + j][:][2].copy())

    r = 2 * (random.random() + 0.5)
    t_w, t_h = img.shape[:2]
    b_w, b_h = background.shape[:2]
    t_w, t_h = int(r * t_w), int(r * t_h)
    x, y = int((b_w - 1.5 * t_w) * random.random()), int((b_h - 1.5 * t_h ) * random.random())

    img = cv2.resize(img, (t_w, t_h), interpolation=cv2.INTER_CUBIC)
    area = cv2.resize(area, (t_w, t_h), interpolation=cv2.INTER_CUBIC)

    seq = iaa.Sequential([
        iaa.Affine(rotate=(-45, 45)),
        iaa.Affine(shear=(-45, 45))
        ])
    seq_det = seq.to_deterministic()

    img = seq_det.augment_images([img])[0]
    area = seq_det.augment_images([area])[0]

    for i in range(area.shape[0]):
        for j in range(area.shape[1]):
            if area[i][j].all() == 1:
                background[x + i][y + j] = img[i][j].copy()
    print('Saving merge\\' + sign + '\\image_%d.jpg...' % l)
    cv2.imwrite('merge\\' + sign + '\\image_%d.jpg'%l, background)
    print('Saving merge\\' + sign + '\\image_%d.txt...' % l)
    f = open("merge\\" + sign + "\\image_%d.txt"%l, 'w')
    f.write("%d %.4f %.4f %.4f %.4f"%(index,  (x + padding) / background.shape[0], (y + padding) / background.shape[1], (area.shape[0] -  2 * padding) / background.shape[0], (area.shape[1] -  2 * padding) / background.shape[1]) )
    f.close()