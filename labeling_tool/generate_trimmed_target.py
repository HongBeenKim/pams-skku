import cv2
import numpy as np
import random
import os
from imgaug import augmenters as iaa  # pip install imgaug

vertex = []
padding = 50  # 잘리는 거 방지 간격
label = ['1', '2', '3', '4', '5', '6', '7', '8', '9']
lightness = 10  # 명도 10가지
size = 3  # r의 크기
back_num = 0  # 백그라운드 사진 카운트
final_vertex = []

index = int(input("몇 번째 표지판으로 하실? (1~%d) : " % (len(label))))

sign = label[index - 1]
target = cv2.imread("target/" + sign + ".jpg", 1)


def save_vertex(event, x, y, a, b):
    if event == cv2.EVENT_LBUTTONUP:
        cv2.circle(target, (x, y), 2, (255, 0, 0), -1)
        vertex.append([y, x])


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
    min_x = min(points[:, 0])
    min_y = min(points[:, 1])

    max_x = max(points[:, 0])
    max_y = max(points[:, 1])

    del_x = max_x - min_x
    del_y = max_y - min_y

    area_box = np.zeros((del_x + padding * 2, del_y + padding * 2, 3), dtype=np.uint8)
    img_box = np.zeros((del_x + padding * 2, del_y + padding * 2, 3), dtype=np.uint8)
    return img_box, area_box


while True:
    cv2.imshow("target", target)
    if cv2.waitKey(33) == ord('d'):
        break
cv2.destroyAllWindows()

# 백그라운드 폴더에 있는 배경사진 수만큼 반복
for root, dirs, files in os.walk(os.getcwd() + '\\background'):
    for file in files:
        r = 1.2
        # r의 크기를 증가. 위의 r값이 초기값이고 밑에 r을 적절히 더해줌
        for l in range(size):
            target = cv2.imread("target/" + sign + ".jpg", 1)
            background = cv2.imread("background/background_%d.jpg" % back_num, 1)

            img, area = make_area_box_and_img_box(vertex)
            vertex = np.array(vertex)
            vertex_num = len(vertex)

            left_top = [min(vertex[:, 0]), min(vertex[:, 1])]
            center = [np.mean(vertex[:, 0]), np.mean(vertex[:, 1])]
            bool_table = []

            final_vertex = []

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
                        bool_table1.append(line_and_point(line, [left_top[0] + i, left_top[1] + j]))
                    if bool_table == bool_table1:
                        area[padding + i][padding + j] = 1

            for i in range(img.shape[0] - 2 * padding):
                for j in range(img.shape[1] - 2 * padding):
                    img[padding + i][padding + j][0] = int(target[left_top[0] + i][left_top[1] + j][:][0].copy())
                    img[padding + i][padding + j][1] = int(target[left_top[0] + i][left_top[1] + j][:][1].copy())
                    img[padding + i][padding + j][2] = int(target[left_top[0] + i][left_top[1] + j][:][2].copy())

            t_h_b, t_w_b = img.shape[:2]  # 이미지 사진의 높이와 폭 크기 (패딩 포함되어있음)
            b_h, b_w = background.shape[:2]  # 백그라운드 사진의 높이와 폭 크기

            t_w_a, t_h_a = int(r * t_w_b), int(r * t_h_b)  # 이미지 사진의 높이와 폭 크기를 r배만큼 확장

            if index == 1 or 2 or 3 or 4 or 8 or 9:
                x, y = int((random.random()) * (b_w / 2 - t_w_a) + b_w / 2), int(
                    (random.random() - 1) * (t_h_a) + b_h / 2)
            elif index == 5 or 6 or 7:
                x, y = int((random.random() - 1) * (t_w_a) + b_w / 2), int(
                    (random.random() - 1) * (t_h_a) + b_h / 2)

            img = cv2.resize(img, (t_w_a, t_h_a), interpolation=cv2.INTER_CUBIC)
            area = cv2.resize(area, (t_w_a, t_h_a), interpolation=cv2.INTER_CUBIC)

            for i in range(t_h_a):
                for j in range(t_w_a):
                    if area[i][j].all() == 1:
                        # 백그라운드 사진 범위를 넘어가는거는 안찍음
                        if (y + i) >= b_h or (x + j) >= b_w or (y + i) < 0 or (x + j) < 0:
                            continue
                        background[y + i][x + j] = img[i][j].copy()
                        final_vertex.append([y + i, x + j])

            final_points = np.array(final_vertex)

            final_min_x = min(final_points[:, 1])
            final_min_y = min(final_points[:, 0])

            final_max_x = max(final_points[:, 1])
            final_max_y = max(final_points[:, 0])

            final_del_x = final_max_x - final_min_x
            final_del_y = final_max_y - final_min_y

            x_width = final_del_x / background.shape[1]
            y_width = final_del_y / background.shape[0]
            x_center = final_min_x / background.shape[1] + x_width / 2
            y_center = final_min_y / background.shape[0] + y_width / 2
            
            # 밝기를 어둡게 5가지, 밝게 5가지 총 10가지 변형
            for m in range(lightness):
                if m < 5:
                    seq = iaa.Sequential([
                        iaa.Multiply(1 - 0.1 * (m))
                    ])
                else:
                    seq = iaa.Sequential([
                        iaa.Multiply(1 + 0.15 * (m - 4))
                    ])
                seq_det = seq.to_deterministic()
                background = seq_det.augment_images([background])[0]

                # 합성한 사진 merge폴더에 사진 레이블에 맞게 저장.
                print('Saving merge\\' + sign + '\\image_%d.%d%d.jpg ...' % (
                    back_num, l, m))
                cv2.imwrite('merge\\' + sign + '\\image_%d.%d%d.jpg' % (back_num, l, m), background)
                print('Saving merge\\' + sign + '\\image_%d.%d%d.txt...' % (back_num, l, m))

                f = open("merge\\" + sign + "\\image_%d.%d%d.txt" % (back_num, l, m), 'w')
                f.write("%d %.4f %.4f %.4f %.4f" % (index, x_center, y_center, x_width, y_width))
                f.close()
            r += 0.5
        back_num += 1
