import cv2
import numpy as np

camera_matrix_L = np.array([[474.383699, 0, 404.369647], [0, 478.128447, 212.932297], [0, 0, 1]])
camera_matrix_R = np.array([[473.334870, 0, 386.312394], [0, 476.881433, 201.662339], [0, 0, 1]])
distortion_coefficients_L = np.array([0.164159, -0.193892, -0.002730, -0.001859])
distortion_coefficients_R = np.array([0.116554, -0.155379, -0.001045, -0.001512])

cam = cv2.VideoCapture(2)
cam.set(3, 800)  # CV_CAP_PROP_FRAME_WIDTH
cam.set(4, 448)  # CV_CAP_PROP_FRAME_HEIGHT

while True:
    ret_val, img = cam.read()  # 캠 이미지 불러오기
    undistorted = cv2.undistort(img, camera_matrix_L, distortion_coefficients_L, None, None)[5:443, 5:795]

    cv2.imshow("Cam Viewer", undistorted)  # 불러온 이미지 출력하기
    if cv2.waitKey(1) == 27:
        break  # esc to quit
