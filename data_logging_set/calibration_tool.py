import cv2
import numpy as np

camera_matrix_L = np.array([[474.383699, 0, 404.369647], [0, 478.128447, 212.932297], [0, 0, 1]])
camera_matrix_R = np.array([[473.334870, 0, 386.312394], [0, 476.881433, 201.662339], [0, 0, 1]])
distortion_coefficients_L = np.array([0.164159, -0.193892, -0.002730, -0.001859])
distortion_coefficients_R = np.array([0.116554, -0.155379, -0.001045, -0.001512])

cam_left = cv2.VideoCapture(0)
cam_right = cv2.VideoCapture(1)

cam_left.set(3, 800)  # CV_CAP_PROP_FRAME_WIDTH
cam_left.set(4, 448)  # CV_CAP_PROP_FRAME_HEIGHT
cam_right.set(3, 800)
cam_right.set(4, 448)

cv2.imshow("left", np.zeros((428, 780), np.uint8))
cv2.imshow("right", np.zeros((428, 780), np.uint8))
cv2.moveWindow("left", 0, 0)
cv2.moveWindow("right", 780, 0)

while True:
    _, left_frame = cam_left.read()  # 캠 이미지 불러오기
    _, right_frame = cam_right.read()
    undistorted_left = cv2.undistort(left_frame, camera_matrix_L, distortion_coefficients_L, None, None)[10:438, 10:790]
    undistorted_right = cv2.undistort(right_frame, camera_matrix_R, distortion_coefficients_R, None, None)[10:438, 10:790]

    cv2.imshow("left", undistorted_left)
    cv2.imshow("right", undistorted_right)

    if cv2.waitKey(1) == 27:
        break  # esc to quit
