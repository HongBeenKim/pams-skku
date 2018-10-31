"""
라이다와 캠 데이터를 동시에 저장하는 코드
김홍빈
2018-10-31
"""
import threading
import cv2
import socket
import time

# ---------------------------- Configuration Values ---------------------------------
LIDAR_IP = '169.254.248.220'
LIDAR_PORT = 2111
LIDAR_BUF = 57600

left_cam_size = (800, 448)
right_cam_size = (800, 448)
sign_cam_size = (800, 448)

left_cam_num = 1
right_cam_num = 2
sign_cam_num = 3
# -----------------------------------------------------------------------------------

def main():
    data = [None, None, None, None, False]  # 순서대로 left frame, right frame, lidar frame, stop flag
    camThreadL = threading.Thread(target=cam, args=(0, left_cam_num, *left_cam_size, data))
    camThreadR = threading.Thread(target=cam, args=(1, right_cam_num, *right_cam_size, data))
    camThreadS = threading.Thread(target=cam, args=(2, sign_cam_num, *sign_cam_size, data))
    lidarThread = threading.Thread(target=lidar, args=(data,))

    lidarThread.start()
    camThreadL.start()
    camThreadR.start()
    camThreadS.start()

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')

    specific_time = time.localtime()
    timeLabel = str(specific_time.tm_hour) + "+" + str(specific_time.tm_min) + "+" + str(specific_time.tm_sec)
    writerL = cv2.VideoWriter("c:\\log_data\\leftcam\\" + timeLabel + ".avi", fourcc, 60, left_cam_size)
    writerR = cv2.VideoWriter("c:\\log_data\\rightcam\\" + timeLabel + ".avi", fourcc, 60, right_cam_size)
    writerS = cv2.VideoWriter("c:\\log_data\\signcam\\" + timeLabel + ".avi", fourcc, 60, sign_cam_size)
    lidar_fd = open("c:\\log_data\\lidar\\" + timeLabel + ".txt", 'w')

    while True:
        # 라이다, 캠 1, 2, 3 저장
        if data[0] is not None and data[1] is not None and data[2] is not None:
            cv2.imshow("left", data[0])
            cv2.imshow("right", data[1])
            cv2.imshow("sign", data[2])
            writerL.write(data[0])
            writerR.write(data[1])
            writerS.write(data[2])
            lidar_fd.write(data[3])
        if cv2.waitKey(1) & 0xff == ord(' '): break

    data[4] = True
    writerL.release()
    writerR.release()
    writerS.release()
    lidar_fd.close()


def lidar(data_set: list):
    INIT_MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)
    sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_lidar.connect((LIDAR_IP, LIDAR_PORT))
    sock_lidar.send(str.encode(INIT_MESG))

    while True:
        lidar_data = str(sock_lidar.recv(LIDAR_BUF))
        if lidar_data.__contains__('sEA'): continue
        data_set[3] = lidar_data

        if data_set[4]: break


def cam(arg_num: int, cam_num: int, width: int, height: int, data_set: list):
    cap = cv2.VideoCapture(cam_num)
    cap.set(3, width)
    cap.set(4, height)

    while True:
        _, frame = cap.read()
        data_set[arg_num] = frame

        if data_set[4]: break


if __name__ == "__main__":
    main()
