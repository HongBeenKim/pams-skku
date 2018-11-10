import threading
import sys

sys.path.append(".")
from data_class import Data

from car_platform import CarPlatform
from lane_cam import LaneCam
from lidar import Lidar
from sign_cam import SignCam
from planner import MotionPlanner
from control import Control


def main():
    data = Data()
    platform = CarPlatform('COM6', data)
    sign_cam = SignCam(data)

    platform_thread = threading.Thread(target=platform.main)
    sign_cam_thread = threading.Thread(target=sign_cam.main)

    platform_thread.start()
    sign_cam_thread.start()


if __name__ == "__main__":
    main()
