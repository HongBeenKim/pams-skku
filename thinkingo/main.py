import threading

from data_class import Data

from car_platform import CarPlatform
from control import Control
from lane_cam import LaneCam
from lidar import Lidar
from planner import MotionPlanner
from sign_cam import SignCam


def main():
    data = Data()
    platform = CarPlatform('COM6', data)

    platform_thread = threading.Thread(target=platform.main)

    platform_thread.start()


if __name__ == "__main__":
    main()
