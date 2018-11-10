import threading
import sys

sys.path.append(".")
from data_source import Source
from data_class import Data

from car_platform import CarPlatform
from lane_cam import LaneCam
from sign_cam import SignCam
from planner import MotionPlanner
from control import Control


def main():
    database = Data()

    data_source = Source(database)
    data_source_thread = threading.Thread(target=data_source.main)
    data_source_thread.start()

    platform = CarPlatform('COM6', database)
    sign_cam = SignCam(database)

    platform_thread = threading.Thread(target=platform.main)
    sign_cam_thread = threading.Thread(target=sign_cam.main)

    platform_thread.start()
    sign_cam_thread.start()


if __name__ == "__main__":
    main()
