import sys

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data


class Lidar(Subroutine):
    RADIUS = 500  # 원일 경우 반지름, 사각형일 경우 한 변

    def __init__(self, data: Data, data_source):
        super().__init__(data)
        self.data_source = data_source
        self.data_list = None
        self.frame = None
        self.stop_fg = False

    def main(self):
        """
        데이터 받아서 저장하는 메서드
        """
        while True:
            temp = self.data_source.lidar_data.split(' ')[116:477]
            # 가공 후의 라이다 데이터를 데이터베이스에 업데이트
            self.data.lidar_data_list = [int(item, 16) for item in temp]

            if self.stop_fg is True:
                break

    def stop(self):
        self.stop_fg = True


if __name__ == "__main__":
    # TODO: 필요한 테스트 코드 작성 (아래는 검증되지 않은 테스트 코드)
    import threading
    from dummy_data_source import DummySource

    test_data = Data()
    test_source = DummySource('2018-11-04-17-01-16')
    current_lidar = Lidar(test_data, test_source)
    lidar_test_thread = threading.Thread(target=current_lidar.main)

    lidar_test_thread.start()
