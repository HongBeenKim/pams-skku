"""
SubRoutine 클래스들은 다음과 같은 구조를 갖습니다.
해당 루틴: lane cam, sign cam, LiDAR, planner, control, platform communicator
"""
from data_class import Data


class Subroutine(object):
    def __init__(self, data: Data):
        """
        :param data:
        self.data 인자는 data_class.py에 정의된 Data 인스턴스를 인자로 받습니다.
        각 subroutine 을 수행하기 위해 필요한 값이나
        subroutine 을 수행한 뒤 결과 값을
        data 인스턴스에 접근해서 얻고 수정합니다.
        """
        self.data = data

    def main(self):
        """
        각 subroutine 이 한 루프에 해야 할 모든 행동을 이 함수에 정의합니다.
        이 함수는 main.py 에서 thread 로 실행됩니다.

        :return:
        리턴 값은 없습니다.
        """
        pass
