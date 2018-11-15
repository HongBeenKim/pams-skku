"""
sign_cam.py 가 개발이 완료되지 않았을 때, 나머지 프로그램 테스트를 위한 코드
https://github.com/Jueun-Park/HEVEN_AutonomousCar_2018/blob/master/src/key_cam.py
김진웅 (2018-05)
"""

import keyboard
from subroutine import Subroutine
from data_class import Data


class KeyCam(Subroutine):
    def __init__(self, data: Data):
        super().__init__(data)
        self.data.current_mode = 0
        keyboard.on_press(self.key_look)

    def key_look(self, e):
        c = e.name
        if c == '0':
            self.data.current_mode = 0
        elif c == '1':
            self.data.current_mode = 1
        elif c == '2':
            self.data.current_mode = 2
        elif c == '3':
            self.data.current_mode = 3
        elif c == '4':
            self.data.current_mode = 4
        elif c == '5':
            self.data.current_mode = 5


if __name__ == "__main__":
    test_data = Data()
    k = KeyCam(test_data)
    keyboard.hook(k.key_look)
    while True:
        print(test_data.current_mode)
