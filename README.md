# Team. SKKU at PAMS
저희는 2018 판교 자율주행 모터쇼(PAMS 2018, Pangyo Autonomous Motor Show)의 대학생 자동차 융합기술 경진대회 자율주행 부문을 준비하고 출전했던 성균관대학교 팀입니다.

## 대회 개요
* 대회명: PAMS 2018 대학생 자동차 융합기술 경진대회 / 자율주행 부문
* 일시: 2018년 11월 16(금) ~ 17(토)
* 장소: 판교 제2테크노밸리 자율주행쇼런 행사장
* 주최: 경기도
* 주관: 차세대융합기술연구원, KINTEX
## 팀원 (7)
김홍빈 노인호 박주은 박준혁 유지찬 임정한 한일석
## 대회 결과
성균관대학교 HEVEN 팀 4위 (총 4개 팀 중)

# Project Directory
## ThinKingo
`thinkingo` 디렉토리에서 저희가 제작한 자율주행 시스템 소스 코드를 관리합니다.

```text
[thinkingo]
data_class.py
subroutine.py
main.py
├ monitoring.py  # 모니터링 시스템
├ car_platform.py  # 차량 플랫폼 통신
│ └ serial_packet.py
├ sign_cam.py  # 표지판 인식
│ └ module.pytorch_yolo.yolo.py
├ planner.py  # 경로 계획
│ ├ lane_cam.py
│ └ parabola.py
└ control.py  # 차량 제어
```
_Thinking Kingo: 생각하는 은행잎, Kingo 는 성균관대학교의 상징인 은행잎을 뜻합니다._

## Labeling Tool
표지판 데이터셋을 만들기 쉽게 해 주는 라벨링 툴입니다.

## Data Logging Set
데이터 로깅과 관련한 코드들을 모아둔 툴 셋입니다. [자세한 설명](https://github.com/HongBeenKim/pams-skku/pull/4)

## Test
테스트 코드 디렉토리입니다.
