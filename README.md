
# Team. SKKU at PAMS
저희는 2018 판교 자율주행 모터쇼(PAMS 2018, Pangyo Autonomous Motor Show)의<br /> 대학생 자동차 융합기술 경진대회 자율주행 부문을 준비하고 출전했던 성균관대학교 팀입니다.

<hr/>

### 대회 개요
* 대회명: PAMS 2018 대학생 자동차 융합기술 경진대회 / 자율주행 부문
* 일시: 2018년 11월 16(금) ~ 17(토)
* 장소: 판교 제2테크노밸리 자율주행쇼런 행사장
* 주최: 경기도
* 주관: 차세대융합기술연구원, KINTEX

### 팀원 (7)
김홍빈 노인호 박주은 박준혁 유지찬 임정한 한일석
### 대회 결과
성균관대학교 HEVEN 팀 4위 (총 4개 팀 중)
### 후기
* [Nov. 20, 2018 thinkingo review](https://www.slideshare.net/ssuser4d0bd1/nov-20-2018-thinkingo-review-123864879?ref=https://www.slideshare.net/ssuser4d0bd1/slideshelf)
* [자율주행 시스템 thinkingo 구조](https://jueun-park.github.io/2018-11-25/thinkingo-system-architecture) `박주은`
* [YOLO v3로 표지판 인식하기](http://localhost:4000/YOLO-Review/) `한일석`

<hr/>

# Project Directory
* [ThinkinGo](https://github.com/HongBeenKim/pams-skku#1-thinkingo)
  * [Multithreading](https://github.com/HongBeenKim/pams-skku#11-multithreading)
    * [Data Space](https://github.com/HongBeenKim/pams-skku#111-data-space)
    * [구현하지 않은 내용](https://github.com/HongBeenKim/pams-skku#112-%EA%B5%AC%ED%98%84%ED%95%98%EC%A7%80-%EC%95%8A%EC%9D%80-%EB%82%B4%EC%9A%A9)
* [Labeling Tool](https://github.com/HongBeenKim/pams-skku#2-labeling-tool)
  * [Cut and Merge](https://github.com/HongBeenKim/pams-skku#21-cut-and-merge)
    * [Warning!!](https://github.com/HongBeenKim/pams-skku#211-warning)
  * [Using `imgaug` Python Module](https://github.com/HongBeenKim/pams-skku#22-imgaug-python-module)
    * [Data Augmentation](https://github.com/HongBeenKim/pams-skku#221-data-augmentation)
* [Data Logging Set](https://github.com/HongBeenKim/pams-skku#3-data-logging-set)
* [Test](https://github.com/HongBeenKim/pams-skku#4-test)

<hr/>

## 1. ThinkinGo

 `thinkingo` 디렉토리에서 저희가 제작한 자율주행 시스템 소스 코드를 관리합니다.
```python
[thinkingo]
data_class.py
subroutine.py
main.py
├ monitoring.py  # 모니터링 시스템
├ car_platform.py  # 차량 플랫폼 통신
│ └ serial_packet.py
├ data_source.py  # cam, lidar stream
├ sign_cam.py  # 표지판 인식
│ └ module.pytorch_yolo.yolo.py
├ planner.py  # 경로 계획
│ ├ lane_cam.py
│ └ parabola.py
└ control.py  # 차량 제어
```
_Thinking Kingo: 생각하는 은행잎, Kingo 는 성균관대학교의 상징인 은행잎을 뜻합니다._
### &nbsp;&nbsp;1.1 Multithreading

&nbsp;&nbsp;&nbsp;&nbsp;각 subroutine은 동일한 데이터 공간에 접근하기 위해 multi threading으로 구현되어 있습니다.

 &nbsp;&nbsp;&nbsp;&nbsp; > ***Subroutines***: `monitoring` `data_source` `car_platform` `sign_cam` `planner` `control`
#### &nbsp;&nbsp;&nbsp;&nbsp;1.1.1 Data Space
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `data_class.py`에 정의된 `class Data` 의 인스턴스를 각 subroutine에게 레퍼런스로 넘겨주어<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 동일한 데이터 공간에 접근할 수 있도록 합니다.

#### &nbsp;&nbsp;&nbsp;&nbsp;1.1.2 구현하지 않은 내용

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; thread join <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; thread lock

<hr/>

## 2. Labeling Tool

 `Labeling Tool` 디렉토리에서는 표지판 데이터셋을 생성합니다.
```text
[Labeling Tool]
background                  # 표지판을 붙여 넣을 배경을 모아놓은 디렉토리
target                      # 학습을 위한 표지판들을 모아놓은 디렉토리
auto_augmentation.py        # 생성된 데이터셋에 적절한 augmentation으로 다양한 데이터셋을 생성
generate_trimmed_target.py  # 표지판을 적절하게 잘라내어 배경의 적절한 위치에 삽입 및 자동 라벨링
```

### &nbsp;&nbsp;2.1 Cut and Merge
&nbsp;&nbsp;&nbsp;&nbsp;`generate_trimmed_target.py` 파일은 `target` 폴더 내 사진을 가져와 필요한 영역을 자르고, <br />
&nbsp;&nbsp;&nbsp;&nbsp;잘린 부분을 `background` 내의 배경사진에 합쳐주며, [YOLO](https://pjreddie.com/darknet/yolo/)에 필요한 라벨정보와 함께 저장됩니다.

#### &nbsp;&nbsp;&nbsp;&nbsp;2.1.1 ***Warning!!***
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ***Target이 볼록 다각형이 아니면 원하는 이미지가 나오지 않습니다.*** <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ***Target이 오목한 모양을 포함하고 있다면, 코드를 수정하여야 할 것 입니다.***

### &nbsp;&nbsp;2.2 `imgaug` Python Module
 &nbsp;&nbsp;&nbsp;&nbsp;생성한 데이터 셋을 [imgaug](https://github.com/aleju/imgaug) 모듈을 사용하여 ***Data Augmentation*** 을 진행하였습니다.

#### &nbsp;&nbsp;&nbsp;&nbsp;2.2.1 Data Augmentation
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ***Data Augmentation*** 은 데이터 셋이 충분하거나 다양하지 않을 때, ***Brightness***, ***Saturation*** 을 변경하거나, <br /> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ***Dropout***, ***Blur*** 처리 혹은 ***Affine transform***, ***padding*** 등을 진행할 수 있습니다.

<hr/>

## 3. Data Logging Set
`data_logging_set` 디렉토리는 데이터 로깅과 관련한 툴 셋이 있습니다. [자세한 설명](https://github.com/HongBeenKim/pams-skku/pull/4)

<hr/>

## 4. Test
`test` 디렉토리는 Test 코드를 위한 디렉토리입니다.
