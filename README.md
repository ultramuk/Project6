# Programmers Autonomous-Driving Dev course. LiDAR SLAM using Cartographer

## Video
---
- Mapping
- Localization

## Goal
---
- Xycar로 Online / Offline SLAM
- Online SLAM: 실제 주행하면서 그와 동시에 map을 제작하고 현재 위치를 파악하는 SLAM
- Offline SLAM: 주행 데이터를 rosbag(ROS 데이터 수집 도구)를 활용하여 모든 데이터를 모은 후 map을 제작 및 현재 위치를 파악하는 SLAM

## Environment
---
- Ubuntu 18.04
- ROS Melodic
- Xycar Model D
- Nvidia TX 2
- Cartographer SLAM
~~~bash
- 패키지 정보 업데이트 후, 빌드에 필요한 패키지들 다운
$ sudo apt-get update
$ sudo apt-get install -y python-wstool python-rosdep ninja-build stow

- 카토그래퍼 설치 폴더 만들고, 카토그래퍼 소스 다운로드
$ mkdir carto_ws && cd carto_ws && wstool init src
$ wstool merge -t src http://raw.githubusercontent.com/cartographer-project/cartographer_Ros/master/cartographer_ros.rosinstall
$ wstool update -t src

- 카토그래퍼의 dependency 설치
$ rosdep update
$ rosdep install --from-paths src --ignore-src -rosdistro=${ROS_DISTRO} -y

- 구글에서 만든 오픈소서 C++ 라이브러리 abseil 설치
$ src/cartographer/scripts/install_abseil.sh

- 루트 계정에서 user 환경 적용 후, 설치
$ sudo su -
$ source /home/nvidia/.bashrc && cd /home/nvidia/carto_ws/
$ catkin_make_isolated --install --use-ninja --install-space /opt/ros/melodic/
~~~

## Structure
---
~~~
xycar_slam
  └─ bag                                  # LiDAR topic을 bag으로 저장한 파일을 모아놓은 폴더
  └─ config                               # Cartogrpher parameter 조정 lua 파일을 모아놓은 폴더
  └─ maps                                 # Online / Offline을 사용해서 mapping한 pbstream 파일을 모아놓은 폴더
  └─ rviz                                 # localiztion & mapping의 결과를 보여주기 위한 rviz 파일
  └─ urdf                                 # localiztion & mapping의 결과를 보여주기 위한 urdf 파일
  └─ planning                             # SLAM 이후, 기준 경로로 주행하기 위해서 사용한 planning 폴더
  └─ launch
       └─ localiztion.launch             # online localiztion launch 파일
       └─ mapping.launch                 # online mapping launch 파일
       └─ offline_localiztion.launch     # offline localiztion launch 파일
       └─ offline_mapping.launch         # offline mapping launch 파일
       └─ stanley_follower.launch        # stanley_follwer 

~~~

## Usage
---
1. Online Mapping
~~~bash
$ roslaunch xycar_slam localiztion.launch
~~~

2. Online Localization
~~~bash
$ roslaunch xycar_slam mapping.launch
~~~

3. Offline Mapping
~~~bash
$ roslaunch xycar_slam offline_localiztion.launch
~~~

4. Offline Localiztion
~~~bash
$ roslaunch xycar_slam offline_mapping.launch
~~~

## Try
---
- mapping lua 파일 조정
- localization lua 파일 조정


## Limitations
---
- 처음 lua 파일을 설정할 때 어떤 파라미터가 정확히 어떤 값인지 제대로 알지 못하고 임의로 값을 조정함.
- mapping이 제대로 되지 않아서 천천히 주행해야만 Online localizition이 수행됨.
- Online localization이 잘되면 path planning 기술을 사용해서 실제 주행까지 하고 싶었지만 정확하지 못함

## What I've learned
---
- Google Cartographer를 사용해서 LiDAR SLAM을 Online과 Offline에서 수행해봄