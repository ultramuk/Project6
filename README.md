# Programmers Autonomous-Driving Dev course. LiDAR SLAM using Cartographer

## Video
---

## Goal
---
- Xycar로 Online / Offline Mapping

## Environment
---
- Ubuntu 18.04
- ROS Melodic
- Xycar Model D
- Nvidia TX 2
- Cartographer

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
  │    └─ localiztion.launch             # online localiztion launch 파일
  │    └─ mapping.launch                 # online mapping launch 파일
  │    └─ offline_localiztion.launch     # offline localiztion launch 파일
  │    └─ offline_mapping.launch         # offline mapping launch 파일
  │    └─ stanley_follower.launch        # stanley_follwer 

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
## Procedure
---

## Try
---

## Limitations
---


## What I've learned
---
