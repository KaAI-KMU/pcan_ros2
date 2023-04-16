# PCAN-ROS2
ROS2를 활용한 CAN 데이터 처리

이번 프로젝트는 ROS2로 CAN 데이터를 처리하는 프로젝트이다. [ucwin-can-settings](https://github.com/KaAI-KMU/ucwin-can-settings) 에서는 windows socket programming으로 값을 주고 받았지만 이번엔 ROS를 사용해서 데이터를 송수신할 수 있도록 한다. 

[Screencast from 04-16-2023 11_20_03 PM.webm](https://user-images.githubusercontent.com/111988634/232325315-054e4554-a04f-4d4e-b166-884b2afa20cb.webm)

**개발환경**

- CMake
- C++
- Ubuntu 22.04
- ROS2 Humble

<br>

**프로젝트 개요**

1. [Ubuntu에 PCAN-Basic API 세팅](https://www.notion.so/PCAN-USB-Ubuntu-Setting-ac499925fcc34dcd95c2cfe8f96e27b2)
2. [Linux (Ubuntu) 환경에서 ROS2 세팅](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
3. [ROS2를 통한 CAN data 송수신](https://kaai.notion.site/ROS2-publish-subscribe-CAN-data-cc54ced8df3a4dda891614fc32eb355b)
4. Data Parsing 후 GUI 출력

<br>

**사용방법**

[PCAN-USB Ubuntu Setting](https://www.notion.so/PCAN-USB-Ubuntu-Setting-ac499925fcc34dcd95c2cfe8f96e27b2)과 [ROS2 Humble을 세팅](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)이 완료됐다고 가정한다.
1. 레포지토리 git clone
    
    ```bash
    cd ros2_ws/src
    git clone https://github.com/KaAI-KMU/pcan_ros2.git
    ```
    
2. build 및 실행
    
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build --packages-select pcan_ros2
    ```
    
    ```bash
    ros2 run cpp_pubsub talker &
    ros2 run cpp_pubsub listener
    ```
