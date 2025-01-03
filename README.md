## 🐢 turtlebot3_Manipulator_System

This project combined **TurtleBot3** and **Open Manipulator** to create a mobile manipulator, and implemented a system that controls the conveyor belt with Arduino to move the block that fits the task to the target point. The UI was composed of **Flask**, and **calibration** was used to calculate the relative coordinates with the target point. we implemented to detect the box with the **YOLO model** and move the box to the target point.

<br>**터틀봇3와 오픈 매니퓰레이터를 결합하여 모바일 매니퓰레이터를 사용하였다.**

**또한 아두이노로 컨베이어 벨트를 제어하여 목표 지점으로 task에 맞는 블럭을 옮기는 시스템을 구현하였다.**

<br>

## ✨ Key Features

- **Object Detection**: Use YOLO detection to detect the location of the box corresponding to the task.
- **Calibration**: The actual distance was measured by removing the distortion of the global cam through calibration.
- **Rotation matrix**: The relative coordinates of the Turtlebot were obtained from the target using a rotation matrix.
- **Flask**: A UI for sending and managing tasks to flask was constructed.
- **Arduino**: The conveyor belt was controlled within the python code using Arduino.
- **Open Manipulator**: pick and place with organic open manipulator movement.

<br>

<img src="https://github.com/user-attachments/assets/1da9bb16-d437-4663-bfaf-968b3b31e8ea" width=500>

<br><br>

## 🎥 Demo

**PC 영상에 버퍼링이 심합니다**

https://github.com/user-attachments/assets/ff0aeb5f-5c30-4dd5-b911-854e7b95eaf9

<br>

## 👨‍💻 Contributors
A big thank you to everyone who made this project possible! 🎉  
Click on the names below to view their GitHub profiles:

- [**MINSIK YOON**](https://github.com/yms0606)  
- [**junha JANG**](https://github.com/zzangzzun)  
- [**seunghwan JEONG**](https://github.com/JSeungHwan)

