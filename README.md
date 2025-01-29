# ROS2_E2E_VisualProcessor



https://github.com/GODCREATOR333/ROS2_E2E_VisualProcessor/assets/66235065/0a815ef2-5370-43d8-95ed-800207aef717

Object Detection Model (YOLOv8) Integration with ROS2 Node Using Websockets for Real-Time Processing (Newer version uses webRTC for minial latency [Updated version still in development mode](https://github.com/GODCREATOR333/SensoryAI-ROS2) .

This repository aims to implement an Object Detection model based on YOLOv8 architecture within a ROS2 environment, enabling real-time detection and processing of objects. By leveraging websockets, this system facilitates seamless communication between components, ensuring efficient data transfer and analysis.

# ROS2_E2E_VisualProcessor

ROS2_E2E_VisualProcessor is an open-source, real-time visual processing system designed for ROS2 (Robot Operating System 2) applications and robotics projects. This processor enables robots to perceive their surroundings using camera and LiDAR systems, providing low-latency solutions through the integration of Linux’s dual kernel, Preempt_RT. It leverages the YOLOv8 object detection model running on the CUDA platform for high-performance object detection and incorporates the Extended Kalman Filter (EKF) Simultaneous Localization and Mapping (SLAM) algorithm to enhance robot navigation and mapping capabilities. This is a hobby project for me to implement what i had learnt in theory to the real world with hands-on approach. I am further using [Nvidia Isaac Sim](https://developer.nvidia.com/isaac/sim) to test viability ,precision and accuracy in a simulated environment of a simple robot with these models running.

## Technologies Used

- **ROS2**: Built on ROS2 for robotic applications, providing the framework for real-time communication and control.
- **YOLOv8**: Implemented the latest YOLOv8 object detection model to achieve high-speed and accurate object detection.
- **CUDA**: Utilized CUDA for accelerating the YOLOv8 model, ensuring high performance on compatible hardware.
- **EKF SLAM**: Integrated the Extended Kalman Filter (EKF) for Simultaneous Localization and Mapping (SLAM) to enhance robot navigation.
- **Preempt_RT**: Leveraged the Preempt_RT kernel patch to reduce latency, making the system suitable for real-time robotics applications.
- **Websockets & WebRTC**: Used Websockets (and WebRTC in the newer version) for seamless, real-time communication between components.

## Features

- **Real-Time Object Detection**: Utilizes the YOLOv8 model on the CUDA platform for efficient and accurate object detection.
- **Low-Latency Processing**: Achieves reduced latency by integrating Linux’s Preempt_RT, which modifies the native scheduler.
- **Enhanced Navigation**: Incorporates EKF SLAM for improved robot localization and mapping.
- **Multi-Sensor Integration**: Processes data from both camera and LiDAR systems to provide a comprehensive understanding of the environment.

## Performance Optimization

The current system latency is approximately 3000ms. To achieve real-time performance, our goal is to reduce this latency to below 200ms. A significant factor contributing to the current latency is a kernel-related issue. We are actively working on optimizing the system to address this and enhance overall performance.


## Acknowledgements

Special thanks to the open-source community and the developers of ROS2, YOLOv8, and the EKF SLAM algorithm for their invaluable contributions.

---

References

Ultralytics YOLOv8: [https://github.com/ultralytics]
ROS2 Documentation: [https://docs.ros.org/en/humble/index.html]
Preempt_RT Patch: [https://www.acontis.com/en/ubuntu-linux-realtime-howto.html]
Xenomai Real-Time Framework: [https://v3.xenomai.org/overview/]
