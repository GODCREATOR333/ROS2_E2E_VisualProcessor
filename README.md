# ROS2_E2E_VisualProcessor



https://github.com/GODCREATOR333/ROS2_E2E_VisualProcessor/assets/66235065/0a815ef2-5370-43d8-95ed-800207aef717

Object Detection Model (YOLOv8) Integration with ROS2 Node Using Websockets for Real-Time Processing (Newer version uses webRTC for minial latency [https://github.com/GODCREATOR333/SensoryAI-ROS2]{Updated version still in development mode}

This repository aims to implement an Object Detection model based on YOLOv8 architecture within a ROS2 environment, enabling real-time detection and processing of objects. By leveraging websockets, this system facilitates seamless communication between components, ensuring efficient data transfer and analysis.

Current Latency Status and Optimization Goals

At present, the latency of the system stands at approximately <3000ms, a figure that exceeds optimal real-time processing requirements. Our primary objective is to minimize this latency to <200ms, thus enhancing the responsiveness and efficiency of the Object Detection model.

Identified Kernel Issue and Proposed Solution

A critical impediment to achieving real-time performance is the scheduler problem inherent in the Ubuntu 22.04 kernel, particularly for applications requiring instantaneous responsiveness. The optimization strategy revolves around enhancing the real-time capabilities of the operating system kernel.

Consideration of Real-Time Kernel Solutions

While various solutions exist, including Xenomai, which facilitates real-time capabilities by running the Cobalt kernel alongside the Linux kernel, we have opted for the Preempt_RT patch approach. This decision stems from its flexible architecture design and minimal alterations to the original system code, ensuring smoother integration and maintenance.

# ROS2_E2E_VisualProcessor

ROS2_E2E_VisualProcessor is an open-source, real-time visual processing system designed for ROS2 (Robot Operating System 2) applications and robotics projects. This processor enables robots to perceive their surroundings using camera and LiDAR systems, providing low-latency solutions through the integration of Linux’s dual kernel, Preempt_RT. It leverages the YOLOv8 object detection model running on the CUDA platform for high-performance object detection and incorporates the Extended Kalman Filter (EKF) Simultaneous Localization and Mapping (SLAM) algorithm to enhance robot navigation and mapping capabilities. This is a hobby project for me to implement what i had learnt in theory to the real world with hands-on approach.

Technologies Used
ROS2: Built on ROS2 for robotic applications, providing the framework for real-time communication and control.
YOLOv8: Implemented the latest YOLOv8 object detection model to achieve high-speed and accurate object detection.
CUDA: Utilized CUDA for accelerating the YOLOv8 model, ensuring high performance on compatible hardware.
EKF SLAM: Integrated the Extended Kalman Filter for Simultaneous Localization and Mapping to enhance robot navigation.
Preempt_RT: Leveraged the Preempt_RT kernel patch to reduce latency, making the system suitable for real-time robotics applications.
Websockets & WebRTC: Used Websockets (and WebRTC in the newer version) for seamless, real-time communication between components.

## Features

- **Real-Time Object Detection**: Utilizes the YOLOv8 model on the CUDA platform for efficient and accurate object detection.
- **Low-Latency Processing**: Achieves reduced latency by integrating Linux’s Preempt_RT, which modifies the native scheduler.
- **Enhanced Navigation**: Incorporates EKF SLAM for improved robot localization and mapping.
- **Multi-Sensor Integration**: Processes data from both camera and LiDAR systems to provide a comprehensive understanding of the environment.

## Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/GODCREATOR333/ROS2_E2E_VisualProcessor.git
   cd ROS2_E2E_VisualProcessor
   ```

2. **Install Dependencies**:

   Ensure that you have ROS2 installed on your system. For installation instructions, refer to the [ROS2 Documentation](https://docs.ros.org/en/rolling/index.html).

   Install the required Python packages:

   ```bash
   pip install -r requirements.txt
   ```

3. **Build the Package**:

   ```bash
   colcon build
   ```

## Usage

1. **Source the ROS2 Environment**:

   ```bash
   source /opt/ros/<your_ros2_distro>/setup.bash
   source install/setup.bash
   ```

2. **Launch the Processor**:

   ```bash
   ros2 launch ros2_e2e_visualprocessor visual_processor_launch.py
   ```

   This will start the visual processor, enabling real-time object detection and SLAM capabilities.

## Configuration

The processor can be configured through various parameters defined in the `config` directory. Key parameters include:

- **Object Detection Threshold**: Sets the confidence threshold for object detection.
- **SLAM Parameters**: Adjust settings related to the EKF SLAM algorithm.
- **Sensor Calibration**: Configure camera and LiDAR calibration settings.

Modify these parameters as needed to suit your specific application requirements.

## Performance Optimization

The current system latency is approximately 3000ms. To achieve real-time performance, our goal is to reduce this latency to below 200ms. A significant factor contributing to the current latency is a kernel-related issue. We are actively working on optimizing the system to address this and enhance overall performance.

## Contributing

We welcome contributions to enhance the functionality and performance of ROS2_E2E_VisualProcessor. Please fork the repository and submit pull requests for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

Special thanks to the open-source community and the developers of ROS2, YOLOv8, and the EKF SLAM algorithm for their invaluable contributions.

---

For more information and updates, visit the [ROS2_E2E_VisualProcessor GitHub Repository](https://github.com/GODCREATOR333/ROS2_E2E_VisualProcessor). 

References

Ultralytics YOLOv8: [https://github.com/ultralytics]
ROS2 Documentation: [https://docs.ros.org/en/humble/index.html]
Preempt_RT Patch: [https://www.acontis.com/en/ubuntu-linux-realtime-howto.html]
Xenomai Real-Time Framework: [https://v3.xenomai.org/overview/]
