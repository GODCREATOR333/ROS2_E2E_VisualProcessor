# ROS2_E2E_VisualProcessor



https://github.com/GODCREATOR333/ROS2_E2E_VisualProcessor/assets/66235065/0a815ef2-5370-43d8-95ed-800207aef717

Object Detection Model (YOLOv8) Integration with ROS2 Node Using Websockets for Real-Time Processing

This repository aims to implement an Object Detection model based on YOLOv8 architecture within a ROS2 environment, enabling real-time detection and processing of objects. By leveraging websockets, this system facilitates seamless communication between components, ensuring efficient data transfer and analysis.

Current Latency Status and Optimization Goals

At present, the latency of the system stands at approximately <3000ms, a figure that exceeds optimal real-time processing requirements. Our primary objective is to minimize this latency to <200ms, thus enhancing the responsiveness and efficiency of the Object Detection model.

Identified Kernel Issue and Proposed Solution

A critical impediment to achieving real-time performance is the scheduler problem inherent in the Ubuntu 22.04 kernel, particularly for applications requiring instantaneous responsiveness. The optimization strategy revolves around enhancing the real-time capabilities of the operating system kernel.

Optimization Strategies

The proposed approach involves the utilization of a microkernel in conjunction with a native Linux kernel, operating simultaneously to address real-time tasks efficiently. Real-time tasks are executed on the microkernel, which assumes control of interrupts and manages them at the lowest level, ensuring swift response times. When real-time tasks are inactive, the Linux kernel resumes operation, ensuring the seamless functioning of non-real-time processes.

Consideration of Real-Time Kernel Solutions

While various solutions exist, including Xenomai, which facilitates real-time capabilities by running the Cobalt kernel alongside the Linux kernel, we have opted for the Preempt_RT patch approach. This decision stems from its flexible architecture design and minimal alterations to the original system code, ensuring smoother integration and maintenance.

Implementation Steps

To mitigate the kernel scheduler problem and enhance real-time performance within the ROS2 architecture, the kernel must undergo patching. This involves integrating a microkernel or utilizing a real-time kernel patch, such as preempt_rt. Subsequently, the system can leverage the optimized kernel to achieve the desired real-time capabilities, thus reducing latency and enhancing overall performance.

Conclusion

In conclusion, the integration of YOLOv8 Object Detection model with ROS2 node using websockets holds significant potential for real-time applications. By addressing kernel-level issues and adopting optimized strategies, we aim to achieve seamless and efficient processing of objects in real-time scenarios, paving the way for diverse applications across domains.

References

Ultralytics YOLOv8: [https://github.com/ultralytics]
ROS2 Documentation: [https://docs.ros.org/en/humble/index.html]
Preempt_RT Patch: [https://www.acontis.com/en/ubuntu-linux-realtime-howto.html]
Xenomai Real-Time Framework: [https://v3.xenomai.org/overview/]
