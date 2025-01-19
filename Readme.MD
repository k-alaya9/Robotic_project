# Robot Navigation Project

## Overview
This project implements an autonomous robot navigation system using **Webots** and **Python**. The robot is tasked with following a black line on the ground, with a dynamic approach to locate and navigate towards a **black door** at the end of the path. The system incorporates a **PID controller** for smoother, more precise movements and error correction as the robot follows the line. The robot is designed to adapt to multiple path variations and ensure it reaches the goal (the black door) reliably.

## Features
- **Black Line Following**: The robot detects and follows a black line on the ground.
- **PID Control**: A PID controller is used to correct the robot's steering by calculating the error from the center of the black line.
- **Dynamic Door Detection**: The robot detects and navigates toward the largest black area (e.g., the black door) when multiple paths are present.
- **Real-Time Feedback**: The robot continuously adjusts its movement in real-time based on the black area detected in the camera feed.

## Key Components
- **Webots Simulator**: Used for the simulation environment, where the robot and its surroundings are modeled.
- **PID Controller**: Used to adjust the robot’s velocity based on the error between the current path and the target.
- **Computer Vision**: The robot uses image processing techniques (using **OpenCV**) to detect black areas and follow the black line.


## How It Works

### 1. **Line Following**:
   - The robot uses a **front-facing camera** to capture the image of the ground. The **black line** is detected by applying color thresholds in **HSV color space**.
   - The position of the black line relative to the camera’s center is calculated, and the robot adjusts its velocity using the PID controller to align with the center of the line.

### 2. **PID Controller**:
   - A **PID controller** is used to steer the robot:
     - **Proportional (P)** term adjusts the robot’s velocity based on the magnitude of the error.
     - **Integral (I)** term helps to reduce accumulated errors.
     - **Derivative (D)** term smoothens the robot’s motion and dampens oscillations.

### 3. **Black Door Detection**:
   - When the robot detects a sufficiently large black area (the black door), it stops and performs an action, such as picking up or navigating past the door.
   - The **largest detected black area** is used to identify the door, ensuring the robot doesn't veer off to other paths.

### 4. **Adaptive Navigation**:
   - The robot can handle multiple paths by continuously evaluating the largest black area and ensuring it follows the right path toward the black door.
   - If the robot detects that it’s deviating from the correct path, it adjusts its trajectory based on real-time feedback.

## Dependencies
- **Python** (3.x)
- **OpenCV**: For image processing.
- **Webots**: For robot simulation.
- **NumPy**: For numerical operations used in image processing and PID calculations.

## Installation

1. **Clone the repository**:
   ```
   git clone https://github.com/k-alaya9/Robotic_project.git
   cd Robotic_project
   cd FinalRoboProject
    ```

2. **Install dependencies**:
    Install OpenCV and NumPy:
    ```
    pip install opencv-python numpy
    ```

3. **Run the Webots simulator**:
    Open the Webots simulator and load the world file (robot-world.wbt).
    Ensure First robot controller is set to my_controller.py for execution.
    Ensure Second robot controller is set to start_controller.py for execution.

4. **Start the simulation**:
    Click on Run in Webots to start the simulation, and the robot will begin navigating the path.


## Features
-Black Line Following: The robot detects and follows a black line on the ground.
-PID Control: A PID controller is used to correct the robot's steering by calculating the error from the center of the black line.
-Dynamic Door Detection: The robot detects and navigates toward the largest black area (e.g., the black door) when multiple paths are present.
-Real-Time Feedback: The robot continuously adjusts its movement in real-time based on the black area detected in the camera feed.

## Controllers
- **my_controllerSuper**: Manages the overall control logic of the robot.
- **start_controller**: Handles the initialization and starting procedures.
- **my_controller**: Additional control logic for specific tasks.
- **box_detection**: Detects boxes using computer vision techniques.


## License
This project is licensed under the MIT License.

## Contact Information
For questions or contributions, please contact k_alaya9 at khaledalaya549@gmail.com.
