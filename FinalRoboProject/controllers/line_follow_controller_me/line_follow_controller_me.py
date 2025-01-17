# """line_following controller."""

# # You may need to import some classes of the controller module. Ex:
# #  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
# import cv2 as cv
# import numpy as np
# from robomaster import robot

# EPUCK_MAX_VELOCITY = 6.28

# kp=0.02
# kd=0.02
# ki=0

# last_error=0
# integral =0

# def range_conversion(s_start, s_end, d_start, d_end, value):
#     """
#     This function is responsible for mapping ranges
#     examples:
#     the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
#     """
#     ration = abs((value - s_start) / (s_end - s_start))
#     if(d_start < d_end):
#         return  d_start + abs(d_end - d_start) * ration 
#     if(d_start > d_end):
#         return  d_start - abs(d_end - d_start) * ration 

# class RobotController(Robot):
#     def __init__(self):
#         Robot.__init__(self)
#         self.timestep = int(self.getBasicTimeStep())
#         self.front_camera=self.getDevice("front camera")
#         self.motor_fl = self.getDevice("wheel1")  # العجلة الأمامية اليسرى
#         self.motor_fr = self.getDevice("wheel2")  # العجلة الأمامية اليمنى
#         self.motor_bl = self.getDevice("wheel3")  # العجلة الخلفية اليسرى
#         self.motor_br = self.getDevice("wheel4")  # العجلة الخلفية اليمنى

#         self.front_camera.enable(self.timestep)
#         # إعداد المحركات كسرعة غير محدودة
#         for motor in [self.motor_fl, self.motor_fr, self.motor_bl, self.motor_br]:
#             motor.setPosition(float("inf"))
#             motor.setVelocity(0)

#         self.is_line=False

#         # self.right_sensor=self.getDevice("ps2")

#         # self.front_sensor=self.getDevice('ps0')
#         # self.center_sensor=self.getDevice('ps1')
#         self.weights_wall=[-1000,-1000,-1000,1000,1000,1000]

#         self.sensors = list(map(lambda v: self.getDevice(f"ir({v})"), range(8)))

#         self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]
#         # self.sensors_wall= list(map(lambda v: self.getDevice(f'ps{v}'),range(6)))
#         # for s in self.sensors_wall:
#         #     # print("enabled: ", s)
#         #     s.enable(self.timestep)

#         for sensor in self.sensors:
#             # print("enabled: ", sensor)
#             sensor.enable(self.timestep)

#         self.step(self.timestep)

#     def PID(self,velocity=EPUCK_MAX_VELOCITY):
#         global last_error,integral 
#         if self.is_line==False:
#             value=self.get_sensors_value()
#             error = 0-value
#             P=kp*error

#             D=kd*(last_error-error)

#             last_error=error
#             I=ki * integral
#             integral+=error
#             stearing= P + D + I
#             self.run_motors_stearing(stearing=stearing,velocity=velocity)
#         else :
#             # kp=0.01
#             # kd=0.01
#             # ki=0
#             last_error=0
#             integral=0
#             value=self.get_sensors_value()
#             error = 0-value
#             P=kp*error
#             D=kd*(last_error-error)

#             last_error=error
#             I=ki * integral
#             integral+=error
#             stearing= P + D + I
#             self.run_motors_stearing(stearing=stearing,velocity=velocity)

    
#     def get_sensors_value(self):
#         value = 0
#         if self.is_line==False:
#             for index, sensor in enumerate(self.sensors):
#                 # print(self.sensors[4].getValue())
#                 # print(self.sensors[4].getValue())
#                 if(sensor.getValue() > 60):
#                     value += self.weights[index]
#         else:
#             for i , s in enumerate(self.sensors_wall):
#                 print(self.sensors_wall[2].getValue())
#                 if(s.getValue()>60):
#                     value+=self.weights_wall[i]
#         return value

#     def line_follow_step(self, velocity = EPUCK_MAX_VELOCITY):
#         """
#         The function that is responsible for doing one step of the line following (without looping)
#         """
#         value = self.get_sensors_value()
#         self.run_motors_stearing(range_conversion(-4000, 4000, 30, -30, value), velocity)
#         right_sensor_value=self.right_sensor.getValue()
#         front_sensor_value=self.front_sensor.getValue()
#         center_sensor_value=self.center_sensor.getValue()
#     def wall_follow_step(self,velocity=EPUCK_MAX_VELOCITY):
#         if front_sensor_value > 80 or center_sensor_value> 80:
#             self.left_motor.setVelocity(-2)
#             self.right_motor.setVelocity(2)
#         else:
#             stearing=range_convertor(60,140,20,-20,right_sensor_value)
#             self.stearing(stearing=stearing)

#     def run_motors_stearing(self, stearing, velocity = EPUCK_MAX_VELOCITY):
#         """
#         A function that is responsible for the stearing functionality for the motor
#         stearing value:
#             - from -100 to 0 will turn left
#             - from 0 to 100 will turn right
#             - if equals 100 will turn the robot around it self to the right
#             - if equals -100 will turn the robot around it self to the left
#             - if equals 50 will turn the robot around right wheel to the right
#             - if equals -50 will turn the robot around left wheel to the left
#         """
#         if(self.is_line==False):
#             if stearing < 0:  # دوران لليسار
#                 fl_speed = velocity
#                 fr_speed = range_conversion(0, 100, velocity, -velocity, stearing)
#                 bl_speed = velocity
#                 br_speed = range_conversion(0, 100, velocity, -velocity, stearing)
#             elif stearing > 0:  # دوران لليمين
#                 fl_speed = range_conversion(0, -100, velocity, -velocity, stearing)
#                 fr_speed = velocity
#                 bl_speed = range_conversion(0, -100, velocity, -velocity, stearing)
#                 br_speed = velocity
#             else:  # حركة مستقيمة
#                 fl_speed = velocity
#                 fr_speed = velocity
#                 bl_speed = velocity
#                 br_speed = velocity

#             # تعيين السرعات للعجلات
#             self.motor_fl.setVelocity(fl_speed)
#             self.motor_fr.setVelocity(fr_speed)
#             self.motor_bl.setVelocity(bl_speed)
#             self.motor_br.setVelocity(br_speed)
#         else:
#             front_value=self.sensors_wall[0].getValue()
#             center_value=self.sensors_wall[1].getValue()
#             right_value=self.sensors_wall[2].getValue()
#             if front_value > 80 or center_value> 80:
#                 self.left_motor.setVelocity(-2)
#                 self.right_motor.setVelocity(2)
#             else:
#                 stearing=range_conversion(60,140,20,-20,right_value)
#                 right_velocity=velocity if stearing < 0 else range_conversion(100,0,-velocity,velocity,stearing)

#                 left_velocity=velocity if stearing > 0 else range_conversion(-100,0,-velocity,velocity,stearing)

#                 self.right_motor.setVelocity(right_velocity)
#                 self.left_motor.setVelocity(left_velocity)

#     def line_following(self, velocity = EPUCK_MAX_VELOCITY):
#         """
#         The function that is responsible for the line fallowing functionality (with looping).
#         """
#         while(self.step(self.timestep) != -1):
#             self.line_follow_step(velocity) 
#     def PID_step(self,velocity=EPUCK_MAX_VELOCITY):
#         while (self.step(self.timestep) != -1):
#             self.PID(velocity)
#             self.get_front_image()
#             width = self.front_camera.getWidth()
#             height = self.front_camera.getHeight()
#             raw_image = self.front_camera.getImage()
#             image = np.frombuffer(raw_image, dtype=np.uint8).reshape((self.front_camera.getHeight(), self.front_camera.getWidth(), 4))
#             # image=cv.resize(image,(10*width,10*height))
#             bgr_image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)
#             cv.imshow('Robot Camera', bgr_image)
#             cv.waitKey(1)
#             # for i , s in enumerate(self.sensors_wall):
#             #     if(s.getValue()>100 and self.sensors[4].getValue()<15):
#             #         print('hh')
#                     # self.is_line=True
#             # if self.sensors_wall[0].getValue()>80 and self.sensors[4].getValue()<15:
#             #     print('hh')
#             #     self.is_line=True
#     def print_sensor_value(self):
#         """
#         The function that is responsible for prining sensors value (for debugging).
#         """
#         while(self.step(self.timestep) != -1):
#             print(self.get_sensors_value())
#             # self.left_motor.setVelocity(2)
#             # self.right_motor.setVelocity(2)


# r = RobotController()
# # r.line_following(2)
# r.PID_step(2)
# # r.print_sensor_value()


import cv2 as cv
import numpy as np
from controller import Robot, Camera, Motor

TIME_STEP = 32  # Time step for Webots simulation

# HSV color ranges for red, blue, and green
COLOR_RANGES = {
    "red": (np.array([0, 120, 70]), np.array([10, 255, 255])),
    "blue": (np.array([110, 150, 50]), np.array([130, 255, 255])),
    "green": (np.array([30, 0, 0]), np.array([70, 255, 255])),
    "yellow": (np.array([50, 50, 50]), np.array([80, 120,120]))
}

# Minimum contour area to detect objects (filtering noise)
MIN_CONTOUR_AREA = 500

def detect_color(image, lower_hsv, upper_hsv):
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_image, lower_hsv, upper_hsv)
    return mask

def find_contours(mask):
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv.contourArea(contour)
        if area > MIN_CONTOUR_AREA:  # Filter by area
            x, y, w, h = cv.boundingRect(contour)
            cx = x + w // 2  # Center x-coordinate
            cy = y + h // 2  # Center y-coordinate
            return cx, cy
    return None, None

def navigate_robot(wheels, image_width, position):
    if position is not None:
        cx, _ = position
        center_offset = cx - image_width // 2

        # Adjust thresholds for turning left, right, or going straight
        if center_offset < -50:  # Object is to the left
            wheels["front_left"].setVelocity(10.0)
            wheels["front_right"].setVelocity(-10.0)
            wheels["rear_left"].setVelocity(10.0)
            wheels["rear_right"].setVelocity(-10.0)
        elif center_offset > 50:  # Object is to the right
            wheels["front_left"].setVelocity(-10.0)
            wheels["front_right"].setVelocity(10.0)
            wheels["rear_left"].setVelocity(-10.0)
            wheels["rear_right"].setVelocity(10.0)
        else:  # Object is in the center, move forward
            wheels["front_left"].setVelocity(10.0)
            wheels["front_right"].setVelocity(10.0)
            wheels["rear_left"].setVelocity(10.0)
            wheels["rear_right"].setVelocity(10.0)
    else:
        # Stop if no object is detected
        for wheel in wheels.values():
            wheel.setVelocity(0.0)

def main():
    # Initialize Webots Robot
    robot = Robot()

    # Get camera and enable it
    camera = robot.getDevice("front camera")
    camera.enable(TIME_STEP)

    # Get wheel motors
    wheels = {
        "front_left": robot.getDevice("wheel1"),
        "front_right": robot.getDevice("wheel2"),
        "rear_left": robot.getDevice("wheel3"),
        "rear_right": robot.getDevice("wheel4"),
    }
    for wheel in wheels.values():
        wheel.setPosition(float('inf'))  # Set wheels to velocity mode
        wheel.setVelocity(0.0)

    while robot.step(TIME_STEP) != -1:
        # Get image from the camera
        image = camera.getImage()
        if image:
            # Convert Webots image to OpenCV format
            width = camera.getWidth()
            height = camera.getHeight()
            img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
            bgr_image = cv.cvtColor(img_array, cv.COLOR_BGRA2BGR)

            # Detect colors and find the largest object for each color
            detected_positions = {}
            for color, (lower_hsv, upper_hsv) in COLOR_RANGES.items():
                mask = detect_color(bgr_image, lower_hsv, upper_hsv)
                position = find_contours(mask)
                if position[0] is not None:
                    detected_positions[color] = position
            print(detected_positions)
            # Decide navigation based on the detected color
            if "redd" in detected_positions:
                navigate_robot(wheels, width, detected_positions["red"])
            elif "bluew" in detected_positions:
                # navigate_robot(wheels, width, detected_positions["blue"])
                print("blue")
            elif "green" in detected_positions:
                navigate_robot(wheels, width, detected_positions["green"])
            else:
                # Stop if no object is detected
                for wheel in wheels.values():
                    wheel.setVelocity(0.0)

            # Display the image with OpenCV (for debugging)
            cv.imshow("Camera", bgr_image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    # Cleanup
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
