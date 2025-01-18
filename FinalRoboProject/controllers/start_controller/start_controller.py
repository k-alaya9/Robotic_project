import cv2 as cv
import numpy as np
from controller import Robot
import math
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, timestep):
        """Calculate the PID control output."""
        self.integral += error * timestep
        derivative = (error - self.previous_error) / timestep
        self.previous_error = error

        # PID formula
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


counter=0
pid = PIDController(Kp=0.01, Ki=0.01, Kd=0.001)  

def navigate_robot(wheels, image_width, position, isForward, threshold=35, timestep=0.032):
    cx, cy, width, height = position

    if cx is not None:  
        center_offset = cx - image_width // 2  
        value=front_sensor.getValue()
        if not isForward:
            if value < 800: 
                for wheel in wheels.values():
                    wheel.setVelocity(0.0)
                print("Robot stopped: Close to the object.")
                return True
        control_output = pid.compute(center_offset, timestep)

        base_speed = 5.0 
        left_speed = base_speed - control_output
        right_speed = base_speed + control_output

        wheels["front_left"].setVelocity(left_speed)
        wheels["front_right"].setVelocity(right_speed)
        wheels["rear_left"].setVelocity(left_speed)
        wheels["rear_right"].setVelocity(right_speed)
    else:
        for wheel in wheels.values():
            wheel.setVelocity(0.0)

    return False

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(1) 
isCatch=False
startTask=False
camera = robot.getDevice('camera')
camera.enable(timestep)
colors=[]
turn_left=False
front_camera = robot.getDevice("front camera")
front_camera.enable(timestep)
# back_camera = robot.getDevice("front camera(1)")
# back_camera.enable(timestep)
front_camera.recognitionEnable(timestep)
front_sensor=robot.getDevice('front_sensor')
front_sensor.enable(timestep)
wheels = {
        "front_left": robot.getDevice("wheel1"),
        "front_right": robot.getDevice("wheel2"),
        "rear_left": robot.getDevice("wheel3"),
        "rear_right": robot.getDevice("wheel4"),
    }
for wheel in wheels.values():
    wheel.setPosition(float('inf'))  # Set wheels to velocity mode
    wheel.setVelocity(0.0)

arms_joints = {"arm1": robot.getDevice("arm1"),
            "arm2": robot.getDevice("arm2"),
            "arm3": robot.getDevice("arm3"),
            "arm4": robot.getDevice("arm4"),
            "arm5": robot.getDevice("arm5"),
    }
for joint in arms_joints.values():
    joint.setPosition(0)
    # joint.setVelocity(0.4) 

devices={
    'gripper_fingers':[
         robot.getDevice("finger::left"),
          robot.getDevice("finger::right"),
    ],
    "arm_joints":[
         robot.getDevice("arm1"),
             robot.getDevice("arm2"),
             robot.getDevice("arm3"),
             robot.getDevice("arm4"),
             robot.getDevice("arm5"),
    ]
}
gripper_fingers = {
    "finger1": robot.getDevice("finger::left"),
    "finger2": robot.getDevice("finger::right"),
}
for finger in gripper_fingers.values():
    finger.setPosition(0.02)


# timestep = 16  
COLOR_RANGES = {
    "Red": (np.array([0, 70, 50]), np.array([10, 255, 255])),
    "Blue": (np.array([120, 100, 50]), np.array([130, 255, 255])),
    "Green": (np.array([40, 0, 10]), np.array([100, 255, 255])),
    "Yellow": (np.array([25, 0, 10]), np.array([40, 255,255]))
}

MIN_CONTOUR_AREA = 1000
def detect_color(image, lower_hsv, upper_hsv):
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_image, lower_hsv, upper_hsv)
    return mask

def find_contours(mask):
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv.contourArea(contour)
        if area > MIN_CONTOUR_AREA: 
            x, y, w, h = cv.boundingRect(contour)
            cx = x + w // 2  
            cy = y + h // 2  
            return cx, cy,w,h
    return None, None,0,0
def receive_message():
    """Check for and process incoming messages."""
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        receiver.nextPacket()
        print(f"Received: {message}")
        return message
    return None
# def navigate_robot(wheels, image_width, position,isForward,threshold=50):
#     if position is not None:
#         cx, _ ,width,_= position
#         center_offset = cx - image_width // 2
#         # value=front_sensor.getValue()
#         # print(value)
#         if not isForward:
#             if width >= 1080: 
#                 for wheel in wheels.values():
#                     wheel.setVelocity(0.0)
#                 print("Robot stopped: Close to the object.")
#                 return True

       
#         if center_offset < -threshold:  
#             wheels["front_left"].setVelocity(5)
#             wheels["front_right"].setVelocity(-5)
#             wheels["rear_left"].setVelocity(5)
#             wheels["rear_right"].setVelocity(-5)
#         elif center_offset > threshold: 
#             wheels["front_left"].setVelocity(-5)
#             wheels["front_right"].setVelocity(5)
#             wheels["rear_left"].setVelocity(-5)
#             wheels["rear_right"].setVelocity(5)
#         else:  
#             wheels["front_left"].setVelocity(5)
#             wheels["front_right"].setVelocity(5)
#             wheels["rear_left"].setVelocity(5)
#             wheels["rear_right"].setVelocity(5)
#     else:
#         for wheel in wheels.values():
#             wheel.setVelocity(0.0)
#     return False

def classify_color(color):
    blue,green,red  = color
    if red > 150 and green < 100 and blue < 100:
        return "Red"
    elif red < 100 and green > 150 and blue < 100:
        return "Green"
    elif red < 100 and green < 100 and blue > 150:
        return "Blue"
    elif red > 150 and green > 150 and blue < 100:
        return "Yellow"
    else:
        return "Unknown"

def gripper_open(devices):
    devices["gripper_fingers"][0].setPosition(0.025)
    devices["gripper_fingers"][1].setPosition(0.025)
    print("Gripper fingers opened.")

def gripper_close(devices):
    # Set gripper fingers to the specified position
    devices["gripper_fingers"][0].setPosition(0.014)
    devices["gripper_fingers"][1].setPosition(0.014)
    print("Gripper fingers closed.")
    


def catch_box(devices, robot, timestep):
 
    gripper_open(devices)

    # devices["arm_joints"][1].setPosition(-1.0)
    # devices["arm_joints"][3].setPosition(-1.0)
    # devices["arm_joints"][2].setPosition(-0.9)
    
    devices["arm_joints"][1].setPosition(0.0)
    devices["arm_joints"][2].setPosition(-1.43)
    devices["arm_joints"][3].setPosition(-1.24)

    delay_time = 1500  
    steps = int(delay_time / timestep) 
    for _ in range(steps):
        robot.step(timestep)  

    gripper_close(devices)

def reset_arm(devices):
    devices["arm_joints"][1].setPosition(0.0)
    devices["arm_joints"][3].setPosition(0.0)
    devices["arm_joints"][2].setPosition(0.0)

def put_box(devices, robot, timestep):
    global isCatch
    steps = int(2500 / timestep) 
    for _ in range(steps):
        robot.step(timestep)
    # devices["arm_joints"][1].setVelocity(0.3)
    # devices["arm_joints"][2].setVelocity(0.75)
    # devices["arm_joints"][3].setVelocity(0.3)
    # devices["arm_joints"][1].setPosition(0.5)
    # devices["arm_joints"][1].setPosition(0.80000000000000013)
    # devices["arm_joints"][2].setPosition(1)
    # devices["arm_joints"][3].setPosition(1)

    devices["arm_joints"][1].setPosition(0.8)
    devices["arm_joints"][2].setPosition(0.4)
    devices["arm_joints"][3].setPosition(2.0)

    

    for _ in range(steps):
        robot.step(timestep)

    for _ in range(steps):
        robot.step(timestep)
    gripper_open(devices)
    reset_arm(devices)
    isCatch=True

    for _ in range(steps):
        robot.step(timestep)
    turn_right_180_degrees()


def turn_right_180_degrees(turn_time=5.1):

    global turn_left
    start_time = robot.getTime()

    while robot.getTime() - start_time < turn_time:
        if not turn_left:
            wheels["front_left"].setVelocity(-5)
            wheels["front_right"].setVelocity(5)
            wheels["rear_left"].setVelocity(-5)
            wheels["rear_right"].setVelocity(5)
        else:
            wheels["front_left"].setVelocity(5)
            wheels["front_right"].setVelocity(-5)
            wheels["rear_left"].setVelocity(5)
            wheels["rear_right"].setVelocity(-5)
        robot.step(timestep)
    stop_moving()

def get_object_positions(recognized_objects):
    for obj in recognized_objects:
        position = obj.getPosition()  
        x, y, z = position
        return position  
        print(f"Object Position: x={x}, y={y}, z={z}")

is_close=False
def move_forward(image):
    # image=back_camera.getImage()
    width = front_camera.getWidth()
    height = front_camera.getHeight()
    img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    bgr_image = cv.cvtColor(img_array, cv.COLOR_BGRA2BGR)
    # cv.imshow("test",bgr_image)
    mask = detect_color(bgr_image, np.array([0,0,0]), np.array([0,0,0]))
    position = find_contours(mask)
    x,y,z,v=position
    if y:
        navigate_robot(wheels,width,position,True,threshold=50)

    # wheels["front_left"].setVelocity(10)
    # wheels["front_right"].setVelocity(10)
    # wheels["rear_left"].setVelocity(10)
    # wheels["rear_right"].setVelocity(10)
    robot.step(timestep)

def stop_moving():
    wheels["front_left"].setVelocity(0)
    wheels["front_right"].setVelocity(0)
    wheels["rear_left"].setVelocity(0)
    wheels["rear_right"].setVelocity(0)
    robot.step(timestep)


def catch_box_2(devices, robot, timestep):
 
    gripper_open(devices)

    # devices["arm_joints"][1].setPosition(0.8)
    # devices["arm_joints"][2].setPosition(0.75)
    # devices["arm_joints"][3].setPosition(1.0)

    devices["arm_joints"][1].setPosition(0.8)
    devices["arm_joints"][2].setPosition(0.55)
    devices["arm_joints"][3].setPosition(1.6)

    delay_time = 1500  
    steps = int(delay_time / timestep) 
    for _ in range(steps):
        robot.step(timestep)  

    gripper_close(devices)

def put_box_2(devices, robot, timestep):
    # turn_right_180_degrees()

    global isCatch
    steps = int(3500 / timestep) 
    for _ in range(steps):
        robot.step(timestep)

    devices["arm_joints"][1].setPosition(0.0)
    devices["arm_joints"][2].setPosition(-1.25)
    devices["arm_joints"][3].setPosition(-1.6)


    for _ in range(steps):
        robot.step(timestep)
    # reset_arm(devices)
    devices["arm_joints"][3].setPosition(0)

    gripper_open(devices)
    for _ in range(steps):
        robot.step(timestep)
    reset_arm(devices)
    isCatch=True


def main():
    isFirstTime= True
    global startTask
    global counter
    global is_close
    global isCatch
    global colors
    global turn_left
    while robot.step(timestep) != -1:
        
        # print(front_sensor.getValue())

        # catch_box_2(devices, robot, timestep)
        # put_box_2(devices, robot, timestep)
        # turn_right_180_degrees()
        # catch_box(devices, robot, timestep)
        # put_box(devices, robot, timestep)
        # width = camera.getWidth()
        # height = camera.getHeight()
        # raw_image = camera.getImage()
        # image = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4))
        # image=cv.resize(image,(10*width,10*height))
        # bgr_image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)
        # cv.imshow("Camera", bgr_image)
        if counter >= 4:
            break
        message = receive_message()
        if message:
            startTask=True
            print(message)
        if startTask==True:
            # print("Starting the next task...")
            if not isCatch:
                if isFirstTime:
                    turn_right_180_degrees()
                    isFirstTime=False
                catch_box(devices, robot, timestep)
                put_box(devices, robot, timestep)
            width = camera.getWidth()
            height = camera.getHeight()
            raw_image = camera.getImage()
            image = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4))
            image=cv.resize(image,(10*width,10*height))
            bgr_image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)
            # cv.imshow("Camera", bgr_image)
            data = np.reshape(bgr_image, (-1,3))
            data = np.float32(data)

            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            flags = cv.KMEANS_RANDOM_CENTERS
            compactness,labels,centers = cv.kmeans(data,1,None,criteria,10,flags)
            color =classify_color(centers[0].astype(np.int32))
            if len(colors)==0 or len(colors)<4:
                if color!="Unknown" and colors.__contains__(color)==False:
                    colors.append(color)
            image = front_camera.getImage()
            if is_close==True:
                if front_sensor.getValue() > 330:
                    move_forward(image)
                else:
                    stop_moving()
                    isCatch=False
                    is_close=False
                    # catch_box_2(devices, robot, timestep)
                    # put_box_2(devices, robot, timestep)
                    # reset_arm(devices)
                    # turn_right_180_degrees(turn_time=5)
                    
            if image:
                width = front_camera.getWidth()
                height = front_camera.getHeight()
                img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
                bgr_image = cv.cvtColor(img_array, cv.COLOR_BGRA2BGR)

                detected_positions = {}
                for color, (lower_hsv, upper_hsv) in COLOR_RANGES.items():
                    mask = detect_color(bgr_image, lower_hsv, upper_hsv)
                    position = find_contours(mask)
                    if position[0] is not None:
                        detected_positions[color] = position

                    # print(colors)
                    # print(detected_positions)
                
                if not is_close and colors and colors[0] in detected_positions and isCatch == True:
                    if colors[0]=='Red' or colors[0]=='Blue':
                            turn_left=True
                    position = detected_positions[colors[0]]
                    is_close = navigate_robot(wheels, width, position,False)
                    if is_close:
                        # recognized_objects = front_camera.getRecognitionObjects()
                        # position=get_object_positions(recognized_objects)
                        # catch_box_to_position(position)
                        catch_box_2(devices, robot, timestep)
                        put_box_2(devices, robot, timestep)
                        colors.remove(colors[0])
                        turn_right_180_degrees()
                        counter = counter +1
                        # isCatch=False

                        # detect_box_position(bgr_image)
                        # colors.pop()
                        # break
            
        if cv.waitKey(1) == 27: 
            break
main()