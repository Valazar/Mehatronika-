from controller import Robot, Motor
import threading
import time
import math
import std_msgs.msg as msg
import rospy

TIME_STEP = 64

robot = Robot()

class DifferentialDriveRobot:
    def __init__(self):
        self.commands = []

        self.w_max = 2.84
        self.wheel_radius = 66.0 / 2
        self.v_max = self.w_max * self.wheel_radius
        self.L = 160
        self.accel = 1 * self.v_max
        self.state = 'idle'

        self.left_wheel = robot.getDevice('left wheel motor')
        self.right_wheel = robot.getDevice('right wheel motor')

        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))

        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)

    def move(self, distance_in_mm, direction, rotation_angle=0):
        v_ref = 0

        T_acc = self.v_max / self.accel
        d_acc = self.accel * T_acc * (T_acc / 2)

        T_dec = T_acc
        d_dec = d_acc

        if (d_acc + d_dec) <  distance_in_mm:
            d_const = distance_in_mm - d_acc - d_dec
            T_const = d_const / self.v_max
            v_vrh = self.v_max
        else:
            d_acc = d_dec = distance_in_mm / 2
            T_const = 0
            T_acc = T_dec = math.sqrt(2 * d_acc / self.accel)
            v_vrh = self.accel * T_acc

        t = t0 = robot.getTime()
        t1 = t0 + T_acc
        t2 = t1 + T_const
        t3 = t2 + T_dec
        w_vrh = v_vrh / self.wheel_radius

        while t < t3:
            robot.step(TIME_STEP)
            t = robot.getTime()

            if(t <= t1):
                v_ref = self.accel * (t - t0)
            elif(t <= t2):
                v_ref = v_vrh
            elif(t <= t3):
                v_ref = v_vrh - self.accel * (t - t2)

            r_wheel_vel = v_ref / self.wheel_radius
            l_wheel_vel = r_wheel_vel

            if direction == 'forward':
                self.left_wheel.setVelocity(l_wheel_vel)
                self.right_wheel.setVelocity(r_wheel_vel)
            elif direction == 'backward':
                self.left_wheel.setVelocity(-l_wheel_vel)
                self.right_wheel.setVelocity(-r_wheel_vel)
            elif direction == 'left':
                self.left_wheel.setVelocity(-l_wheel_vel)
                self.right_wheel.setVelocity(r_wheel_vel)
            elif direction == 'right':
                self.left_wheel.setVelocity(l_wheel_vel)
                self.right_wheel.setVelocity(-r_wheel_vel)

        #Rotation after left or right turn
        if rotation_angle != 0:
            self.rotate(rotation_angle)

        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)

    def rotate(self, angle):
        wheel_distance = self.L
        wheel_circumference = 2 * math.pi * self.wheel_radius
        distance_to_rotate = (wheel_distance * math.pi * angle) / 360
        rotations = distance_to_rotate / wheel_circumference

        if angle > 0:
            self.left_wheel.setPosition(rotations)
            self.right_wheel.setPosition(-rotations)
        else:
            self.left_wheel.setPosition(-rotations)
            self.right_wheel.setPosition(rotations)

# get the time step of the current world
timestep = int(robot.getBasicTimeStep())

myRobot = DifferentialDriveRobot() 

class WebotsThread(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot 
        self.robot.orientation = 'vertical'

    def run(self):
        while robot.step(TIME_STEP) != -1:
            if self.robot.commands:
                command_str = self.robot.commands.pop(0)
                action = command_str[0]
                #Orientation
                if self.robot.orientation == 'horizontal':
                    if action in ['f', 'p']:
                        distance = 65.14
                    elif action in ['l', 'r']:
                        self.robot.orientation = 'vertical'
                        distance = 65.14
                elif self.robot.orientation == 'vertical':
                    if action in ['f', 'p']:
                        distance = 64
                    elif action in ['l', 'r']:
                        self.robot.orientation = 'horizontal'
                        distance = 64
                #Movements
                if action == 'f':
                    self.robot.move(distance, 'forward')
                    time.sleep(1)
                elif action == 'p':
                    self.robot.move(distance, 'backward')
                    time.sleep(1)
                elif action == 'l':
                    self.robot.move(138, 'left')  
                    time.sleep(1)
                    self.robot.move(distance+3, 'forward')
                    time.sleep(1)
                elif action == 'r':
                    self.robot.move(138, 'right')  
                    time.sleep(1)
                    self.robot.move(distance+3, 'forward')
                    time.sleep(1)
    
webots_thread = WebotsThread(DifferentialDriveRobot())
webots_thread.start()

def sub_callback(data):
    command_str = data.data
    action = command_str[0]
    print("Action:", action)
    webots_thread.robot.commands.append(data.data)

rospy.init_node('motion_driver')
rospy.Subscriber('turtlebot_motion', msg.String, sub_callback)
rospy.spin()