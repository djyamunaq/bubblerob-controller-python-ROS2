'''
PYTHON PUB/SUB NODE
- Controller for Coppelia Sim two wheels robot with distance sensor
- Robot goes follows path given by user in CLI
- Velocity controlled by ROS2 parameter by user in CLI
'''

from time import time
from random import randint
import numpy as np
import math
    # Ros2 Node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
    # tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
    # Custom interface
from tutorial_interfaces.msg import XYCoord


class CopSimController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameter to control max velocity
        self.declare_parameter('max_vel', 2.0)

        # tf2 listener 
            # Declare and acquire `target_frame` parameter
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf = None

        # Publishers -> wheels control
        self.leftMotorSpeedPub = self.create_publisher(Float64, 'leftMotorSpeed', 10)
        self.rightMotorSpeedPub = self.create_publisher(Float64, 'rightMotorSpeed', 10)

        # Define timer with period 0.05 (Freq of 20 Hz)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscribers
            # Receive info from sensor trigger
        self.subSensor = self.create_subscription(Float64, 'sensorTrigger', self.sensorCallback, 10)
            # Receive info from simulation time
        self.subSimulationTime = self.create_subscription(Float64, 'simTime', self.simulationTimeCallback, 10)
            # Receive info from positions
        self.subTrajectory = self.create_subscription(XYCoord, 'trajectory', self.trajectoryCallback, 10)

        # Sensor trigger initially not activated
        self.sensorTrigger = False 
        
        # Time starts at 0
        self.currentTime = 0        
        self.simulationTime = 0
                
        # Linear/angular velocity constraints 
        self.maxVel = self.get_parameter('max_vel').get_parameter_value().double_value
        self.angle = -1
        self.angle_correction = 0.5
        # Position variables
        self.initial_pos = None
        self.trajectory = []
        self.curr_pos = None
        self.next_pos = None
        # Trajectory control variables
        self.traj_started = False
        self.aligned = False
        self.arrived = False

    def trajectoryCallback(self, msg):
        pos = np.array([msg.x, msg.y, self.initial_pos[2], 1])
        print('Position received:', (msg.x, msg.y))
        self.trajectory.append(pos)

    # Sensor Subscription Callbacks
    def sensorCallback(self, msg):
        # Update current time
        self.currentTimeUpdated = time()
        # Update sensor state
        self.sensorTrigger = msg.data

    # Simulation Time Subscription Callbacks
    def simulationTimeCallback(self, simTime):
        # Update simulation time
        self.simulationTime = simTime.data
        # Print simulation time
        # print('Simulation time:', self.simulationTime)

    # Timer callback -> read robot current TF, calculate next position and send correspondent robot wheels velocity
    def timer_callback(self):
        self.maxVel = self.get_parameter('max_vel').get_parameter_value().double_value

        try:
            self.tf = self.tf_buffer.lookup_transform('world', 'ros2InterfaceControlledBubbleRob', rclpy.time.Time())
        except:
            self.tf = None
        
        self.movePoint2Point()


    def movePoint2Point(self):
        # Check if theres TF new info
        if self.tf is not None:
            # Checks if its the first TF received to set initial position
            if not self.traj_started:
                self.traj_started = True
                # Get initial position
                self.initial_pos = np.array([self.tf.transform.translation.x, self.tf.transform.translation.y, self.tf.transform.translation.z, 1])
                self.curr_pos = self.initial_pos

            # Robot local frame translation relative to world frame            
            p = np.array([self.tf.transform.translation.x, self.tf.transform.translation.y, self.tf.transform.translation.z, 1])
            # Quaternion correspondent to robot local frame orientation relative to world frame
            Q = np.array([self.tf.transform.rotation.x, self.tf.transform.rotation.y, self.tf.transform.rotation.z, self.tf.transform.rotation.w])
            
            if self.next_pos is not None:
                # T = wTr -> Transformation matrix from robot local frame to world frame
                T = self.getTransformationMatrix(p, Q)
                T_inv = np.linalg.inv(T)
                
                # Goal position in robot frame
                Xd_r = np.dot(T_inv, self.next_pos)

                # Checks if robot arrived
                if np.linalg.norm(Xd_r[0:3]) < 1e-2:
                    self.arrived = True

                angle = np.arctan2(Xd_r[1], Xd_r[0])

                if abs(angle) < 1e-1:
                    self.aligned = True

                self.angle = angle
                self.curr_pos = p


        # Define float message to publish to wheels velocity topic
        msg_left = Float64()
        msg_right = Float64()

        # If arrived in goal, set velocities to 0
        if self.arrived or self.next_pos is None:
            self.next_pos = None
            # Check if there is a next goal
            if len(self.trajectory) > 0:
                self.next_pos = self.trajectory.pop(0)
                self.arrived = False 
            else:
                msg_left.data = 0.0
                msg_right.data = 0.0
        # If robot not aligned to trajectory, correct with velocities to move cw/ccw
        elif not self.aligned:
            if self.angle > 0:
                msg_left.data = -self.angle_correction
                msg_right.data = self.angle_correction
            else:
                msg_left.data = self.angle_correction
                msg_right.data = -self.angle_correction
        # If robot didn't arrive and is aligned in trajectory, go forward
        else:
            msg_left.data = self.maxVel
            msg_right.data = self.maxVel

        # Consider robot is not aligned to trajectory and test in next TF received 
        self.aligned = False

        # Publish velocities
        self.leftMotorSpeedPub.publish(msg_left)
        self.rightMotorSpeedPub.publish(msg_right)

    # Build transformation matrix T from translation vector and quaternion
    def getTransformationMatrix(self, p, Q):
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]

        # Get euler angles from quaternion 
        x, y, z = self.euler_from_quaternion(q0, q1, q2, q3)

        # Mount rotation matrices from euler angles
        Rz = np.array([ [np.cos(z),     -np.sin(z),     0,      0],
                        [np.sin(z),     np.cos(z),      0,      0],
                        [0,             0,              1,      0],
                        [0,             0,              0,      1]])

        
        Rx = np.array([ [1,     0,              0,              0],
                        [0,     np.cos(x),      -np.sin(x),     0],
                        [0,     np.sin(x),      np.cos(x),      0],
                        [0,     0,              0,              1]])

        Ry = np.array([ [np.cos(y),     0,      np.sin(y),      0],
                        [0,             1,      0,              0],
                        [-np.sin(y),    0,      np.cos(y),      0],
                        [0,             0,      0,              1]])

        # Mount Transformation Matrix from rotation matrices
        T = np.dot(np.dot(Ry, Rx), Rz)
        # Add translation component
        T[0, 3] = p[0]
        T[1, 3] = p[1]
        T[2, 3] = p[2]
        T[3, 3] = p[3]

        return T

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)

    # Create and run node
    cop_sim_controller_pubsub = CopSimController()
    rclpy.spin(cop_sim_controller_pubsub)

    # Destroy and shutdown node
    CopSimController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()