'''
PYTHON SUB NODE
- Subscribes to topic /trajectory to get (x, y) position info from command line
'''

import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import XYCoord

class CommandListener(Node):
    def __init__(self):
        super().__init__('path_generator')

        # Publisher to send accumulated received positions to node
        self.trajectoryPub = self.create_publisher(XYCoord, '/trajectory', 10)
        
        # Subscriber to receive positions from topic /trajectory        
        self.positionSub = self.create_subscription(XYCoord, '/position', self.commandCallback, 20)
        self.trajectory = []
        self.pointCounter = 0
        self.nPoints = 4
        self.waiting = True

    def commandCallback(self, msg):
        # Get position and append to trajectory
        xyPos = (msg.x, msg.y)
        print('Position received:', xyPos)
        self.trajectory.append(xyPos)
        self.pointCounter += 1

        # Resume program after 3 position acquired
        if self.waiting and self.pointCounter >= self.nPoints:
            self.waiting = False

            # Print trajectory
            print('Trajectory positions:')
            for pos in self.trajectory:
                # Publish positions to controller node
                print(pos)
                msg = XYCoord()
                msg.x = pos[0]
                msg.y = pos[1]
                self.trajectoryPub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create and run node
    commandListener = CommandListener()
    
    try:
        rclpy.spin(commandListener)
    except SystemExit:
        pass

    # Destroy node
    commandListener.destroy_node()
    rclpy.shutdown()

if __name__=='__name__':
    main()
