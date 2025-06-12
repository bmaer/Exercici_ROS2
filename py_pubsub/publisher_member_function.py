# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from turtlesim.srv import SetPen
from functools import partial

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.node = Node
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) #create_publisher(<type>, <nom_topic>, <buffer>)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.Vx = 0.0
        self.Vy = 0.0
        self.x1 = 0.0
        self.y1 = 0.0
        self.px = 0.0
        self.py = 0.0
    def timer_callback(self):
        msg1 = Twist()

        msg1.linear.x = self.Vx
        msg1.linear.y = self.Vy
        msg1.linear.z = 0.0
        self.publisher_.publish(msg1)

        #self.get_logger().info('Publishing: "%s"' % msg1.data)

    def set_Pen_Service(self, r, g, b, off, width):
        client = self.create_client(SetPen, f"/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    def callback_set_pen(self, future):

        response = future.result()


    def listener_callback(self, msg):
        self.px = msg.x
        self.py = msg.y
        self.Vx = self.x1 - msg.x
        self.Vy = self.y1 - msg.y
    def setXY1(self, xp, yp):
        self.x1 = xp
        self.y1 = yp
    def getPX(self):
        return self.px
    def getPY(self):
        return self.py


def main(args=None):
    estate = 0
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.set_Pen_Service(0,0,0, 1, 0)
    while(True):
        if(estate == 0):
            minimal_publisher.setXY1(1.0, 1.0)
            estate = 1

        if(estate == 1 and (minimal_publisher.getPX() < 1.3 and minimal_publisher.getPX() >= 1.0 and minimal_publisher.getPY() < 1.3 and minimal_publisher.getPY() >= 1.0)):
            minimal_publisher.set_Pen_Service(255, 255, 0, 0, 5)
            estate = 2
            minimal_publisher.setXY1(4.5,4.5)
        if (estate == 2):
            minimal_publisher.setXY1(4.5, 4.5)
        if(estate == 2 and (minimal_publisher.getPX() < 4.7 and minimal_publisher.getPX() >= 4.5 and minimal_publisher.getPY() < 4.7 and minimal_publisher.getPY() >= 4.5)):
            estate = 3
            minimal_publisher.setXY1(10.0, 1.0)
        if (estate == 3):
            minimal_publisher.setXY1(10.0, 1.0)

        if (estate == 3 and (minimal_publisher.getPX() < 10.2 and minimal_publisher.getPX() >= 10.0 and minimal_publisher.getPY() < 1.2 and minimal_publisher.getPY() >= 1.0)):
            estate = 4
            minimal_publisher.setXY1(7.75, 2.25)
        if (estate == 4):
            minimal_publisher.setXY1(7.75, 2.25)
        if (estate == 4 and (minimal_publisher.getPX() < 7.95 and minimal_publisher.getPX() >= 7.75 and minimal_publisher.getPY() < 2.45 and minimal_publisher.getPY() >= 2.25)):
            estate = 5
            minimal_publisher.setXY1(4.5, 1.0)
        if (estate == 5):
            minimal_publisher.setXY1(4.5, 1.0)
        if (estate == 5 and (minimal_publisher.getPX() < 4.7 and minimal_publisher.getPX() >= 4.5 and minimal_publisher.getPY() < 1.2 and minimal_publisher.getPY() >= 1.0)):
            estate = 6
            minimal_publisher.setXY1(10.0, 1.0)
        if (estate == 6):
            minimal_publisher.setXY1(10.0, 1.0)
        if (estate == 6 and (minimal_publisher.getPX() < 10.2 and minimal_publisher.getPX() >=10.0 and minimal_publisher.getPY() < 2.45 and minimal_publisher.getPY() >= 2.25)):
            estate = 7
            minimal_publisher.setXY1(10.0, 0.0)
        if (estate == 7):
            minimal_publisher.setXY1(10.0, 1.0)
        if (estate == 7 and (minimal_publisher.getPX() < 10.8 and minimal_publisher.getPX() >=11.0 and minimal_publisher.getPY() < 1.2 and minimal_publisher.getPY() >= 1.0)):
            estate = 8
            minimal_publisher.setXY1(10.0, 1.0)
        if (estate == 8):
            minimal_publisher.setXY1(10.0, 1.0)
        print(estate)
        rclpy.spin_once(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
